import gc
import importlib
import os
import subprocess
import sys
import textwrap
import weakref

import pytest


def test_public_cpp_api_without_python_can(tmp_path):
    script = textwrap.dedent("""
        import importlib.abc
        import sys

        class BlockPythonCan(importlib.abc.MetaPathFinder):
            def find_spec(self, fullname, path=None, target=None):
                if fullname == "can" or fullname.startswith("can."):
                    raise ModuleNotFoundError("optional python-can is absent", name="can")

        sys.meta_path.insert(0, BlockPythonCan())
        import agx_motor_sdk
        from agx_motor_sdk import ArmDriver, create_can_port
        from agx_motor_sdk import _native
        from agx_motor_sdk.mechanism import JointGroup

        port = create_can_port(
            channel="unused-no-CAN", backend="cpp", auto_connect=False,
            fallback_python_can=False,
        )
        assert isinstance(port._port, _native.SocketCanPort)
        assert not port.is_connected()
        received = []
        port.set_callback(received.append)
        port._on_native_frame(0x511, b"12345678")
        message = received.pop()
        assert message.arbitration_id == 0x511
        assert message.data == bytearray(b"12345678")
        assert message.dlc == 8
        assert not message.is_extended_id
        driver = ArmDriver()
        driver.handle_rx_once(msg=message)
        driver.close()
        port.close()
        assert JointGroup.__module__ == "agx_motor_sdk.mechanism.joint_group"
        from agx_motor_sdk.motor import (
            ArmMotor, MotorBase, MotorDriver, Rb200Driver, Rb200Motor,
        )
        assert issubclass(ArmDriver, MotorDriver) and issubclass(Rb200Driver, MotorDriver)
        assert issubclass(ArmMotor, MotorBase) and issubclass(Rb200Motor, MotorBase)
        assert not hasattr(Rb200Driver, "set_target_current")
        assert not hasattr(Rb200Motor, "set_mit")
        assert "can" not in sys.modules
        try:
            from agx_motor_sdk.can import PythonCanPort
        except ModuleNotFoundError as error:
            assert error.name == "can"
        else:
            raise AssertionError("optional backend must require python-can")
    """)
    environment = dict(os.environ, PYTHONPATH="")
    result = subprocess.run(
        [sys.executable, "-c", script], cwd=tmp_path, env=environment,
        capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.fixture(scope="module")
def native():
    directory = os.environ.get("AGX_SDK_TEST_NATIVE_DIR") or os.environ.get("NATIVE_DIR")
    if not directory:
        pytest.skip("set AGX_SDK_TEST_NATIVE_DIR to the built native module directory")
    with pytest.MonkeyPatch.context() as environment:
        library = os.environ.get("ARM_LIB")
        if library:
            environment.setenv("AGX_MOTOR_SDK_ARM_LIB", library)
        library = os.environ.get("RB200_LIB")
        if library:
            environment.setenv("AGX_MOTOR_SDK_RB200_LIB", library)
        environment.syspath_prepend(directory)
        yield importlib.import_module("_native")


class FakePort:
    def __init__(self):
        self.callback = None
        self.detached = 0
        self.connected = False
        self.connect_count = 0
        self.close_count = 0

    def connect(self):
        self.connected = True
        self.connect_count += 1

    def close(self):
        self.connected = False
        self.close_count += 1

    def is_connected(self):
        return self.connected

    def set_callback(self, callback):
        self.callback = callback

    def clear_callback(self):
        self.callback = None
        self.detached += 1


def test_driver_keeps_port_until_destruction(native):
    port = FakePort()
    reference = weakref.ref(port)
    driver = native.ArmDriver(port)
    del port
    gc.collect()
    assert reference() is not None
    del driver
    gc.collect()
    assert reference() is None


def test_attach_detaches_previous_port(native):
    previous = FakePort()
    replacement = FakePort()
    driver = native.ArmDriver(previous)
    driver.attach_can_port(replacement)
    assert previous.callback is None
    assert previous.detached > 0
    assert replacement.callback is not None
    driver.close()
    assert replacement.callback is None


def test_registry_releases_shared_driver(native):
    config = native.load_module_config(
        os.path.join(os.path.dirname(__file__), "../examples/mechanism/robot_motors.yaml"),
        "head",
    ).joints[0]
    port = FakePort()
    reference = weakref.ref(port)
    driver = native.ArmDriver(port)
    native.MotorRegistry.acquire("fake0", driver, config.node_id, config)
    del port, driver
    gc.collect()
    assert reference() is not None
    native.MotorRegistry.clear()
    gc.collect()
    assert reference() is None


def test_callback_adapter_does_not_retain_port(native):
    port = FakePort()
    driver = native.ArmDriver(port)
    adapter = port.callback
    reference = weakref.ref(port)
    del driver, port
    gc.collect()
    assert reference() is None
    adapter(None)


def test_socket_callback_replace_and_close(native):
    port = native.SocketCanPort("unused-no-CAN")
    for _ in range(100):
        port.set_callback(lambda *_args: None)
        port.set_callback(None)
    port.close()


@pytest.mark.parametrize(
    "factory",
    ["constructor", "constructor_channel", "yaml", "yaml_channel", "factory", "factory_channel"],
)
@pytest.mark.parametrize("explicit_close", [False, True])
def test_group_injected_port_lifecycle(native, factory, explicit_close):
    example_yaml = os.path.join(
        os.path.dirname(__file__), "../examples/mechanism/robot_motors.yaml"
    )
    config = native.load_module_config(example_yaml, "head")
    channel = "fake-group-lifecycle"
    if factory == "constructor":
        group = native.JointGroup(config)
    elif factory == "constructor_channel":
        group = native.JointGroup(config, channel)
    elif factory == "yaml":
        group = native.JointGroup.from_yaml(example_yaml, "head")
    elif factory == "yaml_channel":
        group = native.JointGroup.from_yaml(example_yaml, "head", channel)
    elif factory == "factory":
        group = native.create_joint_group(config)
    else:
        group = native.create_joint_group(config, channel)

    assert isinstance(group, native.JointGroup)
    port = FakePort()
    reference = weakref.ref(port)
    for can_channel in group.can_channels:
        group.attach_can_port(port, can_channel)
    assert not group.is_connected()
    assert group.connect()
    assert group.is_connected()
    assert port.connect_count == 1
    assert port.callback is not None

    if explicit_close:
        group.close()
        assert not group.is_connected()
        assert port.callback is None
        assert port.detached > 0
        assert port.close_count == 0
        group.close()

    del port
    gc.collect()
    assert reference() is not None
    del group
    gc.collect()
    assert reference() is None

def test_group_motor_dispatches_by_protocol(native, tmp_path):
    if not os.environ.get("AGX_MOTOR_SDK_RB200_LIB") and not os.environ.get("RB200_LIB"):
        pytest.skip("set AGX_MOTOR_SDK_RB200_LIB for the mixed-protocol check")
    yaml_path = tmp_path / "mixed.yaml"
    yaml_path.write_text(textwrap.dedent("""
        mixed:
          can_channel: fake-mixed
          protocol: arm
          joints:
            - joint_name: arm_joint
              node_id: 1
            - joint_name: rb200_joint
              node_id: 2
              protocol: rb200
    """))
    group = native.JointGroup.from_yaml(str(yaml_path), "mixed")
    port = FakePort()
    group.attach_can_port(port, "fake-mixed")
    assert group.connect()
    arm, rb200, invalid = group.motor(0), group.motor(1), group.motor(99)
    assert isinstance(arm, native.ArmMotor) and arm.protocol == "arm"
    assert isinstance(rb200, native.Rb200Motor) and rb200.protocol == "rb200"
    assert not isinstance(invalid, (native.ArmMotor, native.Rb200Motor))
    assert not invalid.is_valid
    assert rb200.get_low_speed_feedback() is None
    assert not group.set_mit(1, 0.0, 0.0, 0.0)
    group.close()
    native.MotorRegistry.clear()
