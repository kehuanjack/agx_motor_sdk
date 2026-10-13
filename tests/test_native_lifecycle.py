import gc
import importlib
import os
import weakref

import pytest


@pytest.fixture(scope="module")
def native():
    directory = os.environ.get("AGX_SDK_TEST_NATIVE_DIR") or os.environ.get("NATIVE_DIR")
    if not directory:
        pytest.skip("set AGX_SDK_TEST_NATIVE_DIR to the built native module directory")
    with pytest.MonkeyPatch.context() as environment:
        library = os.environ.get("ARM_LIB")
        if library:
            environment.setenv("AGX_MOTOR_SDK_ARM_LIB", library)
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
    native.MotorRegistry.acquire_arm("fake0", driver, config.node_id, config)
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
        group = native.ArmJointGroup(config)
    elif factory == "constructor_channel":
        group = native.ArmJointGroup(config, channel)
    elif factory == "yaml":
        group = native.ArmJointGroup.from_yaml(example_yaml, "head")
    elif factory == "yaml_channel":
        group = native.ArmJointGroup.from_yaml(example_yaml, "head", channel)
    elif factory == "factory":
        group = native.create_joint_group(config)
    else:
        group = native.create_joint_group(config, channel)

    assert isinstance(group, native.ArmJointGroup)
    assert isinstance(group, native.JointGroupBase)
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