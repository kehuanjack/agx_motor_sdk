# agx_motor_sdk

Python and C++ SDK for AgileX arm CAN motors. Control one motor with `ArmDriver`,
or configure multiple joints with `ArmJointGroup`.

Supports Linux SocketCAN at **1 Mbps**, with motor node IDs **1..15**. Prebuilt
protocol libraries are available for **x86_64** and **aarch64**, built on Ubuntu
22.04. Compatibility with other Linux versions has not yet been verified.

| I want to... | Start here |
|--------------|------------|
| Install and connect to one motor | [Python quick start](#python-quick-start) |
| Configure multiple joints | [Joint groups](#joint-groups), then [YAML configuration](YAML_CONFIG.md) |
| Use C++ | [C++ installation](#c-installation) |
| Use ROS 2 | [ROS 2 installation](#ros-2-installation) |
| Look up methods and parameters | [API reference](API.md) |
| Install offline or choose a backend | [Advanced usage](docs/ADVANCED.md) |
| Run software checks | [Testing](docs/TESTING.md) |

## Python quick start

### 1. Install

You need Python 3.8 or newer, a C++17 compiler and CMake 3.14 or newer. On Ubuntu:

```bash
sudo apt update
sudo apt install -y build-essential cmake libasio-dev libyaml-cpp-dev python3-dev can-utils iproute2
git clone https://github.com/kehuanjack/agx_motor_sdk.git
cd agx_motor_sdk
uv venv --python 3.11
uv pip install ".[python-can]"
```

These commands use [uv](https://docs.astral.sh/uv/getting-started/installation/)
to isolate Python dependencies. Installation automatically downloads and verifies
the matching protocol library; network access is required.

### 2. Set up CAN

Connect the CAN adapter and power the motor. Replace `can0` with your interface:

```bash
ip -brief link show
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

The interface must be up at 1 Mbps. Check motor power, CAN wiring, termination
and node ID. Do not change an interface already in use.

### 3. Check communication

Run Python in the environment you installed into:

```bash
uv run --no-sync python
```

Query the motor version. This example does not enable or move the motor:

```python
from agx_motor_sdk import ArmDriver, create_can_port

bus = create_can_port(channel="can0", backend="cpp", fallback_python_can=False)
driver = ArmDriver(bus)
try:
    version = driver.get_version(6, timeout=0.5)  # Replace 6 with your node ID.
    if version is None:
        print("No version reply: check power, wiring, bitrate and node ID")
    else:
        print(version)
finally:
    driver.close()
    bus.close()
```

The `cpp` backend receives feedback automatically; no Python receive loop is
needed. For `python-can`, see [backend selection](docs/ADVANCED.md#python-can-backend).

## Joint groups

Use YAML to define joint names, node IDs, directions and position limits. Start
with the [configuration example](YAML_CONFIG.md#minimal-example), then load its
top-level module key:

```python
from agx_motor_sdk.mechanism import ArmJointGroup

group = ArmJointGroup.from_yaml("robot_motors.yaml", "head")
try:
    if not group.connect():
        raise RuntimeError("Cannot connect: check the CAN interface")
    index = group.index_of("head_pitch_joint")
    if not group.is_valid_joint_index(index):
        raise ValueError("Joint name not found in configuration")
    print(group.motor(index).get_state())  # None until feedback has arrived.
finally:
    group.close()
```

This example reads cached feedback without requesting motion. Group commands use
joint-space values; `ArmDriver` commands use motor-space values. Positions are
radians and velocities are radians per second. See [control methods](API.md#mechanism-layer).

**Before enabling or commanding motion, secure the mechanism and check targets
and limits. Enabling can apply torque; closing the SDK does not disable the motor.**

## C++ installation

From the repository root, with the dependencies above installed:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DPYTHON_BINDING=OFF
cmake --build build -j4
cmake --install build --prefix "$HOME/.local"
```

In your application's CMake configuration:

```cmake
find_package(agx_motor_sdk REQUIRED)
target_link_libraries(your_target PRIVATE agx_motor_sdk)
```

Configure your application with `-DCMAKE_PREFIX_PATH="$HOME/.local"` if CMake
cannot find the SDK. Include `agx_motor_sdk/motor/arm_driver.hpp` for motor control
or `agx_motor_sdk/mechanism/joint_group.hpp` for joint groups.

## ROS 2 installation

Place this repository under the workspace's `src/` directory, then run from the
workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select agx_motor_sdk --symlink-install
source install/setup.bash
python3 -c "import agx_motor_sdk; print(agx_motor_sdk.__version__)"
```

This installs both the C++ library and Python package. A separate `pip install`
is not needed.

## Examples

Run Python examples from the repository root using your installed environment:

| Task | Python example | C++ binary under `build/bin/` |
|------|----------------|--------------------------------|
| Read position, velocity and current | `examples/arm/get_high_speed.py` | `agx_motor_sdk_arm_get_high_speed_demo` |
| Read voltage, temperatures and status | `examples/arm/get_low_speed.py` | `agx_motor_sdk_arm_get_low_speed_demo` |
| Enable | `examples/arm/enable.py` | `agx_motor_sdk_arm_enable_demo` |
| Disable | `examples/arm/disable.py` | `agx_motor_sdk_arm_disable_demo` |
| Move to zero | `examples/arm/go_zero.py` | `agx_motor_sdk_arm_go_zero_demo` |
| Control joints from YAML | `examples/mechanism/head_from_yaml.py` | `agx_motor_sdk_arm_head_from_yaml_demo` |

Motion examples can enable and move the mechanism. Review their configuration
before running them. To read feedback from node 6 on `can0`:

```bash
uv run --no-sync python examples/arm/get_high_speed.py can0 6
./build/bin/agx_motor_sdk_arm_get_high_speed_demo can0 6
```

## Troubleshooting

| Symptom | Check first |
|---------|-------------|
| Protocol download fails | Network/release access; see [offline installation](docs/ADVANCED.md#offline-installation) |
| Protocol library cannot load | CPU architecture; set an absolute `AGX_MOTOR_SDK_ARM_LIB` path for a custom location |
| CAN connection fails | Interface name, permissions, interface state and bitrate |
| Version query returns `None` | Motor power, wiring, termination, node ID and timeout; receive loop when using `python-can` |
| Cached feedback returns `None` | No matching feedback has arrived yet; cached reads do not request replies |
| YAML is rejected | Module key, field names and [validation rules](YAML_CONFIG.md#validation-rules) |
| Position direction or limits look wrong | [Units and polarity](YAML_CONFIG.md#units-and-polarity) |

For ports and callbacks, see [safe shutdown](docs/ADVANCED.md#safe-shutdown).

## License

SDK source: [Apache License 2.0](LICENSE). Prebuilt protocol libraries are proprietary and for
use with AgileX motors only.