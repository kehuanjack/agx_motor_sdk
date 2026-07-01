# agx_motor_sdk

AgileX motor driver SDK with two CAN motor protocols. Names `arm` / `chassis` refer to **motor firmware protocols**.

| Protocol | CAN bitrate | Driver |
|----------|-------------|--------|
| arm | 1 Mbps | `ArmDriver` / `agx::motor::ArmDriver` |
| chassis | 500 kbps | `ChassisDriver` / `agx::motor::ChassisDriver` |

| Layer | Description |
|-------|-------------|
| Protocol | Prebuilt `libagx_motor_protocol_arm` / `libagx_motor_protocol_chassis` |
| Transport | C++ socketcan by default; optional `python-can` |

**API reference**: [API.md](API.md) — C++/Python method tables, types, CAN layer, and naming map.

## Requirements

| Component | Version / notes |
|-----------|-----------------|
| OS | Linux (socketcan); Windows/macOS supported for **python-can** backend only |
| CMake | ≥ 3.14 (standalone C++ build, pip native extension) |
| C++ compiler | C++17 |
| Python | ≥ 3.8 (optional; for Python API and pip install) |
| CAN | SocketCAN interface (e.g. `can0`); `iproute2` / `can-utils` for setup scripts |
| Network | First configure / `pip install` downloads protocol `.so` from GitHub Release (see below) |

Protocol binaries are **not** in this repo. CMake / `pip install` fetch them from
[`kehuanjack/agx_motor_ctrl` Releases](https://github.com/kehuanjack/agx_motor_ctrl/releases)
using **`__protocol_version__`** in `agx_motor_sdk/version.py` (Release tag `v{protocol_version}`, e.g. `v1.1.0`).
**SDK version** (`__version__`, `package.xml`) may move independently when only the wrapper/API changes.
Override with `AGX_MOTOR_SDK_PROTOCOL_VERSION`, `AGX_MOTOR_SDK_GITHUB_REPO`, or place files under `agx_motor_sdk/protocol/`.

## Get the source

```bash
git clone https://github.com/kehuanjack/agx_motor_sdk.git
cd agx_motor_sdk
```

Install from a Git URL (no clone needed):

```bash
pip install "git+https://github.com/kehuanjack/agx_motor_sdk.git"
# optional python-can backend (pip ≥ 21; use direct URL syntax for extras)
pip install "agx_motor_sdk[python-can] @ git+https://github.com/kehuanjack/agx_motor_sdk.git"
```

Pin a tag or branch:

```bash
pip install "git+https://github.com/kehuanjack/agx_motor_sdk.git@v1.1.0"
pip install "agx_motor_sdk[python-can] @ git+https://github.com/kehuanjack/agx_motor_sdk.git@v1.1.0"
```

### ROS 2 / colcon workspace

Clone into your workspace `src/` (package root = repo root):

```bash
cd ~/ros2_ws/src
git clone https://github.com/kehuanjack/agx_motor_sdk.git
cd ~/ros2_ws
colcon build --packages-select agx_motor_sdk
source install/setup.bash
```

## Install

### CMake

```bash
cd agx_motor_sdk
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
cmake --build . -j"$(nproc)"
```

CMake downloads protocol libraries from GitHub Release into `agx_motor_sdk/protocol/` and copies them to `build/protocol/` for demos. To skip download:

```bash
cmake .. -DAGX_MOTOR_SDK_FETCH_PROTOCOL=OFF
# or
export AGX_MOTOR_SDK_SKIP_DOWNLOAD=1
```

Optional install:

```bash
cmake --install . --prefix /usr/local
# cmake --install . --prefix "$HOME/.local"
```

Build outputs:

| Artifact | Path |
|----------|------|
| SDK library | `build/libagx_motor_sdk.so` |
| Demo binaries | `build/bin/agx_motor_sdk_arm_enable_demo` |
| Protocol libs | `build/protocol/libagx_motor_protocol_*.so` |

Run demos from `build/` to auto-locate `build/protocol/`, or set `AGX_MOTOR_SDK_ARM_LIB` / `AGX_MOTOR_SDK_CHASSIS_LIB`.

Use in another CMake project:

```cmake
find_package(agx_motor_sdk REQUIRED)
target_link_libraries(your_target PRIVATE agx_motor_sdk)
```

### colcon (ROS workspace)

See [ROS 2 / colcon workspace](#ros-2--colcon-workspace) above for cloning. Protocol libraries are fetched at configure time (same `protocol_fetch.py` as standalone CMake).

When built inside a ROS 2 workspace (`ament_cmake`), the package also installs the Python API (`agx_motor_sdk` + `_native` extension) via `ament_cmake_python`. **No `pip install` is required** for ROS Python nodes — `source install/setup.bash` is enough.

```bash
cd ~/ros2_ws
colcon build --packages-select agx_motor_sdk --symlink-install
source install/setup.bash
python3 -c "import agx_motor_sdk; print(agx_motor_sdk.__version__)"
```

C++ consumers in the same workspace still link `libagx_motor_sdk.so` from `install/agx_motor_sdk/lib/`. Python and C++ artifacts install to separate paths and do not conflict.

**Dependencies:** `ament_cmake`, `ament_cmake_python`, `python3-dev`, `pybind11-dev` (or ROS `pybind11_vendor`).

### pip (Python + C++ CAN backend)

From a local clone:

```bash
cd agx_motor_sdk
pip install .
pip install ".[python-can]"   # optional python-can backend
```

Or from GitHub: `pip install "git+https://github.com/kehuanjack/agx_motor_sdk.git"`; with extras: `pip install "agx_motor_sdk[python-can] @ git+https://github.com/kehuanjack/agx_motor_sdk.git"` (see [Get the source](#get-the-source)).

`pip install` downloads both protocol libraries into `agx_motor_sdk/protocol/`.

## CAN setup (Linux)

```bash
sudo ./scripts/can_up_1m.sh      # 1 Mbps
sudo ./scripts/can_up_500k.sh    # 500 kbps
```

Or manually:

```bash
sudo ip link set can0 up type can bitrate 1000000   # 1 Mbps
sudo ip link set can0 up type can bitrate 500000    # 500 kbps
```

## Python quick start

`ArmDriver` / `ChassisDriver` attach to the CAN port at construction and parse RX via `set_callback` → `handle_rx_once`. **How you must drive receive depends on the transport backend** (see table below).

| Backend | Select | RX mechanism | Python recv loop |
|---------|--------|--------------|------------------|
| **cpp** (default) | `AGX_MOTOR_SDK_CAN_BACKEND=cpp` or omit | C++ socketcan + background reader thread; frames delivered to the driver callback | **Not required** — `bus.recv()` is a no-op compatibility hook |
| **python-can** | `backend="python-can"` or `AGX_MOTOR_SDK_CAN_BACKEND=python-can` | Blocking `bus.recv()`; callback runs inside `recv()` | **Required** — run `recv()` in a loop (typically a daemon thread) |

### cpp backend (default)

No Python thread needed; the C++ layer reads CAN asynchronously.

```python
from agx_motor_sdk import ArmDriver, create_can_port

bus = create_can_port(channel="can0")  # default: cpp
driver = ArmDriver(bus)

try:
    node_id = 6
    print(driver.get_version(node_id, timeout=0.5))
    driver.set_enable(node_id, True)
finally:
    driver.close()
    bus.close()
```

### python-can backend

`recv()` must be polled; start a background loop before issuing driver commands.

```python
import threading
from agx_motor_sdk import ArmDriver, create_can_port

bus = create_can_port(channel="can0", backend="python-can")
driver = ArmDriver(bus)

stop = threading.Event()

def recv_loop():
    while not stop.is_set():
        try:
            bus.recv()
        except Exception:
            if stop.is_set():
                break

threading.Thread(target=recv_loop, daemon=True).start()

try:
    node_id = 6
    print(driver.get_version(node_id, timeout=0.5))
    driver.set_enable(node_id, True)
finally:
    stop.set()
    driver.close()
    bus.close()
```

Scripts under `examples/` include a recv thread so they work with **both** backends when `AGX_MOTOR_SDK_CAN_BACKEND=python-can`; with the default **cpp** backend that thread is redundant but harmless.

## Environment variables

| Variable | Purpose |
|----------|---------|
| `AGX_MOTOR_SDK_ARM_LIB` | Path to arm protocol library |
| `AGX_MOTOR_SDK_CHASSIS_LIB` | Path to chassis protocol library |
| `AGX_MOTOR_SDK_CAN_BACKEND` | `cpp` (default) or `python-can` |
| `AGX_MOTOR_SDK_SKIP_DOWNLOAD` | Skip protocol download during cmake / pip install |
| `AGX_MOTOR_SDK_PROTOCOL_VERSION` | Protocol Release tag version (default: `__protocol_version__` in `version.py`) |
| `AGX_MOTOR_SDK_GITHUB_REPO` | GitHub `owner/repo` for protocol Release assets (default: `kehuanjack/agx_motor_ctrl`) |
| `AGX_MOTOR_SDK_FETCH_PROTOCOL` | CMake option, default `ON` |

## Examples

### Scripts (`scripts/`)

Configure CAN before running examples:

| Script | Description |
|--------|-------------|
| `scripts/can_up_1m.sh` | Bring up `can0` at 1 Mbps |
| `scripts/can_up_500k.sh` | Bring up `can0` at 500 kbps |
| `scripts/find_all_can_port.sh` | List CAN interfaces and USB `bus-info` |
| `scripts/can_muti_activate.sh` | Activate multiple CAN ports by USB mapping (`can0`–`can4`, etc.) |

```bash
sudo ./scripts/can_up_1m.sh
python3 examples/arm/enable.py

sudo ./scripts/can_up_500k.sh
python3 examples/chassis/set_rpm.py

sudo ./scripts/can_muti_activate.sh
```

### Python (`examples/`)

| Script | Description |
|--------|-------------|
| `examples/arm/enable.py` | Read version and enable |
| `examples/arm/disable.py` | Read version and disable |
| `examples/arm/go_zero.py` | Enable, set profile velocity, go to zero [rad] |
| `examples/arm/get_high_speed.py` | Poll high-speed feedback [rad, rad/s, A] |
| `examples/arm/get_low_speed.py` | Poll low-speed feedback (voltage, temperature, status) |
| `examples/chassis/enable.py` | Enable |
| `examples/chassis/disable.py` | Disable |
| `examples/chassis/set_rpm.py` | Stiffness + profile velocity + target RPM |
| `examples/chassis/get_high_speed.py` | Poll high-speed feedback [INC, RPM, A] |
| `examples/chassis/get_low_speed.py` | Poll low-speed feedback |

```bash
python3 examples/arm/enable.py
python3 examples/chassis/set_rpm.py
```

### C++ (`sample/`, built to `build/bin/`)

| Binary | Description |
|--------|-------------|
| `agx_motor_sdk_arm_enable_demo` | Read version and enable |
| `agx_motor_sdk_arm_disable_demo` | Disable |
| `agx_motor_sdk_arm_go_zero_demo` | Enable and go to zero |
| `agx_motor_sdk_arm_get_high_speed_demo` | Poll high-speed feedback [rad] |
| `agx_motor_sdk_arm_get_low_speed_demo` | Poll low-speed feedback |
| `agx_motor_sdk_chassis_enable_demo` | Enable |
| `agx_motor_sdk_chassis_disable_demo` | Disable |
| `agx_motor_sdk_chassis_set_rpm_demo` | Stiffness + target RPM |
| `agx_motor_sdk_chassis_get_high_speed_demo` | Poll high-speed feedback [INC/RPM] |
| `agx_motor_sdk_chassis_get_low_speed_demo` | Poll low-speed feedback |

```bash
cd build
./bin/agx_motor_sdk_arm_enable_demo can0 6
```

## License

- **SDK source (Python / C++)**: [MIT](LICENSE). Copyright © Agilex Robotics Co., Ltd.
- **Prebuilt protocol libraries**: AgileX proprietary; for use with AgileX motors only.
