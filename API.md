# API Reference

Look up method names, units and return values here. For installation and a first
connection, start with [README](README.md). For joint settings, use
[YAML configuration](YAML_CONFIG.md).

| Area | Reference |
|------|-----------|
| Joint groups and per-joint control | [Mechanism layer](#mechanism-layer) |
| Single-motor commands and parameters | [ArmDriver](#armdriver-arm-protocol-1-mbps) |
| Feedback fields | [Shared types](#shared-types) |
| Port options and callbacks | [CAN transport](#can-transport) |
| Equivalent C++ method names | [Naming map](#c--python-naming-map) |

Node IDs are `1..15`. Positions are radians, velocities radians per second and
currents amperes. The default `cpp` backend receives automatically; `python-can`
needs a [receive loop](docs/ADVANCED.md#python-can-backend).

Cached reads return `None` / `std::nullopt` before matching feedback arrives and
do not request device replies. Blocking queries can return empty on timeout or
failure. A successful send-only call does not prove that the motor executed it.
Closing a driver or group does not disable motors.

---

## Python package exports

```python
from agx_motor_sdk import (
    ArmDriver,
    DriverStatus,
    HighSpeedFeedback,
    LowSpeedFeedback,
    VersionInfo,
    create_can_port,
    create_can_port_config,
    __version__,
)
```

C++ headers live under `include/agx_motor_sdk/`; namespace `agx::motor`.

---

## Mechanism layer

YAML-driven joint groups for multi-joint modules (`head`, `fold`, …). One module = one `ArmJointGroup`. Schema: [YAML_CONFIG.md](YAML_CONFIG.md). Examples: `examples/mechanism/`.

### Python imports

```python
from agx_motor_sdk.mechanism import (
    ArmJointGroup,
    GroupConfig,
    JointConfig,
    JointState,
    load_module_config,
    joint_names_from_config,
)
from agx_motor_sdk.motor import ArmMotorBase
```

### Construction

| Python | C++ |
|--------|-----|
| `ArmJointGroup(config)` | `ArmJointGroup(config)` |
| `ArmJointGroup(config, can_channel)` | `ArmJointGroup(config, can_channel)` |
| `ArmJointGroup.from_yaml(path, key)` | `ArmJointGroup::FromYaml(path, key)` |
| `ArmJointGroup.from_yaml(path, key, can)` | `ArmJointGroup::FromYaml(path, key, can)` |
| — | `CreateJointGroup(config)` → `unique_ptr<JointGroupBase>` (always `ArmJointGroup`) |
| — | `CreateArmJointGroup` |
| — | `CreateJointGroupFromYaml(path, key[, can])` |

YAML loading and direct group construction both validate configuration. See
[validation rules](YAML_CONFIG.md#validation-rules) and
[channel overrides](YAML_CONFIG.md#multiple-can-interfaces).

### Joint index semantics

| Symbol | C++ | Python | Value |
|--------|-----|--------|-------|
| Invalid index | `kInvalidJointIndex` (`types.hpp`) | `INVALID_JOINT_INDEX` on `ArmJointGroup` | `-1` |

- `index_of(name)` → index or `INVALID_JOINT_INDEX`.
- `is_valid_joint_index(index)` → `index ∈ [0, joint_count)`.
- `motor(index)` never raises: out-of-range returns a no-op `ArmMotorBase` with `is_valid=False`.

### Group methods (`ArmJointGroup`)

| Method | C++ | Description |
|--------|-----|-------------|
| `attach_can_port(can[, channel])` | `AttachCanPort` | Supply external `CanPort` before `connect`; group keeps the port alive |
| `connect()` | `Connect` | `bool`: whether all required CAN connections opened |
| `close()` | `Close` | Disconnect this group without closing externally supplied ports |
| `is_connected()` | `IsConnected` | `bool`: whether all required connections are available |
| `enable()` / `disable()` | `Enable` / `Disable` | All joints |
| `configure()` | `Configure` | Apply YAML profile / MIT to all joints |
| `joint_count` | `JointCount` | Number of joints |
| `joint_names` | `JointNames` | Names in YAML order |
| `protocol` | `protocol()` | Always `"arm"` |
| `can_channel` | `can_channel()` | Module default channel |
| `can_channels` | `CanChannels` | Distinct channels used by joints |
| `config` | `config()` | `GroupConfig` |
| `index_of(name)` | `IndexOf` | Name → index or `INVALID_JOINT_INDEX` |
| `is_valid_joint_index(i)` | `IsValidJointIndex` | Range check |
| `node_id(i)` | `NodeId` | CAN node ID (1–15) |
| `joint_can_channel(i)` | `JointCanChannel` | Resolved channel for joint |
| `joint_config(i)` | `GetJointConfig` | `JointConfig` |
| `set_target_position(i, pos)` | `SetTargetPosition` | Joint-space position [rad] (limits + polarity) |
| `get_state(i)` | `GetState` | `JointState` or `None` |
| `motor(index)` | `MotorArm` | `ArmMotorBase` (invalid index → no-op) |
| `set_mit(i, pos, vel, torque)` | `SetMit` | Joint-space MIT; needs YAML `mit_tf` / `mit_kp` / `mit_kd` |

The group keeps supplied ports alive until replacement or group destruction, even
after `close()`. Close external ports yourself after all users have stopped using
them. Stop motor commands before closing the group.
See [safe shutdown](docs/ADVANCED.md#safe-shutdown).

### Per-joint motor view (`ArmMotorBase`)

Wraps one joint with joint-space helpers plus `ArmDriver` methods.

| Method | Description |
|--------|-------------|
| `is_valid` | `True` for a real joint index |
| `protocol`, `node_id`, `can_channel`, `name`, `config` | Metadata |
| `set_enable([timeout])` | Enable motor |
| `set_target_position(pos)` | Joint-space position [rad] (limits + polarity) |
| `set_target_velocity(vel)` | Joint-space velocity [rad/s] (polarity) |
| `get_state()` | `JointState` (position, velocity, torque with polarity) |
| `configure()` | Apply this joint's YAML |
| `get_high_speed_feedback()` / `get_low_speed_feedback()` | Cached joint-space high-speed feedback; low-speed is raw driver cache |
| `set_mit`, `get_version`, `set_target_current`, `set_mit_control` / `set_mit_control_crc`, `set_collision_threshold`, `get_param` / `set_param` | See [ArmDriver](#armdriver-arm-protocol-1-mbps) |

For a checked, read-only example, see [joint groups](README.md#joint-groups).
Motion examples are listed in [README](README.md#examples).

---

## C++ / Python naming map

| C++ | Python | Notes |
|-----|--------|-------|
| `AttachCanPort` | `attach_can_port` | C++ driver: caller keeps port alive; Python driver: SDK keeps port alive |
| `Close` | `close` | Driver: stop use of the port without closing it; may wait for active RX callback |
| `HandleRxOnce` | `handle_rx_once` | Manual RX when not using port callback |
| `GetHighSpeedState` | `get_high_speed_feedback` | Cached read; non-blocking |
| `GetLowSpeedState` | `get_low_speed_feedback` | Cached read; non-blocking |
| `GetVersion` | `get_version` | Blocking |
| `SetEnable` | `set_enable` | |
| `SetTargetVelocity` | `set_target_velocity` | |
| `SetTargetPosition` | `set_target_position` | |
| `SetTargetCurrent` | `set_target_current` | |
| `SetReset` | `set_reset` | |
| `SetClearError` | `set_clear_error` | |
| `SetZeroOffset` | `set_zero_offset` | |
| `SetProfileAccDec` | `set_profile_acc_dec` | |
| `SetProfileVelocity` | `set_profile_vel` | |
| `SetCurrentLimit` | `set_current_limit` | |
| `SetMitControl` | `set_mit_control` | |
| `SetMitControlCrc` | `set_mit_control_crc` | |
| `SetCollisionThreshold` | `set_collision_threshold` | |
| `GetParam` / `SetParam` | `get_param` / `set_param` | `ArmDriver.MotorParam` |
| `CreateSocketCanPort` | `create_can_port` | Factory; see CAN layer below |

---

## Shared types

### C++ (`types.hpp`)

| Type | Fields |
|------|--------|
| `CanFrame` | `id`, `data`, `timestamp_ns` |
| `DriverStatusBits` | `undervoltage`, `motor_overtemp`, `driver_overcurrent`, `driver_overtemp`, `collision_tripped`, `driver_error`, `enabled`, `stall_tripped` |
| `HighSpeedState` | `position`, `velocity`, `current`, `timestamp_ns` — [rad], [rad/s] |
| `LowSpeedState` | `bus_voltage_v`, `driver_temp_deg`, `motor_temp_deg`, `bus_current`, `status_raw`, `status`, `timestamp_ns` |
| `VersionInfo` | `software`, `hardware`, `motor`, `timestamp_ns` |

### Python (`NamedTuple`)

| Type | Fields |
|------|--------|
| `DriverStatus` | Same booleans as `DriverStatusBits` |
| `HighSpeedFeedback` | `position`, `velocity`, `current`, `timestamp` [s] |
| `LowSpeedFeedback` | `bus_voltage_v`, `driver_temp_deg`, `motor_temp_deg`, `bus_current`, `status_raw`, `status`, `timestamp` [s] |
| `VersionInfo` | `software`, `hardware`, `motor`, `timestamp` [s] |

Cached feedback returns empty when no matching frame has been received yet.

---

## CAN transport

### Python

#### `create_can_port_config(**kwargs) -> dict`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `channel` | `"can0"` | Interface name |
| `interface` | `"socketcan"` | python-can bustype |
| `bitrate` | `1000000` | Nominal bitrate [bit/s] |
| `enable_check_can` | `True` | Verify interface exists (python-can) |
| `auto_connect` | `True` | Open on construction |
| `timeout` | `1.0` | Default `recv()` timeout [s] |
| `receive_own_messages` | `False` | python-can option |
| `local_loopback` | `False` | Socket loopback |
| `backend` | env or `"cpp"` | `"cpp"` or `"python-can"` |
| `fallback_python_can` | `True` | Fall back if C++ backend fails |

#### `create_can_port(config=None, **kwargs) -> CanPort`

Returns a port with:

| Method | Description |
|--------|-------------|
| `send(message)` | Send `python-can` `Message` |
| `recv()` | cpp: no-op sleep; python-can: blocking read, triggers callback |
| `set_callback(cb)` | `cb(Message)` on each RX frame |
| `clear_callback()` | Remove handler; an already running callback may still finish |
| `connect()` / `close()` | Open / close bus |
| `is_connected()` | Connection state |

Environment: `AGX_MOTOR_SDK_CAN_BACKEND=cpp|python-can`.
Use `create_can_port_config(...)` before `create_can_port(config)` to supply the
listed defaults, especially `auto_connect` for the `python-can` backend.

### C++

Include `agx_motor_sdk/can/can_port.hpp`. Create a port with
`agx::motor::CreateSocketCanPort("can0", false)` and check `Open()` before use.

| `CanPort` method | Description |
|------------------|-------------|
| `Open()` | `bool`: open the interface and start receiving |
| `Close()` | Stop receiving and close the socket; may wait for a callback to finish |
| `IsOpen()` | Whether port is active |
| `Send(frame)` | Transmit one frame |
| `SetReceiveHandler(handler)` | Install handler invoked on RX thread; removal/replacement waits for active callback; self-unregister is allowed |

For callback restrictions and external ports, see
[safe shutdown](docs/ADVANCED.md#safe-shutdown).

---

## `ArmDriver` (arm protocol, 1 Mbps)

Units: position [rad], velocity [rad/s], current [A], profile acc/dec [rad/s²].

### Construction

```python
driver = ArmDriver(can_port)          # Python
```

```cpp
auto driver = std::make_unique<agx::motor::ArmDriver>(can.get());  // C++
```

Pass an opened CAN port to send commands and receive feedback. With C++
`ArmDriver(CanPort*)`, keep the port alive until the driver is closed or destroyed.
The Python driver keeps its supplied port alive until replacement or destruction,
including after `close()`. Neither driver `close()` closes that port.
One `ArmDriver` per arm-protocol bus is typical.

### Motor parameters (`ArmDriver.MotorParam`)

| C++ | Python | Meaning |
|-----|--------|---------|
| `kId` | `ID` | Node ID (1–15) |
| `kAc` | `AC` | Profile acceleration [rad/s²], max 12.56 |
| `kDc` | `DC` | Profile deceleration [rad/s²], max 12.56 |
| `kVv` | `VV` | Profile velocity [rad/s], max 20 |
| `kIq` | `IQ` | Torque-loop current limit [A], max 12 |
| `kOi` | `OI` | Collision protection current [A], max 12 |
| `kOt` | `OT` | Collision protection time [s], max 2 |
| `kTx` | `TX` | Fast feedback: 0=push, 1=response |
| `kTf` | `TF` | MIT torque limit [N·m], default ±8 |
| `kSo` | `SO` | Zero offset [rad] |
| `kPp` | `PP` | Position Kp |
| `kKp` | `KP` | Velocity Kp |
| `kKi` | `KI` | Velocity Ki |

### Methods

| Method | Blocking | Description |
|--------|----------|-------------|
| `attach_can_port` / `AttachCanPort` | RX wait possible | Bind CAN TX/RX; replacing a port may wait for its active callback |
| `close` / `Close` | RX wait possible | Stop driver operation; no device ACK wait; port remains open |
| `handle_rx_once` / `HandleRxOnce` | no | Parse one frame manually |
| `get_high_speed_feedback` / `GetHighSpeedState` | no | Cached position, velocity, current |
| `get_low_speed_feedback` / `GetLowSpeedState` | no | Cached voltage, temps, status |
| `get_version` / `GetVersion` | yes | Firmware version strings |
| `set_enable` / `SetEnable` | no | Enable/disable; optional brake args |
| `set_target_velocity` / `SetTargetVelocity` | no | Velocity mode [rad/s] |
| `set_target_position` / `SetTargetPosition` | no | Position mode [rad] |
| `set_target_current` / `SetTargetCurrent` | no | Current mode [A] |
| `set_reset` / `SetReset` | optional | `mode`: 0=reset, 1=cal enter, 2=cal save |
| `set_clear_error` / `SetClearError` | optional | Clear faults |
| `set_zero_offset` / `SetZeroOffset` | optional | Zero offset [rad]; `save_to_flash` |
| `set_profile_acc_dec` / `SetProfileAccDec` | no | Profile acc/dec [rad/s²] |
| `set_profile_vel` / `SetProfileVelocity` | no | Profile velocity cap [rad/s] |
| `set_current_limit` / `SetCurrentLimit` | no | Phase current limit [A] |
| `set_mit_control` / `SetMitControl` | no | MIT control (no CRC); clamps in library |
| `set_mit_control_crc` / `SetMitControlCrc` | no | MIT control with CRC |
| `set_collision_threshold` / `SetCollisionThreshold` | no | Collision current [A] and time [s] |
| `get_param` / `GetParam` | yes | Read parameter |
| `set_param` / `SetParam` | yes | Write parameter |

**MIT limits** (library-side): `p_des` ∈ [-12.5, 12.5] rad; `v_des` ∈ [-45, 45] rad/s; `kp` ∈ [0, 500]; `kd` ∈ [-5, 5]; `t_ff` mapped by `t_am` (default 16 without CRC, 8 with CRC).

---

## Runtime environment

Defaults work for a standard installation. To select a CAN backend, supply a
protocol library path or configure offline installation, see
[environment variables](docs/ADVANCED.md#environment-variables).

---

## Examples

| Language | Path |
|----------|------|
| Python (driver) | `examples/arm/` |
| Python (mechanism) | `examples/mechanism/` |
| C++ | `sample/` → `build/bin/agx_motor_sdk_*_demo` |

See [README](README.md) for build and CAN setup.
