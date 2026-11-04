# API Reference

Look up method names, units and return values here. For installation and a first
connection, start with [README](README.md). For joint settings, use
[YAML configuration](YAML_CONFIG.md).

| Area | Reference |
|------|-----------|
| Joint groups and per-joint control | [Mechanism layer](#mechanism-layer) |
| Commands shared by all protocols | [MotorDriver](#motordriver-common-driver-api) |
| Arm protocol: `ArmDriver`, `ArmMotor`, MIT, parameters | [API_ARM.md](docs/API_ARM.md) |
| RB200 protocol: `Rb200Driver`, `Rb200Motor`, CiA402 status | [API_RB200.md](docs/API_RB200.md) |
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

Each protocol has its own driver and per-joint motor class. Code shared by all
protocols lives in the base classes `MotorDriver` and `MotorBase`; protocol extras
live only in the subclass (for example MIT exists on `ArmMotor` but not on
`Rb200Motor`). `get_low_speed_feedback()` / `GetLowSpeedState()` has the same name
in every protocol class and returns that protocol's own feedback type.

| Protocol | Driver | Per-joint motor | Low-speed feedback | Reference |
|----------|--------|-----------------|--------------------|-----------|
| `arm` | `ArmDriver` | `ArmMotor` | `LowSpeedFeedback` / `LowSpeedState` | [API_ARM.md](docs/API_ARM.md) |
| `rb200` | `Rb200Driver` | `Rb200Motor` | `Rb200LowSpeedFeedback` / `Rb200LowSpeedState` | [API_RB200.md](docs/API_RB200.md) |

---

## Python package exports

```python
from agx_motor_sdk import (
    MotorDriver,             # common base
    ArmDriver,               # arm protocol
    Rb200Driver,             # RB200 protocol
    Cia402Status,
    DriverStatus,
    HighSpeedFeedback,
    LowSpeedFeedback,
    Rb200LowSpeedFeedback,
    VersionInfo,
    create_can_port,
    create_can_port_config,
    __version__,
)
```

C++ headers live under `include/agx_motor_sdk/`; namespace `agx::motor`
(drivers) and `agx::mechanism` (joint groups).

| Layer | Common | Arm | RB200 |
|-------|--------|-----|-------|
| Driver | `motor/motor_driver.hpp` | `motor/arm/arm_driver.hpp` | `motor/rb200/rb200_driver.hpp` |
| Per-joint motor | `mechanism/motor_base.hpp` | `mechanism/arm/arm_motor.hpp` | `mechanism/rb200/rb200_motor.hpp` |
| Python | `motor/motor_driver.py`, `motor/motor_base.py` | `motor/arm_driver.py`, `motor/arm_motor.py` | `motor/rb200_driver.py`, `motor/rb200_motor.py` |

---

## Mechanism layer

YAML-driven joint groups for multi-joint modules (`head`, `fold`, …). One module = one `JointGroup`; joints may use different protocols. Schema: [YAML_CONFIG.md](YAML_CONFIG.md). Examples: `examples/mechanism/`.

### Python imports

```python
from agx_motor_sdk.mechanism import (
    JointGroup,
    GroupConfig,
    JointConfig,
    JointState,
    load_module_config,
    joint_names_from_config,
)
from agx_motor_sdk.motor import MotorBase, ArmMotor, Rb200Motor
```

### Construction

| Python | C++ |
|--------|-----|
| `JointGroup(config)` | `JointGroup(config)` |
| `JointGroup(config, can_channel)` | `JointGroup(config, can_channel)` |
| `JointGroup.from_yaml(path, key)` | `JointGroup::FromYaml(path, key)` |
| `JointGroup.from_yaml(path, key, can)` | `JointGroup::FromYaml(path, key, can)` |
| — | `CreateJointGroup(config[, can])` → `unique_ptr<JointGroup>` |
| — | `CreateJointGroupFromYaml(path, key[, can])` |

YAML loading and direct group construction both validate configuration. See
[validation rules](YAML_CONFIG.md#validation-rules) and
[channel overrides](YAML_CONFIG.md#multiple-can-interfaces).

### Joint index semantics

| Symbol | C++ | Python | Value |
|--------|-----|--------|-------|
| Invalid index | `kInvalidJointIndex` (`types.hpp`) | `INVALID_JOINT_INDEX` on `JointGroup` | `-1` |

- `index_of(name)` → index or `INVALID_JOINT_INDEX`.
- `is_valid_joint_index(index)` → `index ∈ [0, joint_count)`.
- `motor(index)` never raises: out-of-range returns a no-op `MotorBase` with `is_valid=False`.
- A valid index returns the joint's protocol class: `ArmMotor` or `Rb200Motor`.
  Check with `isinstance` before calling protocol-only methods.

### Group methods (`JointGroup`)

| Method | C++ | Description |
|--------|-----|-------------|
| `attach_can_port(can[, channel])` | `AttachCanPort` | Supply external `CanPort` before `connect`; group keeps the port alive |
| `connect()` | `Connect` | `bool`: whether all required CAN connections opened |
| `close()` | `Close` | Disconnect this group without closing externally supplied ports |
| `is_connected()` | `IsConnected` | `bool`: whether all required connections are available |
| `enable([interval])` / `disable([interval])` | `Enable` / `Disable` | All joints; optional delay `interval` [s] between joint commands (default `0`, must be ≥ 0) |
| `configure()` | `Configure` | Apply YAML profile / MIT to all joints |
| `joint_count` | `JointCount` | Number of joints |
| `joint_names` | `JointNames` | Names in YAML order |
| `protocol` | `protocol()` | Module default protocol: `"arm"` or `"rb200"` (joints may override) |
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
| `motor(index)` | `Motor` | `ArmMotor` / `Rb200Motor` by joint protocol; invalid index → no-op `MotorBase` |
| `set_mit(i, pos, vel, torque)` | `SetMit` | Joint-space MIT (arm joints only; `False` for RB200); needs YAML `mit_tf` / `mit_kp` / `mit_kd` |

The group keeps supplied ports alive until replacement or group destruction, even
after `close()`. Close external ports yourself after all users have stopped using
them. Stop motor commands before closing the group.
See [safe shutdown](docs/ADVANCED.md#safe-shutdown).

### Per-joint motor view (`MotorBase`)

Wraps one joint with joint-space helpers (limits and polarity). These methods exist
on every protocol; protocol extras are in [ArmMotor](docs/API_ARM.md#per-joint-motor-armmotor)
and [Rb200Motor](docs/API_RB200.md#per-joint-motor-rb200motor).

| Method | C++ | Description |
|--------|-----|-------------|
| `is_valid` | `IsValid` | `True` for a real joint index |
| `protocol`, `node_id`, `can_channel`, `name`, `config` | `protocol`, `NodeId`, `CanChannel`, `Name`, `Config` | Metadata |
| `set_enable(enable=True, ...)` | `SetEnable` | Enable or disable motor |
| `set_target(value, mode)` | `SetTarget` | Generic target; mode 0=velocity, 1=position |
| `set_target_position(pos)` | `SetTargetPosition` | Joint-space position [rad] (limits + polarity) |
| `set_target_velocity(vel)` | `SetTargetVelocity` | Joint-space velocity [rad/s] (polarity) |
| `get_state()` | `GetState` | `JointState` (position, velocity, torque with polarity) |
| `configure()` | `Configure` | Apply this joint's YAML (arm also syncs `mit_tf`) |
| `get_high_speed_feedback()` | `GetHighSpeedState` | Cached joint-space position, velocity, current |
| `get_version()` | `GetVersion` | Firmware version (blocking) |
| `set_reset`, `set_clear_error`, `set_zero_offset` | `SetReset`, `SetClearError`, `SetZeroOffset` | See [MotorDriver](#motordriver-common-driver-api) |
| `set_profile_acc_dec`, `set_profile_vel`, `set_current_limit` | `SetProfileAccDec`, `SetProfileVelocity`, `SetCurrentLimit` | Profile and limits |

```python
from agx_motor_sdk.motor import ArmMotor, Rb200Motor

m = group.motor(group.index_of("lift_j1"))
if isinstance(m, Rb200Motor):
    fb = m.get_low_speed_feedback()      # Rb200LowSpeedFeedback or None
elif isinstance(m, ArmMotor):
    fb = m.get_low_speed_feedback()      # LowSpeedFeedback or None
```

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
| `GetLowSpeedState` | `get_low_speed_feedback` | Cached read; returns the protocol's own type |
| `GetVersion` | `get_version` | Blocking |
| `SetEnable` | `set_enable` | |
| `SetTargetVelocity` | `set_target_velocity` | |
| `SetTargetPosition` | `set_target_position` | |
| `SetTargetCurrent` | `set_target_current` | Arm only |
| `SetReset` | `set_reset` | |
| `SetClearError` | `set_clear_error` | |
| `SetZeroOffset` | `set_zero_offset` | |
| `SetProfileAccDec` | `set_profile_acc_dec` | |
| `SetProfileVelocity` | `set_profile_vel` | |
| `SetCurrentLimit` | `set_current_limit` | |
| `SetMitControl` | `set_mit_control` | Arm only |
| `SetMitControlCrc` | `set_mit_control_crc` | Arm only |
| `SetCollisionThreshold` | `set_collision_threshold` | Arm only |
| `GetParam` / `SetParam` | `get_param` / `set_param` | Arm only; `ArmDriver.MotorParam` |
| `CreateSocketCanPort` | `create_can_port` | Factory; see CAN layer below |

---

## Shared types

### C++ (`types.hpp`)

| Type | Fields |
|------|--------|
| `CanFrame` | `id`, `data`, `timestamp_ns` |
| `DriverStatusBits` | `undervoltage`, `motor_overtemp`, `driver_overcurrent`, `driver_overtemp`, `collision_tripped`, `driver_error`, `enabled`, `stall_tripped` |
| `HighSpeedState` | `position`, `velocity`, `current`, `timestamp_ns` — [rad], [rad/s] |
| `LowSpeedState` (arm) | `bus_voltage_v`, `driver_temp_deg`, `motor_temp_deg`, `bus_current`, `status_raw`, `status`, `timestamp_ns` |
| `VersionInfo` | `software`, `hardware`, `motor`, `timestamp_ns` |

### Python (`NamedTuple`)

| Type | Fields |
|------|--------|
| `DriverStatus` | Same booleans as `DriverStatusBits` |
| `HighSpeedFeedback` | `position`, `velocity`, `current`, `timestamp` [s] |
| `LowSpeedFeedback` (arm) | `bus_voltage_v`, `driver_temp_deg`, `motor_temp_deg`, `bus_current`, `status_raw`, `status`, `timestamp` [s] |
| `VersionInfo` | `software`, `hardware`, `motor`, `timestamp` [s] |

RB200 types (`Rb200LowSpeedState`, `Cia402StatusBits` / Python `Rb200LowSpeedFeedback`,
`Cia402Status`) are listed in [API_RB200.md](docs/API_RB200.md#feedback-types).

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

## `MotorDriver` (common driver API)

`MotorDriver` is the base class of `ArmDriver` and `Rb200Driver`. Construct a
protocol driver, not `MotorDriver` itself. Units: position [rad], velocity [rad/s],
current [A], profile acc/dec [rad/s²].

Pass an opened CAN port to send commands and receive feedback. With C++
`ArmDriver(CanPort*)` / `Rb200Driver(CanPort*)`, keep the port alive until the driver
is closed or destroyed. The Python driver keeps its supplied port alive until
replacement or destruction, including after `close()`. Neither driver `close()`
closes that port. Use one driver per protocol per bus; arm and RB200 drivers may
share one port.

| Method | Blocking | Description |
|--------|----------|-------------|
| `protocol` / `protocol_name()` | no | `"arm"` or `"rb200"` |
| `attach_can_port` / `AttachCanPort` | RX wait possible | Bind CAN TX/RX; replacing a port may wait for its active callback |
| `close` / `Close` | RX wait possible | Stop driver operation; no device ACK wait; port remains open |
| `handle_rx_once` / `HandleRxOnce` | no | Parse one frame manually |
| `get_high_speed_feedback` / `GetHighSpeedState` | no | Cached position, velocity, current |
| `get_low_speed_feedback` / `GetLowSpeedState` | no | Defined per protocol; see [arm](docs/API_ARM.md) / [RB200](docs/API_RB200.md) |
| `get_version` / `GetVersion` | yes | Firmware version strings |
| `set_enable` / `SetEnable` | no | Enable/disable; optional brake args |
| `set_target` / `SetTarget` | no | Generic target with explicit mode byte |
| `set_target_velocity` / `SetTargetVelocity` | no | Velocity mode [rad/s] |
| `set_target_position` / `SetTargetPosition` | no | Position mode [rad] |
| `set_reset` / `SetReset` | optional | `mode`: 0=reset, 1=cal enter, 2=cal save |
| `set_clear_error` / `SetClearError` | optional | Clear faults |
| `set_zero_offset` / `SetZeroOffset` | optional | Zero offset [rad]; `save_to_flash` |
| `set_profile_acc_dec` / `SetProfileAccDec` | no | Profile acc/dec [rad/s²] |
| `set_profile_vel` / `SetProfileVelocity` | no | Profile velocity cap [rad/s] |
| `set_current_limit` / `SetCurrentLimit` | no | Phase current limit [A] |

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
