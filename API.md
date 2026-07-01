# API Reference

Public interfaces for **agx_motor_sdk** v1.1.0. Names `arm` / `chassis` denote **motor firmware protocols**, not robot subsystems.

| Protocol | CAN bitrate | C++ driver | Python driver |
|----------|-------------|------------|---------------|
| arm | 1 Mbps | `agx::motor::ArmDriver` | `ArmDriver` |
| chassis | 500 kbps | `agx::motor::ChassisDriver` | `ChassisDriver` |

**Node ID**: `1`–`15` on all control and feedback APIs.

**RX model**: Drivers parse incoming frames via a CAN receive callback. With the default **cpp** backend, RX runs in a C++ background thread; with **python-can**, you must poll `recv()` in a loop. See [README — Python quick start](README.md#python-quick-start).

---

## Python package exports

```python
from agx_motor_sdk import (
    ArmDriver,
    ChassisDriver,
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

## C++ / Python naming map

| C++ | Python | Notes |
|-----|--------|-------|
| `AttachCanPort` | `attach_can_port` | Auto-called when `can_port` is passed to constructor |
| `Close` | `close` | |
| `HandleRxOnce` | `handle_rx_once` | Manual RX when not using port callback |
| `GetHighSpeedState` | `get_high_speed_feedback` | Cached read; non-blocking |
| `GetLowSpeedState` | `get_low_speed_feedback` | Cached read; non-blocking |
| `GetVersion` | `get_version` | Arm only; blocking |
| `SetEnable` | `set_enable` | |
| `SetTargetVelocity` | `set_target_velocity` | Arm only |
| `SetTargetPosition` | `set_target_position` | Arm only |
| `SetTargetRpm` | `set_target_rpm` | Chassis only |
| `SetTargetInc` | `set_target_inc` | Chassis only |
| `SetTargetCurrent` | `set_target_current` | |
| `SetReset` | `set_reset` | |
| `SetClearError` | `set_clear_error` | |
| `SetZeroOffset` | `set_zero_offset` | |
| `SetProfileAccDec` | `set_profile_acc_dec` | |
| `SetProfileVelocity` | `set_profile_vel` | |
| `SetCurrentLimit` | `set_current_limit` | |
| `SetMitControl` | `set_mit_control` | Arm only |
| `SetMitControlCrc` | `set_mit_control_crc` | Arm only |
| `SetCollisionThreshold` | `set_collision_threshold` | Arm only |
| `GetParam` / `SetParam` | `get_param` / `set_param` | Arm only; `ArmMotorParam` / `MotorParam` |
| `SetStiffness` | `set_stiffness` | Chassis only |
| `CreateSocketCanPort` | `create_can_port` | Factory; see CAN layer below |

---

## Shared types

### C++ (`types.hpp`)

| Type | Fields |
|------|--------|
| `CanFrame` | `id`, `data`, `timestamp_ns` |
| `DriverStatusBits` | `undervoltage`, `motor_overtemp`, `driver_overcurrent`, `driver_overtemp`, `collision_tripped`, `driver_error`, `enabled`, `stall_tripped` |
| `HighSpeedState` | `position`, `velocity`, `current`, `timestamp_ns` — arm: rad / rad/s; chassis: INC / RPM |
| `LowSpeedState` | `bus_voltage_v`, `driver_temp_deg`, `motor_temp_deg`, `bus_current`, `status_raw`, `status`, `timestamp_ns` |
| `VersionInfo` | `software`, `hardware`, `motor`, `timestamp_ns` — arm only |

### Python (`NamedTuple`)

| Type | Fields |
|------|--------|
| `DriverStatus` | Same booleans as `DriverStatusBits` |
| `HighSpeedFeedback` | `position`, `velocity`, `current`, `timestamp` [s] |
| `LowSpeedFeedback` | `bus_voltage_v`, `driver_temp_deg`, `motor_temp_deg`, `bus_current`, `status_raw`, `status`, `timestamp` [s] |
| `VersionInfo` | `software`, `hardware`, `motor`, `timestamp` [s] |

**Status semantics (chassis)**: `collision_tripped` = sensor fault; `stall_tripped` = homed; `enabled` = status bit 6.

Blocking APIs return `std::nullopt` / `None` on timeout or send failure. Cached feedback returns empty when no frame has been received yet.

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
| `clear_callback()` | Remove handler |
| `connect()` / `close()` | Open / close bus |
| `is_connected()` | Connection state (python-can) |

Environment: `AGX_MOTOR_SDK_CAN_BACKEND=cpp|python-can`.

### C++

```cpp
#include "agx_motor_sdk/can/can_port.hpp"

auto can = agx::motor::CreateSocketCanPort("can0", /*local_loopback=*/false);
can->Open();
can->SetReceiveHandler([](const agx::motor::CanFrame& frame) { /* ... */ });
can->Send(frame);
can->Close();
```

| `CanPort` method | Description |
|------------------|-------------|
| `Open()` | Open interface; start RX thread (`SocketCanPort`) |
| `Close()` | Stop RX and close socket |
| `IsOpen()` | Whether port is active |
| `Send(frame)` | Transmit one frame |
| `SetReceiveHandler(handler)` | `nullptr` clears; called from RX thread |

---

## `ArmDriver` (arm protocol, 1 Mbps)

Units: position [rad], velocity [rad/s], current [A], profile acc/dec [rad/s²].

### Construction

```python
driver = ArmDriver(can_port)          # Python
```

```cpp
auto driver = std::make_unique<agx::motor::ArmDriver>(can);  // C++
```

Pass an opened CAN port to auto-bind TX/RX. One `ArmDriver` per arm-protocol bus is typical.

### Motor parameters (`ArmMotorParam` / `ArmDriver.MotorParam`)

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
| `attach_can_port` / `AttachCanPort` | no | Bind CAN TX/RX |
| `close` / `Close` | no | Detach and destroy native handle |
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

## `ChassisDriver` (chassis protocol, 500 kbps)

Units: position [INC], velocity [RPM], current [A], profile acc/dec [RPM/s].

No `get_version`, `get_param`, `set_param`, MIT, or collision-threshold APIs.

### Methods

| Method | Blocking | Description |
|--------|----------|-------------|
| `attach_can_port` / `AttachCanPort` | no | Bind CAN TX/RX |
| `close` / `Close` | no | Detach and destroy native handle |
| `handle_rx_once` / `HandleRxOnce` | no | Parse one frame manually |
| `get_high_speed_feedback` / `GetHighSpeedState` | no | Cached INC, RPM, current |
| `get_low_speed_feedback` / `GetLowSpeedState` | no | Cached voltage, temps, status |
| `set_enable` / `SetEnable` | no | Enable/disable (0x420) |
| `set_target_rpm` / `SetTargetRpm` | no | RPM mode (0x410 mode 0) |
| `set_target_inc` / `SetTargetInc` | no | Position mode [INC] (mode 1) |
| `set_target_current` / `SetTargetCurrent` | no | Current mode [A] (mode 2) |
| `set_reset` / `SetReset` | optional | Reset / calibration (0x000) |
| `set_clear_error` / `SetClearError` | optional | Clear faults (0x010) |
| `set_zero_offset` / `SetZeroOffset` | optional | Encoder zero [INC] (0x020) |
| `set_profile_acc_dec` / `SetProfileAccDec` | no | Profile acc/dec [RPM/s] (0x430) |
| `set_profile_vel` / `SetProfileVelocity` | no | Profile RPM cap (0x440) |
| `set_current_limit` / `SetCurrentLimit` | no | Current limit (0x450) |
| `set_stiffness` / `SetStiffness` | no | Stiffness 10–2000 (0x470) |

---

## Multi-protocol bus

Do **not** attach both `ArmDriver` and `ChassisDriver` to the same `CanPort` callback chain. Use separate CAN interfaces, or a single RX dispatcher that calls `handle_rx_once` on the correct driver by `node_id` / protocol rules.

---

## Runtime environment

| Variable | Purpose |
|----------|---------|
| `AGX_MOTOR_SDK_ARM_LIB` | Path to `libagx_motor_protocol_arm` |
| `AGX_MOTOR_SDK_CHASSIS_LIB` | Path to `libagx_motor_protocol_chassis` |
| `AGX_MOTOR_SDK_CAN_BACKEND` | `cpp` or `python-can` |
| `AGX_MOTOR_SDK_SKIP_DOWNLOAD` | Skip protocol download at build/install |
| `AGX_MOTOR_SDK_PROTOCOL_VERSION` | Protocol Release tag (default: `version.__protocol_version__`) |
| `AGX_MOTOR_SDK_GITHUB_REPO` | Source repo for protocol Release assets |

---

## Examples

| Language | Path |
|----------|------|
| Python | `examples/arm/`, `examples/chassis/` |
| C++ | `sample/` → `build/bin/agx_motor_sdk_*_demo` |

See [README](README.md) for build and CAN setup.
