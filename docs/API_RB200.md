# RB200 protocol API

RB200-specific API on top of the common API in [API.md](../API.md). RB200 uses the
arm command frame set; its low-speed frame (`0x530 + node`) is replaced by the
CiA402 status word, error code and TPDO2 counter. Protocol library:
`libagx_motor_protocol_rb200.so` (see
[environment variables](ADVANCED.md#environment-variables)).

| Class | Python | C++ header |
|-------|--------|------------|
| Driver | `agx_motor_sdk.Rb200Driver` | `motor/rb200/rb200_driver.hpp` |
| Per-joint motor | `agx_motor_sdk.motor.Rb200Motor` | `mechanism/rb200/rb200_motor.hpp` |
| Low-speed feedback | `Rb200LowSpeedFeedback` | `Rb200LowSpeedState` (`motor/rb200/types.hpp`) |

## Construction

```python
from agx_motor_sdk import Rb200Driver
driver = Rb200Driver(can_port)
fb = driver.get_low_speed_feedback(node_id)   # Rb200LowSpeedFeedback or None
if fb is not None and fb.status.fault:
    print(hex(fb.error_code))
```

```cpp
auto driver = std::make_unique<agx::motor::Rb200Driver>(can.get());
auto fb = driver->GetLowSpeedState(node_id);  // std::optional<Rb200LowSpeedState>
```

An `ArmDriver` and an `Rb200Driver` may be attached to the same CAN port; each
parses only frames of its own protocol. In a `JointGroup`, set `protocol: rb200`
per module or per joint in [YAML](../YAML_CONFIG.md).

## Driver methods

All [MotorDriver](../API.md#motordriver-common-driver-api) methods, plus:

| Method | Blocking | Description |
|--------|----------|-------------|
| `get_low_speed_feedback` / `GetLowSpeedState` | no | Cached `Rb200LowSpeedFeedback` |

RB200 does not accept current mode (mode 2), so `set_target_current`, MIT,
collision threshold and `get_param` / `set_param` are not available.

## Per-joint motor (`Rb200Motor`)

`group.motor(i)` returns `Rb200Motor` for joints with `protocol: rb200`. It has all
[MotorBase](../API.md#per-joint-motor-view-motorbase) methods plus:

| Method | C++ | Description |
|--------|-----|-------------|
| `get_low_speed_feedback()` | `GetLowSpeedState` | Cached `Rb200LowSpeedFeedback` |

`JointGroup.set_mit` returns `False` for RB200 joints.

## Feedback types

### `Rb200LowSpeedFeedback` (Python) / `Rb200LowSpeedState` (C++)

| Field | Meaning |
|-------|---------|
| `status_word` | Raw CiA402 status word |
| `error_code` | Driver error code |
| `tpdo2_count` | TPDO2 frame counter |
| `status` | Decoded `Cia402Status` / `Cia402StatusBits` |
| `timestamp` [s] (Python) / `timestamp_ns` | Cache update time |

### `Cia402Status` (Python) / `Cia402StatusBits` (C++)

| Field | Bit |
|-------|-----|
| `ready_to_switch_on` | 0 |
| `switched_on` | 1 |
| `operation_enabled` | 2 |
| `fault` | 3 |
| `voltage_enabled` | 4 |
| `quick_stop` | 5 (`False` = quick stop active) |
| `switch_on_disabled` | 6 |
| `warning` | 7 |
| `remote` | 9 |
| `target_reached` | 10 |
| `internal_limit_active` | 11 |
| `homing_complete` | 15 |
