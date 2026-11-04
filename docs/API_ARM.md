# Arm protocol API

Arm-protocol extras on top of the common API in [API.md](../API.md). CAN bitrate
1 Mbps. Protocol library: `libagx_motor_protocol_arm.so`.

| Class | Python | C++ header |
|-------|--------|------------|
| Driver | `agx_motor_sdk.ArmDriver` | `motor/arm/arm_driver.hpp` |
| Per-joint motor | `agx_motor_sdk.motor.ArmMotor` | `mechanism/arm/arm_motor.hpp` |
| Low-speed feedback | `LowSpeedFeedback` | `LowSpeedState` (`types.hpp`) |

## Construction

```python
from agx_motor_sdk import ArmDriver
driver = ArmDriver(can_port)
```

```cpp
auto driver = std::make_unique<agx::motor::ArmDriver>(can.get());
```

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

### Arm-only driver methods

In addition to [MotorDriver](../API.md#motordriver-common-driver-api):

| Method | Blocking | Description |
|--------|----------|-------------|
| `get_low_speed_feedback` / `GetLowSpeedState` | no | Cached `LowSpeedFeedback`: voltage, temps, status |
| `set_target_current` / `SetTargetCurrent` | no | Current mode [A] |
| `set_mit_control` / `SetMitControl` | no | MIT control (no CRC); clamps in library |
| `set_mit_control_crc` / `SetMitControlCrc` | no | MIT control with CRC |
| `set_collision_threshold` / `SetCollisionThreshold` | no | Collision current [A] and time [s] |
| `get_param` / `GetParam` | yes | Read parameter |
| `set_param` / `SetParam` | yes | Write parameter |

**MIT limits** (library-side): `p_des` ∈ [-12.5, 12.5] rad; `v_des` ∈ [-45, 45] rad/s; `kp` ∈ [0, 500]; `kd` ∈ [-5, 5]; `t_ff` mapped by `t_am` (default 16 without CRC, 8 with CRC).

## Per-joint motor (`ArmMotor`)

`group.motor(i)` returns `ArmMotor` for joints with `protocol: arm` (the default).
It has all [MotorBase](../API.md#per-joint-motor-view-motorbase) methods plus:

| Method | C++ | Description |
|--------|-----|-------------|
| `set_mit(pos, vel, torque)` | `SetMit` | Joint-space MIT using YAML `mit_kp` / `mit_kd` / `mit_tf` |
| `get_low_speed_feedback()` | `GetLowSpeedState` | Cached `LowSpeedFeedback` |
| `set_target_current(a)` | `SetTargetCurrent` | Current mode [A] |
| `set_mit_control(...)` / `set_mit_control_crc(...)` | `SetMitControl` / `SetMitControlCrc` | Low-level MIT in joint space |
| `set_collision_threshold(a, s)` | `SetCollisionThreshold` | Collision current [A] and time [s] |
| `get_param(p)` / `set_param(p, v)` | `GetParam` / `SetParam` | `ArmDriver.MotorParam` |

`configure()` on `ArmMotor` also writes YAML `mit_tf` to motor parameter `TF`.

