# YAML configuration

Use one top-level module key for each joint group. Set the CAN interface, actual
motor node IDs and joint names first; add motion settings only when needed.
Return to [joint-group usage](README.md#joint-groups) for loading and connecting.

## Minimal example

```yaml
head:
  can_channel: can0
  joints:
    - joint_name: head_pitch_joint
      node_id: 6
      polarity: 1.0
      lower_limit: -0.5
      upper_limit: 0.5
```

Load with `ArmJointGroup.from_yaml("robot_motors.yaml", "head")`. Replace the
interface, node ID and limits to match your hardware. Loading YAML alone does not
connect, enable a motor or apply motion settings.

Unknown keys are ignored, so check spelling carefully. Use `lower_limit` and
`upper_limit`; a `limits: [min, max]` field is not supported.

## Module-level keys (`GroupConfig`)

| YAML key | Python attribute | Required | Default | Meaning |
|----------|------------------|----------|---------|---------|
| `can_channel` | `can_channel` | **Yes** | — | Default CAN interface name (e.g. `can0`). Joints may override. |
| `profile_vel` | `default_profile_vel_rad_s` | No | — | Default motion profile speed [rad/s] for joints without their own `profile_vel`. |
| `profile_accel` | `profile_accel_rad_s2` | No | — | Default profile acceleration [rad/s²]. |
| `profile_decel` | `profile_decel_rad_s2` | No | — | Default profile deceleration [rad/s²]. |
| `mit_tf` | `mit_tf` | No | — | MIT torque mapping half-range [N·m] (`MotorParam.TF`). Used by `configure()` and joint-space `set_mit`. |
| `mit_kp` | `mit_kp` | No | — | Default MIT position gain for `set_mit`. |
| `mit_kd` | `mit_kd` | No | — | Default MIT damping gain for `set_mit`. |
| `torque_k` | `torque_k` | No | — | Scale factor in estimated torque: `torque_nm = current × torque_k × torque_b`. |
| `torque_b` | `torque_b` | No | — | Second factor in torque estimate and MIT feedforward mapping. |
| `joints` | `joints` | **Yes** | — | List of joint entries (see below). |

## Joint-level keys (`JointConfig`)

| YAML key | Required | Inherits module default | Meaning |
|----------|----------|-------------------------|---------|
| `joint_name` | **Yes** | — | Logical name (URDF / logging). |
| `node_id` | **Yes** | — | CAN node ID (1–15). |
| `polarity` | No | `1.0` | Per-joint linear coefficient: command `motor = joint / polarity`, feedback `joint = motor × polarity` (position, velocity, torque). Default `1.0`. `0` = pass-through. |
| `can_channel` | No | `can_channel` | Per-joint bus override for multi-channel modules. |
| `profile_vel` | No | ✓ | Override profile speed [rad/s] for this joint. |
| `profile_accel` | No | ✓ | Override profile acceleration [rad/s²]. |
| `profile_decel` | No | ✓ | Override profile deceleration [rad/s²]. |
| `mit_tf` | No | ✓ | Per-joint MIT torque limit. |
| `mit_kp` | No | ✓ | Per-joint MIT Kp (required for `set_mit`). |
| `mit_kd` | No | ✓ | Per-joint MIT Kd (required for `set_mit`). |
| `torque_k` | No | ✓ (else 1.0) | Per-joint torque estimate scale. |
| `torque_b` | No | ✓ (else 1.0) | Per-joint torque estimate / MIT feedforward scale. |
| `lower_limit` | No | — | Soft lower limit in joint space [rad]. |
| `upper_limit` | No | — | Soft upper limit in joint space [rad]. |

Use `lower_limit` / `upper_limit` separately. A `limits: [min, max]` array is
**not** parsed.

## Units and polarity

Positions and limits are in radians; velocities are in radians per second.
`ArmDriver` uses motor-space values. Group and per-joint position commands apply
joint limits and polarity before sending a motor-space value.

| Polarity | Effect |
|----------|--------|
| `1.0` (default) | Joint and motor directions match |
| `-1.0` | Reverse the joint direction |
| Other finite nonzero value | Command: motor = joint / polarity; feedback: joint = motor * polarity |
| `0` | Pass-through, not a disabled joint |

Limits constrain joint-space position commands; they are not hardware safety
stops. Choose them for your mechanism and keep independent safety measures.

## Validation rules

YAML loading and direct `ArmJointGroup` construction enforce the same rules:

- The module default `can_channel` must be non-empty and `joints` must be non-empty.
- Each joint needs a non-empty name, unique within the group, and a node ID in `1..15`.
- An empty joint channel resolves to the module default. Each resolved
  `(can_channel, node_id)` must be unique within the group. The same node ID on
  different channels is allowed.
- Supplied module and joint numeric configuration values must be finite
  (no NaN or infinity), including polarity, torque factors and enabled limits.
- If both limits are enabled, `lower_limit <= upper_limit`. A single limit is
  allowed. Finite `polarity: 0` retains its pass-through behavior.

These are software configuration checks, not validation of device replies or
firmware parameter ranges. Invalid configurations raise an exception before CAN
connection. Check the exception message for the invalid field or conflicting joint.

## Multiple CAN interfaces

Set a joint's `can_channel` only when it uses a different bus. Omitted or empty
joint channels use the module default. The same node ID may be used on different
interfaces, but not twice on the same interface within one group.

Passing a constructor channel override replaces the module default and all joint
channels that are empty or equal to the previous default. This also replaces an
explicit joint channel with the same value as that previous default. Other joint
channels remain unchanged. The resulting configuration is validated again:
an empty override or a channel merge that creates duplicate nodes is rejected.

## Applying motion settings

After connecting, call `group.configure()` to apply the supplied motion settings:

- A positive `profile_vel` sets the profile speed.
- Positive `profile_accel` and `profile_decel` set acceleration and deceleration;
  supply both to apply them.
- `mit_tf` configures the MIT torque mapping range.

Module motion settings are defaults for joints that do not override them.
`mit_kp` and `mit_kd` are used by `set_mit`; they are not applied as persistent
parameters by `configure()`. Check method results and device feedback; loading a
valid configuration does not prove that the motor accepted its settings.

## Joint-space commands

Per-joint motor methods apply polarity and limits:

- `motor.set_target_position(x)` — clamp → polarity → driver
- `motor.set_target_velocity(v)` — polarity → driver
- `motor.get_high_speed_feedback()` — driver feedback → polarity (position, velocity, current)
- `motor.set_mit_control` / `set_mit_control_crc` — limit (position) + polarity → driver
- `motor.get_state()` — reads `get_high_speed_feedback()` and estimates torque
- `motor.set_mit(p, v, t)` — delegates to `set_mit_control` (requires YAML `mit_*`)

`ArmDriver` low-level APIs use motor-space values; the per-joint `ArmMotorBase`
API applies polarity.

## Topology

- Groups using the same `can_channel` communicate on the same CAN bus.
- Within one group, duplicate resolved `(can_channel, node_id)` entries are rejected.
  Across groups, the same pair addresses the same motor. Avoid conflicting
  configurations or commands for that motor; prefer one group per physical joint.
- One module may use multiple channels via per-joint `can_channel`.

Stop motor commands before closing a group. If you supplied an external CAN port,
close it yourself after all users have stopped using it.
See [safe shutdown](docs/ADVANCED.md#safe-shutdown).

## Example with motion settings

```yaml
head:
  can_channel: can0
  profile_vel: 5.0
  mit_tf: 16.0
  mit_kp: 10.0
  mit_kd: 0.8
  joints:
    - joint_name: head_pitch_joint
      node_id: 6
      polarity: -1.0
      lower_limit: -0.523
      upper_limit: 1.396
```

See also `examples/mechanism/robot_motors.yaml`.

## Looking up a joint

- `index_of(name)` returns the joint index, or `INVALID_JOINT_INDEX` (`-1`) when the
  name is not in the group. It does not raise.
- `is_valid_joint_index(index)` checks whether `index` is in `[0, joint_count)`.
- `motor(index)` never raises for out-of-range or invalid indices. It returns a
  no-op motor (`is_valid=False`) that implements `ArmMotorBase`. All calls return
  `False` / `None` and do not touch the CAN bus.
- Valid indices still require `connect()` (or an attached CAN port) before bus
  operations succeed.

Use the checked, read-only example in [README](README.md#joint-groups) before
trying motion. Method names and return types are listed in [API reference](API.md).
