"""YAML configuration types for the mechanism layer.

Full field tables: :doc:`YAML_CONFIG.md` in the package root.

Typical workflow::

    from agx_motor_sdk.mechanism import ArmJointGroup, load_module_config

    cfg = load_module_config("robot_motors.yaml", "head")
    group = ArmJointGroup(cfg)
"""

from typing import List

from agx_motor_sdk._native import GroupConfig as _GroupConfig
from agx_motor_sdk._native import JointConfig as _JointConfig
from agx_motor_sdk._native import JointState as _JointState
from agx_motor_sdk._native import load_module_config as _load_module_config

GroupConfig = _GroupConfig
"""One YAML module block (e.g. ``head``, ``fold``).

Attributes:
    can_channel: Default CAN interface name; required in YAML.
    default_profile_vel_rad_s: Module default profile speed [rad/s] max 20.
    profile_accel_rad_s2: Default acceleration [rad/s²] max 12.56.
    profile_decel_rad_s2: Default deceleration [rad/s²] max 12.56.
    mit_tf: MIT torque half-range [N·m]; default motor param ±8.
    mit_kp: Default MIT position gain for ``set_mit``; MIT clamp [0, 500].
    mit_kd: Default MIT damping for ``set_mit``; MIT clamp [-5, 5].
    torque_k: Torque estimate scale (``torque_nm = current × torque_k × torque_b``).
    torque_b: Second torque scale; default 1.0 when unset at joint level.
    joints: Non-empty list of :class:`JointConfig` entries.
"""

JointConfig = _JointConfig
"""Per-joint YAML entry (``joints`` list item).

Attributes:
    name: ``joint_name`` from YAML (required).
    node_id: CAN node ID, 1–15 (required).
    can_channel: Resolved channel (joint override or module default).
    polarity: Per-joint linear coefficient (default ``1.0``).
    profile_vel_rad_s: Override profile speed [rad/s] max 20.
    profile_accel_rad_s2: Override acceleration max 12.56 rad/s².
    profile_decel_rad_s2: Override deceleration max 12.56 rad/s².
    mit_tf: Per-joint MIT torque half-range [N·m].
    mit_kp: Per-joint MIT Kp; required for ``set_mit``; clamp [0, 500].
    mit_kd: Per-joint MIT Kd; required for ``set_mit``; clamp [-5, 5].
    torque_k: Per-joint torque estimate scale; default 1.0 when unset.
    torque_b: Per-joint torque / MIT feedforward scale; default 1.0 when unset.
    lower_limit_rad: Soft lower limit in joint space [rad].
    upper_limit_rad: Soft upper limit in joint space [rad].
    has_lower_limit: Whether ``lower_limit_rad`` is applied on ``set_target_position``.
    has_upper_limit: Whether ``upper_limit_rad`` is applied on ``set_target_position``.
"""

JointState = _JointState
"""Joint-space feedback from ``motor.get_state()``.

Attributes:
    position_rad: Joint position [rad].
    velocity_rad_s: Joint velocity [rad/s].
    torque_nm: Estimated torque [N·m] from phase current and ``torque_k`` × ``torque_b``.
"""


def load_module_config(yaml_path: str, module_key: str) -> GroupConfig:
    """Load one module block from a robot motors YAML file.

    Args:
        yaml_path: Path to YAML (e.g. ``robot_motors.yaml``).
        module_key: Top-level key, e.g. ``"head"``.

    Returns:
        Parsed :class:`GroupConfig` with resolved per-joint ``can_channel`` values.

    Raises:
        RuntimeError: Missing file/key or empty ``joints``.
    """
    return _load_module_config(yaml_path, module_key)


def joint_names_from_config(config: GroupConfig) -> List[str]:
    """Return joint names in YAML list order."""
    return [joint.name for joint in config.joints]


__all__ = [
    "GroupConfig",
    "JointConfig",
    "JointState",
    "joint_names_from_config",
    "load_module_config",
]
