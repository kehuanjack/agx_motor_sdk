"""Mechanism layer — multi-joint groups from YAML.

Public API: :class:`ArmJointGroup` (one per YAML module).

See :mod:`agx_motor_sdk.mechanism.joint_group` for group API.
Per-joint motor API: :mod:`agx_motor_sdk.motor.arm_motor_base`.
"""

from agx_motor_sdk.mechanism.config import (
    GroupConfig,
    JointConfig,
    JointState,
    load_module_config,
)
from agx_motor_sdk.mechanism.joint_group import ArmJointGroup
from agx_motor_sdk.motor.arm_motor_base import ArmMotorBase

__all__ = [
    "ArmJointGroup",
    "ArmMotorBase",
    "GroupConfig",
    "JointConfig",
    "JointState",
    "load_module_config",
]
