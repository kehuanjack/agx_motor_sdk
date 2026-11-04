"""Mechanism layer — multi-joint groups from YAML.

Public API: :class:`JointGroup` (one per YAML module, arm and RB200 joints).

See :mod:`agx_motor_sdk.mechanism.joint_group` for group API.
Per-joint motor API: :class:`~agx_motor_sdk.motor.MotorBase`,
:class:`~agx_motor_sdk.motor.ArmMotor`, :class:`~agx_motor_sdk.motor.Rb200Motor`.
"""

from agx_motor_sdk.mechanism.config import (
    GroupConfig,
    JointConfig,
    JointState,
    load_module_config,
)
from agx_motor_sdk.mechanism.joint_group import JointGroup
from agx_motor_sdk.motor import ArmMotor, MotorBase, Rb200Motor

__all__ = [
    "ArmMotor",
    "GroupConfig",
    "JointConfig",
    "JointGroup",
    "JointState",
    "MotorBase",
    "Rb200Motor",
    "load_module_config",
]
