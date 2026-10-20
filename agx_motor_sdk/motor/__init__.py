"""Motor protocol drivers (arm).

Public API
----------

Bus driver (one per CAN channel)
    :class:`~agx_motor_sdk.motor.arm_driver.ArmDriver`

Per-joint motor interface (from :meth:`~agx_motor_sdk.mechanism.ArmJointGroup.motor`)
    :class:`~agx_motor_sdk.motor.arm_motor_base.ArmMotorBase`

Feedback types (from C++ bindings; documented in :mod:`agx_motor_sdk._native`)
    :class:`~agx_motor_sdk._native.DriverStatus`
    :class:`~agx_motor_sdk._native.HighSpeedFeedback`
    :class:`~agx_motor_sdk._native.LowSpeedFeedback`
    :class:`~agx_motor_sdk._native.VersionInfo`

Example::

    from agx_motor_sdk.mechanism import ArmJointGroup

    group = ArmJointGroup.from_yaml("robot_motors.yaml", "head")
    group.connect()
    group.motor(0).set_enable()
"""

from agx_motor_sdk._native import (
    DriverStatus,
    HighSpeedFeedback,
    LowSpeedFeedback,
    VersionInfo,
)

from . import arm_driver as arm_driver
from . import arm_motor_base as arm_motor_base
from .arm_driver import ArmDriver
from .arm_motor_base import ArmMotorBase

__all__ = [
    "ArmDriver",
    "ArmMotorBase",
    "DriverStatus",
    "HighSpeedFeedback",
    "LowSpeedFeedback",
    "VersionInfo",
    "arm_driver",
    "arm_motor_base",
]
