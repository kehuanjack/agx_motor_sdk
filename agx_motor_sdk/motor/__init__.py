"""Motor protocol drivers and per-joint motors.

Each protocol is kept in its own module:

=========  ==========================  ===========================
Protocol   Bus driver                  Per-joint motor
=========  ==========================  ===========================
common     :class:`MotorDriver`        :class:`MotorBase`
arm        :class:`ArmDriver`          :class:`ArmMotor`
rb200      :class:`Rb200Driver`        :class:`Rb200Motor`
=========  ==========================  ===========================

Per-joint motors come from :meth:`~agx_motor_sdk.mechanism.JointGroup.motor`.

Feedback types (from C++ bindings; documented in :mod:`agx_motor_sdk._native`):
:class:`HighSpeedFeedback`, :class:`VersionInfo`, :class:`DriverStatus` (arm),
:class:`LowSpeedFeedback` (arm), :class:`Rb200LowSpeedFeedback` and
:class:`Cia402Status` (RB200).
"""

from agx_motor_sdk._native import (
    Cia402Status,
    DriverStatus,
    HighSpeedFeedback,
    LowSpeedFeedback,
    Rb200LowSpeedFeedback,
    VersionInfo,
)

from . import arm_driver as arm_driver
from . import arm_motor as arm_motor
from . import motor_base as motor_base
from . import motor_driver as motor_driver
from . import rb200_driver as rb200_driver
from . import rb200_motor as rb200_motor
from .arm_driver import ArmDriver
from .arm_motor import ArmMotor
from .motor_base import MotorBase
from .motor_driver import MotorDriver
from .rb200_driver import Rb200Driver
from .rb200_motor import Rb200Motor

__all__ = [
    "ArmDriver",
    "ArmMotor",
    "Cia402Status",
    "DriverStatus",
    "HighSpeedFeedback",
    "LowSpeedFeedback",
    "MotorBase",
    "MotorDriver",
    "Rb200Driver",
    "Rb200LowSpeedFeedback",
    "Rb200Motor",
    "VersionInfo",
    "arm_driver",
    "arm_motor",
    "motor_base",
    "motor_driver",
    "rb200_driver",
    "rb200_motor",
]
