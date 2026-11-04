"""agx_motor_sdk — AgileX CAN motor driver SDK.

Subpackages
-----------

``can``
    CAN port factories (:func:`~agx_motor_sdk.can.create_can_port`).
``motor``
    Protocol drivers: :class:`~agx_motor_sdk.motor.ArmDriver`,
    :class:`~agx_motor_sdk.motor.Rb200Driver`.
``mechanism``
    Multi-joint :class:`~agx_motor_sdk.mechanism.JointGroup` from YAML.

Top-level shortcuts (same as ``motor`` / ``can`` exports)::

    from agx_motor_sdk import ArmDriver, Rb200Driver, create_can_port
"""

from agx_motor_sdk.can import create_can_port, create_can_port_config
from agx_motor_sdk.motor import (
    ArmDriver,
    Cia402Status,
    DriverStatus,
    HighSpeedFeedback,
    LowSpeedFeedback,
    MotorDriver,
    Rb200Driver,
    Rb200LowSpeedFeedback,
    VersionInfo,
)
from agx_motor_sdk.version import __protocol_version__, __version__

__all__ = [
    "__version__",
    "__protocol_version__",
    "ArmDriver",
    "Cia402Status",
    "DriverStatus",
    "HighSpeedFeedback",
    "LowSpeedFeedback",
    "MotorDriver",
    "Rb200Driver",
    "Rb200LowSpeedFeedback",
    "VersionInfo",
    "create_can_port",
    "create_can_port_config",
]
