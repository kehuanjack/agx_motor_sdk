"""agx_motor_sdk — AgileX CAN motor driver SDK.

Subpackages
-----------

``can``
    CAN port factories (:func:`~agx_motor_sdk.can.create_can_port`).
``motor``
    Low-level driver: :class:`~agx_motor_sdk.motor.ArmDriver`.
``mechanism``
    Multi-joint :class:`~agx_motor_sdk.mechanism.ArmJointGroup` from YAML.

Top-level shortcuts (same as ``motor`` / ``can`` exports)::

    from agx_motor_sdk import ArmDriver, create_can_port
"""

from agx_motor_sdk.can import create_can_port, create_can_port_config
from agx_motor_sdk.motor import (
    ArmDriver,
    DriverStatus,
    HighSpeedFeedback,
    LowSpeedFeedback,
    VersionInfo,
)
from agx_motor_sdk.version import __protocol_version__, __version__

__all__ = [
    "__version__",
    "__protocol_version__",
    "ArmDriver",
    "DriverStatus",
    "HighSpeedFeedback",
    "LowSpeedFeedback",
    "VersionInfo",
    "create_can_port",
    "create_can_port_config",
]
