"""RB200-protocol motor driver (1 Mbps CAN, CiA402 status word).

Wraps the C++ :class:`~agx_motor_sdk._native.Rb200Driver`. Common methods come
from :class:`~agx_motor_sdk.motor.motor_driver.MotorDriver`. RB200 supports
velocity (mode 0) and position (mode 1) targets; current mode is not available.

Quick start::

    from agx_motor_sdk.can import create_can_port
    from agx_motor_sdk.motor import Rb200Driver

    bus = create_can_port(channel="can0")
    driver = Rb200Driver(bus)
    driver.set_enable(1, True)
    low = driver.get_low_speed_feedback(1)
"""

from typing import Any, Optional

from agx_motor_sdk._native import Rb200Driver as _Rb200Driver
from agx_motor_sdk._native import Rb200LowSpeedFeedback
from agx_motor_sdk.motor.motor_driver import MotorDriver


class Rb200Driver(MotorDriver):
    """RB200-protocol motor driver.

    Node IDs are **1–15**. ``AGX_MOTOR_SDK_RB200_LIB`` overrides the protocol
    library path.
    """

    _NATIVE_CLASS = _Rb200Driver

    def __init__(self, can_port: Optional[Any] = None) -> None:
        """Create driver and optionally bind a CAN port (see :class:`MotorDriver`)."""
        super().__init__(can_port)

    def get_low_speed_feedback(self, node_id: int) -> Optional[Rb200LowSpeedFeedback]:
        """Read cached RB200 low-speed feedback (non-blocking, CAN ID ``0x530 + node``).

        Args:
            node_id: Motor node ID, 1–15.

        Returns:
            CiA402 status word, error code, TPDO2 counter and decoded status bits;
            or ``None`` if no matching frame has been received yet.
        """
        return self._native.get_low_speed_feedback(node_id)


__all__ = ["Rb200Driver"]
