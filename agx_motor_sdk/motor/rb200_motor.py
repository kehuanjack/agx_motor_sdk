"""Per-joint RB200-protocol motor (mechanism layer)."""

from typing import Optional

from agx_motor_sdk._native import Rb200LowSpeedFeedback
from agx_motor_sdk.motor.motor_base import MotorBase


class Rb200Motor(MotorBase):
    """Per-joint RB200 motor (CiA402 status word, velocity/position modes).

    Returned by :meth:`~agx_motor_sdk.mechanism.JointGroup.motor` for
    ``protocol: rb200`` joints. Common methods come from :class:`MotorBase`.
    Current mode and MIT are not available on RB200.
    """

    def get_low_speed_feedback(self) -> Optional[Rb200LowSpeedFeedback]:
        """Read cached RB200 low-speed feedback (CAN ID ``0x530 + node``).

        Returns:
            Status word, error code, TPDO2 counter and decoded CiA402 bits;
            or ``None`` if no frame has been received yet.
        """
        return self._native.get_low_speed_feedback()


__all__ = ["Rb200Motor"]
