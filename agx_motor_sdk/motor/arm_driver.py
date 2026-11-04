"""Arm-protocol motor driver (1 Mbps CAN).

This module wraps the C++ :class:`~agx_motor_sdk._native.ArmDriver` with:

* Common methods from :class:`~agx_motor_sdk.motor.motor_driver.MotorDriver`
* Arm-only commands: current mode, MIT, collision threshold, parameters

Quick start::

    from agx_motor_sdk.can import create_can_port
    from agx_motor_sdk.motor import ArmDriver

    bus = create_can_port(channel="can0")
    driver = ArmDriver(bus)
    driver.set_enable(1, True)
    state = driver.get_high_speed_feedback(1)
"""

from typing import Any, Optional

from agx_motor_sdk._native import ArmDriver as _ArmDriver
from agx_motor_sdk._native import LowSpeedFeedback
from agx_motor_sdk.motor.motor_driver import MotorDriver


class ArmDriver(MotorDriver):
    """Arm-protocol motor driver.

    Common methods are inherited from :class:`MotorDriver`.

    Node IDs are **1–15**. Units: position [rad], velocity [rad/s], current [A].

    Nested types:

    * :attr:`MotorParam` — parameter IDs for :meth:`get_param` / :meth:`set_param`

    """

    _NATIVE_CLASS = _ArmDriver
    MotorParam = _ArmDriver.MotorParam
    """Motor parameter IDs for :meth:`get_param` / :meth:`set_param`.

    Example: ``ArmDriver.MotorParam.TF``

    Members:

    * ``ID`` — node ID (1–15)
    * ``AC`` — profile acceleration [rad/s²], max 12.56
    * ``DC`` — profile deceleration [rad/s²], max 12.56
    * ``VV`` — profile velocity [rad/s], max 20
    * ``IQ`` — torque-loop current limit [A], max 12
    * ``OI`` — collision protection current threshold [A], max 12
    * ``OT`` — collision protection time threshold [s], max 2
    * ``TX`` — fast feedback mode: 0=push, 1=response
    * ``TF`` — MIT feedforward torque limit [N·m], default ±8
    * ``SO`` — joint zero offset [rad]
    * ``PP`` — position-loop proportional gain Kp
    * ``KP`` — velocity-loop proportional gain Kp
    * ``KI`` — velocity-loop integral gain Ki
    """

    def __init__(self, can_port: Optional[Any] = None) -> None:
        """Create driver and optionally bind a CAN port.

        Args:
            can_port: Object from :func:`~agx_motor_sdk.can.create_can_port`, a
                :class:`~agx_motor_sdk._native.SocketCanPort`, or ``None`` to bind
                later via :meth:`attach_can_port`. ``AGX_MOTOR_SDK_ARM_LIB``
                overrides the protocol library path.

        Example::

            from agx_motor_sdk.can import create_can_port
            from agx_motor_sdk.motor import ArmDriver

            bus = create_can_port(channel="can0")
            driver = ArmDriver(bus)
            driver.set_enable(1, True)
        """
        super().__init__(can_port)

    def get_low_speed_feedback(self, node_id: int) -> Optional[LowSpeedFeedback]:
        """Read cached low-speed feedback (non-blocking).

        Args:
            node_id: Motor node ID, 1–15.

        Returns:
            Cached bus voltage, temperatures, current, and decoded status bits;
            or ``None`` if no matching frame has been received yet.
        """
        return self._native.get_low_speed_feedback(node_id)

    def set_target_current(
        self, node_id: int, current: float, timeout: float = 0.0
    ) -> bool:
        """Current / torque mode target [A].

        Args:
            node_id: Motor node ID, 1–15.
            current: Desired phase current in amperes.
            timeout: Command timeout in seconds. ``0`` disables timeout monitoring.
        """
        return self._native.set_target_current(node_id, current, timeout)

    def set_mit_control(
        self,
        node_id: int,
        p_des: float,
        v_des: float,
        kp: float,
        kd: float,
        t_ff: float,
        t_am: float = 16.0,
    ) -> bool:
        """MIT impedance control (frame without CRC).

        This is the most flexible arm-protocol control mode. It sends desired
        position, velocity, impedance gains, and feedforward torque in a single
        command frame. Values are clamped inside the protocol library before
        transmission.

        Args:
            node_id: Motor node ID, 1–15.
            p_des: Desired joint position in radians. Library clamp:
                ``[-12.5, 12.5]``.
            v_des: Desired joint velocity in rad/s. Library clamp:
                ``[-45, 45]``.
            kp: Position stiffness gain. Larger values pull harder toward
                ``p_des``. Library clamp: ``[0, 500]``.
            kd: Damping / derivative gain. Larger values resist velocity error.
                Library clamp: ``[-5, 5]``.
            t_ff: Feedforward torque in N·m. This is mapped into the protocol
                torque field using ``t_am`` as the half-range.
            t_am: Torque mapping half-range in N·m. For example, with
                ``t_am=16``, the encoded torque range is approximately
                ``[-16, +16]`` N·m. Values ``<= 0`` fall back to the protocol
                default internally.

        Returns:
            ``True`` if the frame was sent successfully.
        """
        return self._native.set_mit_control(
            node_id, p_des, v_des, kp, kd, t_ff, t_am
        )

    def set_mit_control_crc(
        self,
        node_id: int,
        p_des: float,
        v_des: float,
        kp: float,
        kd: float,
        t_ff: float,
        t_am: float = 8.0,
    ) -> bool:
        """MIT impedance control (frame with CRC field).

        Same control law as :meth:`set_mit_control`, but uses the CRC-enabled
        firmware frame format.

        Args:
            node_id: Motor node ID, 1–15.
            p_des: Desired joint position in radians. Library clamp:
                ``[-12.5, 12.5]``.
            v_des: Desired joint velocity in rad/s. Library clamp:
                ``[-45, 45]``.
            kp: Position stiffness gain. Library clamp: ``[0, 500]``.
            kd: Damping / derivative gain. Library clamp: ``[-5, 5]``.
            t_ff: Feedforward torque in N·m.
            t_am: Torque mapping half-range in N·m for CRC frame mode. The
                protocol default is typically ``8``.

        Returns:
            ``True`` if the frame was sent successfully.
        """
        return self._native.set_mit_control_crc(
            node_id, p_des, v_des, kp, kd, t_ff, t_am
        )

    def set_collision_threshold(
        self, node_id: int, current_threshold: float, time_threshold: float
    ) -> bool:
        """Configure collision protection current [A] and duration [s].

        Args:
            node_id: Motor node ID, 1–15.
            current_threshold: Trip current threshold in amperes.
            time_threshold: Trip persistence time in seconds.
        """
        return self._native.set_collision_threshold(
            node_id, current_threshold, time_threshold
        )

    def get_param(
        self,
        node_id: int,
        param: MotorParam,
        timeout: float = 1.0,
    ) -> Optional[float]:
        """Read a motor parameter (blocking).

        Args:
            node_id: Motor node ID, 1–15.
            param: e.g. :attr:`MotorParam.TF` for MIT torque limit [N·m].
            timeout: Maximum wait time in seconds for the reply.

        Returns:
            Parameter value on success, otherwise ``None`` on timeout or send
            failure.
        """
        return self._native.get_param(node_id, param, timeout)

    def set_param(
        self,
        node_id: int,
        param: MotorParam,
        value: float,
        timeout: float = 1.0,
    ) -> bool:
        """Write a motor parameter (blocking).

        Args:
            node_id: Motor node ID, 1–15.
            param: Parameter selector, e.g. :attr:`MotorParam.AC`.
            value: Value to write. Units depend on ``param``.
            timeout: Maximum wait time in seconds for the acknowledgment.
        """
        return self._native.set_param(node_id, param, value, timeout)



__all__ = ["ArmDriver"]
