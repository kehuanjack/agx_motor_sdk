"""Per-joint arm-protocol motor (mechanism layer).

Adds arm-only commands to :class:`~agx_motor_sdk.motor.motor_base.MotorBase`:
MIT control, current mode, collision threshold and parameters.
"""

from typing import Optional

from agx_motor_sdk._native import LowSpeedFeedback
from agx_motor_sdk.motor.arm_driver import ArmDriver
from agx_motor_sdk.motor.motor_base import MotorBase


class ArmMotor(MotorBase):
    """Per-joint arm-protocol motor (1 Mbps CAN).

    Returned by :meth:`~agx_motor_sdk.mechanism.JointGroup.motor` for
    ``protocol: arm`` joints. Common methods come from :class:`MotorBase`.

    Nested types:

    * :attr:`MotorParam` — parameter IDs for :meth:`get_param` / :meth:`set_param`

    Example::

        group = JointGroup.from_yaml("robot_motors.yaml", "head")
        group.connect()
        m = group.motor(0)
        m.set_enable()
        m.set_mit(0.0, 0.0, 0.0)
    """

    MotorParam = ArmDriver.MotorParam
    """Motor parameter IDs for :meth:`get_param` / :meth:`set_param`.

    Example: ``ArmMotor.MotorParam.TF``

    Members:

    * ``ID`` — node ID (1–15)
    * ``AC`` — profile acceleration [rad/s²], protocol max 12.56
    * ``DC`` — profile deceleration [rad/s²], protocol max 12.56
    * ``VV`` — profile velocity [rad/s], protocol max 20
    * ``IQ`` — torque-loop current limit [A], protocol max 12
    * ``OI`` — collision protection current threshold [A], protocol max 12
    * ``OT`` — collision protection time threshold [s], protocol max 2
    * ``TX`` — fast feedback mode: 0=push, 1=response
    * ``TF`` — MIT feedforward torque limit [N·m], default ±8
    * ``SO`` — joint zero offset [rad]
    * ``PP`` — position-loop proportional gain Kp
    * ``KP`` — velocity-loop proportional gain Kp
    * ``KI`` — velocity-loop integral gain Ki
    """

    def set_mit(
        self, position_rad: float, velocity_rad_s: float, torque_nm: float
    ) -> bool:
        """High-level MIT using YAML ``mit_kp``, ``mit_kd``, and ``mit_tf``.

        Joint-space inputs are transformed to motor space (YAML limits on position,
        ``polarity`` on all axes), then sent as MIT. Motor-space MIT clamps (protocol library): position
        [-12.5, 12.5] rad, velocity [-45, 45] rad/s.

        Requires ``mit_tf``, ``mit_kp``, and ``mit_kd`` on the joint or module
        YAML. Auto-connects the parent group if needed.

        Args:
            position_rad: Desired joint position [rad].
            velocity_rad_s: Desired joint velocity [rad/s].
            torque_nm: Feedforward torque [N·m]; encoded using YAML ``mit_tf``
                as half-range (typical ±8 or ±16 N·m).

        Returns:
            ``True`` if the MIT frame was sent successfully.

        Example::

            m.set_mit(0.0, 0.0, 0.0)
        """
        return bool(
            self._native.set_mit(position_rad, velocity_rad_s, torque_nm)
        )

    def get_low_speed_feedback(self) -> Optional[LowSpeedFeedback]:
        """Read cached low-speed feedback (non-blocking).

        Returns:
            Cached bus voltage, temperatures, current, and decoded status bits;
            or ``None`` if no matching frame has been received yet.
        """
        return self._native.get_low_speed_feedback()

    def set_target_current(self, current: float, timeout: float = 0.0) -> bool:
        """Current / torque mode target [A].

        Args:
            current: Desired phase current in amperes.
            timeout: Command timeout in seconds (≥ 0). ``0`` disables timeout
                monitoring.

        Returns:
            ``True`` if the command was sent successfully.
        """
        return bool(self._native.set_target_current(current, timeout))

    def set_mit_control(
        self,
        p_des: float,
        v_des: float,
        kp: float,
        kd: float,
        t_ff: float,
        t_am: float = 16.0,
    ) -> bool:
        """Low-level MIT impedance control (frame without CRC).

        Joint-space inputs (YAML limits on position, ``polarity`` applied before transmit).

        Args:
            p_des: Desired joint position in radians. Library clamp:
                ``[-12.5, 12.5]``.
            v_des: Desired joint velocity in rad/s. Library clamp:
                ``[-45, 45]``.
            kp: Position stiffness gain. Library clamp: ``[0, 500]``.
            kd: Damping / derivative gain. Library clamp: ``[-5, 5]``.
            t_ff: Feedforward torque in joint space [N·m], mapped using ``t_am`` as half-range.
            t_am: Torque mapping half-range in N·m (default 16). Values ``<= 0``
                fall back to the protocol default internally.

        Returns:
            ``True`` if the frame was sent successfully.

        Example::

            m.set_mit_control(0.0, 0.0, 50.0, 1.0, 0.0)
        """
        return bool(
            self._native.set_mit_control(p_des, v_des, kp, kd, t_ff, t_am)
        )

    def set_mit_control_crc(
        self,
        p_des: float,
        v_des: float,
        kp: float,
        kd: float,
        t_ff: float,
        t_am: float = 8.0,
    ) -> bool:
        """Low-level MIT impedance control (frame with CRC field).

        Same control law as :meth:`set_mit_control`, but uses the CRC-enabled
        firmware frame format. Joint-space inputs. Default ``t_am`` is 8 N·m for this variant.

        Args:
            p_des: Desired joint position in radians. Library clamp:
                ``[-12.5, 12.5]``.
            v_des: Desired joint velocity in rad/s. Library clamp:
                ``[-45, 45]``.
            kp: Position stiffness gain. Library clamp: ``[0, 500]``.
            kd: Damping / derivative gain. Library clamp: ``[-5, 5]``.
            t_ff: Feedforward torque in N·m.
            t_am: Torque mapping half-range in N·m (default 8).

        Returns:
            ``True`` if the frame was sent successfully.
        """
        return bool(
            self._native.set_mit_control_crc(p_des, v_des, kp, kd, t_ff, t_am)
        )

    def set_collision_threshold(
        self, current_threshold: float, time_threshold: float
    ) -> bool:
        """Configure collision protection current [A] and duration [s].

        Protocol max: current 12 A, time 2 s (enforced in protocol library).

        Args:
            current_threshold: Trip current threshold in amperes.
            time_threshold: Trip persistence time in seconds.

        Returns:
            ``True`` if the command was sent successfully.
        """
        return bool(
            self._native.set_collision_threshold(
                current_threshold, time_threshold
            )
        )

    def get_param(
        self,
        param: MotorParam,
        timeout: float = 1.0,
    ) -> Optional[float]:
        """Read a motor parameter (blocking).

        Args:
            param: e.g. :attr:`MotorParam.TF` for MIT torque limit [N·m].
            timeout: Maximum wait time in seconds for the reply (≥ 0).

        Returns:
            Parameter value on success, otherwise ``None`` on timeout or send
            failure.

        Example::

            tf = m.get_param(ArmMotor.MotorParam.TF)
        """
        return self._native.get_param(param, timeout)

    def set_param(
        self,
        param: MotorParam,
        value: float,
        timeout: float = 1.0,
    ) -> bool:
        """Write a motor parameter (blocking).

        Args:
            param: Parameter selector, e.g. :attr:`MotorParam.AC`.
            value: Value to write. Units depend on ``param``.
            timeout: Maximum wait time in seconds for the acknowledgment (≥ 0).

        Returns:
            ``True`` if the write succeeded.

        Example::

            m.set_param(ArmMotor.MotorParam.VV, 10.0)
        """
        return bool(self._native.set_param(param, value, timeout))



__all__ = ["ArmMotor"]
