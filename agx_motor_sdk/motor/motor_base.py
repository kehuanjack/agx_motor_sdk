"""Per-joint motor API shared by all protocols (mechanism layer).

Obtain instances from :meth:`~agx_motor_sdk.mechanism.JointGroup.motor`. The
returned object is the protocol subclass for that joint:

* :class:`~agx_motor_sdk.motor.arm_motor.ArmMotor` — ``protocol: arm``
* :class:`~agx_motor_sdk.motor.rb200_motor.Rb200Motor` — ``protocol: rb200``
* :class:`MotorBase` — no-op motor for an invalid index (``is_valid=False``)

Quick start::

    from agx_motor_sdk.mechanism import JointGroup

    group = JointGroup.from_yaml("robot_motors.yaml", "head")
    group.connect()
    group.configure()

    m = group.motor(group.index_of("head_pitch_joint"))
    if m.is_valid:
        m.set_enable()
        m.set_target_position(0.3)
        state = m.get_state()
"""

from typing import Any, Optional

from agx_motor_sdk._native import HighSpeedFeedback, JointConfig, JointState, VersionInfo


class MotorBase:
    """Per-joint motor, common methods (1 Mbps CAN).

    Node ID and CAN channel are fixed by YAML; you do not pass ``node_id`` to
    each command.

    Units: joint-space position [rad], velocity [rad/s], current [A].

    Invalid joint indices yield a no-op motor with ``is_valid=False`` and
    ``node_id==0``; all commands return ``False`` / ``None`` without touching CAN.
    """

    def __init__(self, native: Any) -> None:
        self._native = native

    @property
    def is_valid(self) -> bool:
        """``False`` for invalid-index no-op motors; always check before use."""
        return bool(self._native.is_valid)

    @property
    def protocol(self) -> str:
        """Joint protocol from YAML: ``"arm"`` or ``"rb200"``."""
        return str(self._native.protocol)

    @property
    def node_id(self) -> int:
        """CAN node ID (1–15); ``0`` on no-op motors."""
        return int(self._native.node_id)

    @property
    def can_channel(self) -> str:
        """Resolved CAN channel for this joint."""
        return str(self._native.can_channel)

    @property
    def name(self) -> str:
        """Joint name from YAML ``joint_name``."""
        return str(self._native.name)

    @property
    def config(self) -> JointConfig:
        """Static per-joint YAML configuration."""
        return self._native.config

    def set_enable(
        self,
        enable: bool = True,
        has_brake: bool = False,
        brake_on: bool = False,
    ) -> bool:
        """Enable or disable driver output.

        Args:
            enable: ``True`` to enable output, ``False`` to disable.
            has_brake: Whether to include the brake bit in the command frame.
            brake_on: Brake intent when ``has_brake`` is ``True``.

        Returns:
            ``True`` if the command was sent successfully.

        Example::

            m.set_enable()
            m.set_enable(False)
        """
        return bool(self._native.set_enable(enable, has_brake, brake_on))

    def get_state(self) -> Optional[JointState]:
        """Read joint-space feedback (non-blocking).

        Returns:
            :class:`~agx_motor_sdk.mechanism.JointState` with ``position_rad`` [rad],
            ``velocity_rad_s`` [rad/s], ``torque_nm`` [N·m] (polarity applied to all
            three); or ``None`` if no
            feedback has been received yet or this is a no-op motor.

        Example::

            state = m.get_state()
            if state is not None:
                print(state.position_rad, state.velocity_rad_s)
        """
        return self._native.get_state()

    def configure(self) -> bool:
        """Apply YAML profile and ``mit_tf`` to this motor.

        Profile velocity/accel/decel are sent only when their YAML values are
        ``> 0``.

        Returns:
            ``True`` if every applicable setting was applied successfully.

        Example::

            group.configure()  # all joints
            m.configure()      # this joint only
        """
        return bool(self._native.configure())

    def get_high_speed_feedback(self) -> Optional[HighSpeedFeedback]:
        """Read cached high-speed feedback in **joint space** (non-blocking).

        Returns:
            ``position`` [rad], ``velocity`` [rad/s], ``current`` [A] with YAML
            ``polarity`` applied; or ``None`` if no matching frame has been received yet.
        """
        return self._native.get_high_speed_feedback()

    def get_version(self, timeout: float = 1.0) -> Optional[VersionInfo]:
        """Query firmware version strings (blocking).

        Args:
            timeout: Maximum wait time in seconds for the reply frame (≥ 0).

        Returns:
            Firmware/software metadata on success, otherwise ``None`` on timeout
            or send failure.
        """
        return self._native.get_version(timeout)

    def set_target(self, value: float, mode: int, timeout: float = 0.0) -> bool:
        """Joint-space target with explicit control mode byte.

        Low-level alternative to :meth:`set_target_position`, :meth:`set_target_velocity`,
        and :meth:`set_target_current`. Joint-space transforms (limits, polarity) are
        applied according to ``mode`` before the command is sent in motor space.

        Args:
            value: Target in joint space. Units depend on ``mode``: ``0`` → angular
                velocity [rad/s]; ``1`` → position [rad]; ``2`` → current [A] (arm only)
                Other values are forwarded as-is per protocol.
            mode: Arm-protocol control mode byte: ``0`` = velocity, ``1`` = position,
                ``2`` = current / torque. Additional values are protocol-defined and
                not validated here.
            timeout: Command timeout in seconds (≥ 0). ``0`` disables timeout
                monitoring.

        Returns:
            ``True`` if the command was sent successfully.
        """
        return bool(self._native.set_target(value, mode, timeout))

    def set_target_position(self, position: float, timeout: float = 0.0) -> bool:
        """Joint-space position command [rad].

        Applies YAML ``lower_limit`` / ``upper_limit`` clamp (when set), then
        ``polarity`` (``motor = joint / polarity``), then sends a position-mode target.

        Args:
            position: Target position in joint space [rad].
            timeout: Command timeout in seconds (≥ 0). ``0`` disables timeout
                monitoring.

        Returns:
            ``True`` if the command was sent successfully.

        Example::

            m.set_target_position(0.5)
        """
        return bool(self._native.set_target_position(position, timeout))

    def set_target_velocity(self, velocity: float, timeout: float = 0.0) -> bool:
        """Joint-space velocity mode target [rad/s] (polarity applied).

        Args:
            velocity: Desired angular velocity in joint space [rad/s].
            timeout: Command timeout in seconds (≥ 0). ``0`` disables timeout
                monitoring.

        Returns:
            ``True`` if the command was sent successfully.
        """
        return bool(self._native.set_target_velocity(velocity, timeout))

    def set_reset(self, mode: int = 0, timeout: float = 1.0) -> bool:
        """Reset or calibration command.

        Args:
            mode: ``0`` = normal reset; ``1`` = enter calibration;
                ``2`` = save calibration.
            timeout: Maximum wait time in seconds for an acknowledgment (≥ 0).

        Returns:
            ``True`` if the command succeeded.
        """
        return bool(self._native.set_reset(mode, timeout))

    def set_clear_error(self, timeout: float = 1.0) -> bool:
        """Clear all fault and alarm states.

        Args:
            timeout: Maximum wait time in seconds for an acknowledgment (≥ 0).

        Returns:
            ``True`` if the command succeeded.
        """
        return bool(self._native.set_clear_error(timeout))

    def set_zero_offset(
        self,
        zero_offset: float = 0.0,
        save_to_flash: bool = False,
        timeout: float = 1.0,
    ) -> bool:
        """Set mechanical zero offset [rad].

        Args:
            zero_offset: Offset to store in radians. ``0`` means treat the
                current position as the new zero reference.
            save_to_flash: Whether to persist the offset to non-volatile memory.
            timeout: Maximum wait time in seconds for an acknowledgment (≥ 0).

        Returns:
            ``True`` if the command succeeded.
        """
        return bool(
            self._native.set_zero_offset(zero_offset, save_to_flash, timeout)
        )

    def set_profile_acc_dec(self, acceleration: float, deceleration: float) -> bool:
        """Set motion profile acceleration and deceleration [rad/s²].

        Protocol max per field: 12.56 rad/s² (enforced in protocol library).

        Args:
            acceleration: Positive acceleration limit in rad/s².
            deceleration: Positive deceleration limit in rad/s².

        Returns:
            ``True`` if the command was sent successfully.
        """
        return bool(self._native.set_profile_acc_dec(acceleration, deceleration))

    def set_profile_vel(self, velocity: float) -> bool:
        """Set profile velocity cap [rad/s].

        Protocol max: 20 rad/s (enforced in protocol library).

        Args:
            velocity: Maximum planned motion speed in rad/s.

        Returns:
            ``True`` if the command was sent successfully.
        """
        return bool(self._native.set_profile_vel(velocity))

    def set_current_limit(self, current: float) -> bool:
        """Set phase current limit [A].

        Protocol max: 12 A (enforced in protocol library).

        Args:
            current: Upper current limit in amperes.

        Returns:
            ``True`` if the command was sent successfully.
        """
        return bool(self._native.set_current_limit(current))



__all__ = ["MotorBase"]
