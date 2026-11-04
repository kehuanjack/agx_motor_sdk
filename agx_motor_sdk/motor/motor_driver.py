"""Common motor driver API shared by all protocols.

Use a protocol class instead of this base:

* :class:`~agx_motor_sdk.motor.arm_driver.ArmDriver` — arm protocol
* :class:`~agx_motor_sdk.motor.rb200_driver.Rb200Driver` — RB200 protocol

Both run on 1 Mbps CAN and can share one CAN port (frames are routed by node ID).
"""

import time
from typing import Any, Optional, Union

from agx_motor_sdk._native import HighSpeedFeedback, VersionInfo
from agx_motor_sdk.can.port_utils import native_can_port
from agx_motor_sdk.can.socket_can import Message


class MotorDriver:
    """Common driver methods (enable, targets, profile, version).

    Node IDs are **1–15**. Units: position [rad], velocity [rad/s], current [A].
    Subclasses set :attr:`_NATIVE_CLASS`.
    """

    _NATIVE_CLASS: Any = None

    def __init__(self, can_port: Optional[Any] = None) -> None:
        """Create driver and optionally bind a CAN port.

        Args:
            can_port: Object from :func:`~agx_motor_sdk.can.create_can_port`, a
                :class:`~agx_motor_sdk._native.SocketCanPort`, or ``None`` to bind
                later via :meth:`attach_can_port`.
        """
        if self._NATIVE_CLASS is None:
            raise TypeError("use ArmDriver or Rb200Driver")
        self._can_port = can_port
        self._native = self._NATIVE_CLASS(native_can_port(can_port))

    @property
    def protocol(self) -> str:
        """Loaded protocol library: ``"arm"`` or ``"rb200"``."""
        return str(self._native.protocol)

    def close(self) -> None:
        """Detach CAN and destroy the native protocol handle."""
        self._native.close()

    def attach_can_port(self, can_port: Any) -> None:
        """Bind an opened CAN port for TX/RX.

        Args:
            can_port: Opened CAN object. Usually created by
                :func:`~agx_motor_sdk.can.create_can_port`. The port must remain
                open until :meth:`close`; this driver does not own its lifetime.
        """
        self._can_port = can_port
        self._native.attach_can_port(native_can_port(can_port))

    def handle_rx_once(
        self,
        id: Optional[int] = None,
        data: Optional[Union[bytes, bytearray]] = None,
        dlc: Optional[int] = None,
        msg: Optional[Message] = None,
        timestamp: Optional[float] = None,
    ) -> bool:
        """Parse one received CAN frame and update internal caches.

        Pass either a ``python-can`` :class:`~can.message.Message` **or** raw
        ``id``/``data`` fields.

        Args:
            id: CAN arbitration ID. Used only when ``msg`` is not provided.
            data: CAN payload bytes. Used only when ``msg`` is not provided.
            dlc: Optional payload truncation length. When provided, only the
                first ``dlc`` bytes of ``data`` are forwarded.
            msg: ``python-can`` message object. If provided, ``id`` / ``data`` /
                ``dlc`` are ignored.
            timestamp: Receive timestamp in seconds. Used only for raw
                ``id``/``data`` input. If omitted, current wall-clock time is used.

        Returns:
            ``True`` if the frame was recognized and state was updated.
        """
        if msg is not None:
            frame_id = int(msg.arbitration_id)
            payload = bytes(msg.data)
            ts = float(msg.timestamp) if msg.timestamp else time.time()
        else:
            frame_id = int(id or 0)
            payload = bytes(data or b"")
            if dlc is not None:
                payload = payload[: int(dlc)]
            ts = float(timestamp) if timestamp is not None else time.time()
        return self._native.handle_rx_once(frame_id, payload, int(ts * 1e9))

    def get_high_speed_feedback(self, node_id: int) -> Optional[HighSpeedFeedback]:
        """Read cached high-speed feedback (non-blocking).

        Args:
            node_id: Motor node ID, 1–15.

        Returns:
            ``position`` [rad], ``velocity`` [rad/s], ``current`` [A]; or ``None``
            if no matching frame has been received yet.
        """
        return self._native.get_high_speed_feedback(node_id)

    def get_version(self, node_id: int, timeout: float = 1.0) -> Optional[VersionInfo]:
        """Query firmware version strings (blocking).

        Args:
            node_id: Motor node ID, 1–15.
            timeout: Maximum wait time in seconds for the reply frame.

        Returns:
            Firmware/software metadata on success, otherwise ``None`` on timeout
            or send failure.
        """
        return self._native.get_version(node_id, timeout)

    def set_enable(
        self,
        node_id: int,
        enable: bool = True,
        has_brake: bool = False,
        brake_on: bool = False,
    ) -> bool:
        """Enable or disable driver output.

        Args:
            node_id: Motor node ID, 1–15.
            enable: ``True`` to enable output, ``False`` to disable.
            has_brake: Whether to include the brake bit in the command frame.
            brake_on: Brake intent when ``has_brake`` is ``True``.
        """
        return self._native.set_enable(node_id, enable, has_brake, brake_on)

    def set_target(
        self, node_id: int, value: float, mode: int, timeout: float = 0.0
    ) -> bool:
        """Generic target command with explicit control mode byte.

        Low-level alternative to :meth:`set_target_velocity` and :meth:`set_target_position`.

        Args:
            node_id: Motor node ID, 1–15.
            value: Target in motor space. Units depend on ``mode``: ``0`` → angular
                velocity [rad/s]; ``1`` → position [rad]; ``2`` → phase current [A]
                (arm only). Other values are forwarded as-is per protocol.
            mode: Control mode byte: ``0`` = velocity, ``1`` = position,
                ``2`` = current (arm only; RB200 rejects it), ``0xFF`` = unchanged.
            timeout: Command timeout in seconds (≥ 0). ``0`` disables timeout
                monitoring.

        Returns:
            ``True`` if the command was sent successfully.
        """
        return self._native.set_target(node_id, value, mode, timeout)

    def set_target_velocity(
        self, node_id: int, velocity: float, timeout: float = 0.0
    ) -> bool:
        """Velocity mode target [rad/s].

        Args:
            node_id: Motor node ID, 1–15.
            velocity: Desired angular velocity in rad/s.
            timeout: Command timeout in seconds. ``0`` disables timeout monitoring.
        """
        return self._native.set_target_velocity(node_id, velocity, timeout)

    def set_target_position(
        self, node_id: int, position: float, timeout: float = 0.0
    ) -> bool:
        """Position mode target [rad].

        Args:
            node_id: Motor node ID, 1–15.
            position: Desired joint position in radians.
            timeout: Command timeout in seconds. ``0`` disables timeout monitoring.
        """
        return self._native.set_target_position(node_id, position, timeout)

    def set_reset(self, node_id: int, mode: int = 0, timeout: float = 1.0) -> bool:
        """Reset or calibration command.

        Args:
            mode: ``0`` = normal reset; ``1`` = enter calibration;
                ``2`` = save calibration.
        """
        return self._native.set_reset(node_id, mode, timeout)

    def set_clear_error(self, node_id: int, timeout: float = 1.0) -> bool:
        """Clear all fault and alarm states.

        Args:
            node_id: Motor node ID, 1–15.
            timeout: Maximum wait time in seconds for an acknowledgment.
        """
        return self._native.set_clear_error(node_id, timeout)

    def set_zero_offset(
        self,
        node_id: int,
        zero_offset: float = 0.0,
        save_to_flash: bool = False,
        timeout: float = 1.0,
    ) -> bool:
        """Set mechanical zero offset [rad].

        Args:
            node_id: Motor node ID, 1–15.
            zero_offset: Offset to store in radians. ``0`` means treat the
                current position as the new zero reference.
            save_to_flash: Whether to persist the offset to non-volatile memory.
            timeout: Maximum wait time in seconds for an acknowledgment.
        """
        return self._native.set_zero_offset(
            node_id, zero_offset, save_to_flash, timeout
        )

    def set_profile_acc_dec(
        self, node_id: int, acceleration: float, deceleration: float
    ) -> bool:
        """Set motion profile acceleration and deceleration [rad/s²].

        Args:
            node_id: Motor node ID, 1–15.
            acceleration: Positive acceleration limit in rad/s².
            deceleration: Positive deceleration limit in rad/s².
        """
        return self._native.set_profile_acc_dec(node_id, acceleration, deceleration)

    def set_profile_vel(self, node_id: int, velocity: float) -> bool:
        """Set profile velocity cap [rad/s].

        Args:
            node_id: Motor node ID, 1–15.
            velocity: Maximum planned motion speed in rad/s.
        """
        return self._native.set_profile_vel(node_id, velocity)

    def set_current_limit(self, node_id: int, current: float) -> bool:
        """Set phase current limit [A].

        Args:
            node_id: Motor node ID, 1–15.
            current: Upper current limit in amperes.
        """
        return self._native.set_current_limit(node_id, current)



__all__ = ["MotorDriver"]
