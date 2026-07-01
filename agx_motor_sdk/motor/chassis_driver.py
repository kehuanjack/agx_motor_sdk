"""Python bindings for chassis-protocol motors (ctypes + libagx_motor_protocol_chassis)."""
from typing import Any, Optional, Union

from ctypes import (
    byref,
    cast,
    c_int,
    c_uint8,
    c_uint16,
    c_uint32,
    c_uint64,
    POINTER,
)
from can.message import Message

from agx_motor_sdk.can.backend import CanPort
from ._protocol_common import (
    TX_FN,
    TxCallback,
    DriverStatus,
    HighSpeedFeedback,
    LowSpeedFeedback,
    _chassis_cache,
    _MotorHighSpeedFeedback,
    _MotorLowSpeedFeedback,
    high_speed_from_ctypes,
    low_speed_from_ctypes,
    message_timestamp,
    timestamp_to_ns,
)


class ChassisDriver:
    """Chassis-protocol motor driver (500 kbps CAN).

    Units: position [INC], velocity [RPM], current [A], profile [RPM/s]. Node IDs: 1–15.
    Different CAN format from :class:`ArmDriver`. Do not attach both drivers to one
    ``CanPort``; dispatch RX by ``node_id`` if protocols share a bus.

    Low-speed ``status``: ``collision_tripped`` = sensor fault, ``stall_tripped`` = homed,
    ``enabled`` = status bit6 set. No MIT, param, version, or collision-threshold APIs.
    """

    def __init__(self, can_port: Optional[CanPort] = None) -> None:
        """Load libagx_motor_protocol_chassis and create a native handle.

        Args:
            can_port: Optional CAN port. If it supports ``set_callback``,
                ``attach_can_port`` is called automatically.

        Raises:
            RuntimeError: Protocol library missing or ``motor_create`` failed.
        """
        self._lib = None
        self._handle = None
        self._tx_cb: Optional[TxCallback] = None
        self._can_port: Optional[CanPort] = None

        lib = _chassis_cache.get()
        if lib is None:
            raise RuntimeError(
                "libagx_motor_protocol_chassis not available: {0}".format(
                    _chassis_cache.load_error() or "unknown error"
                )
            )
        self._lib = lib
        self._handle = lib.motor_create()
        if not self._handle:
            raise RuntimeError("motor_create failed")
        if can_port is not None and hasattr(can_port, "set_callback"):
            self.attach_can_port(can_port)

    def _make_tx_callback(self, can_port: CanPort) -> TxCallback:
        @TX_FN
        def tx(_ctx, id, data_ptr, dlc):
            try:
                payload = bytes(data_ptr[i] for i in range(int(dlc)))
                msg = Message(arbitration_id=int(id), data=payload, is_extended_id=False)
                can_port.send(msg)
                return 1
            except Exception:
                return 0

        return tx

    def _on_can_frame(self, msg: Message) -> None:
        self.handle_rx_once(msg=msg)

    def set_tx_callback(self, fn: TxCallback, ctx: Optional[Any] = None) -> None:
        """Register a low-level ctypes TX callback (alternative to ``attach_can_port``).

        Args:
            fn: Callback ``(ctx, can_id, data_ptr, dlc) -> int``; non-zero means send OK.
            ctx: User context passed as the first argument to ``fn``.
        """
        self._lib.motor_set_tx_callback(self._handle, fn, ctx)
        self._tx_cb = fn

    def attach_can_port(self, can_port: CanPort) -> None:
        """Bind CAN for TX/RX.

        Args:
            can_port: Connected port with ``set_callback`` / ``recv`` / ``send``.
                Must stay valid until ``close()``.

        Raises:
            RuntimeError: TX callback registration failed.
        """
        self.set_tx_callback(self._make_tx_callback(can_port))
        if not self.has_tx_callback():
            raise RuntimeError("failed to attach tx callback")
        self._can_port = can_port
        can_port.set_callback(self._on_can_frame)

    def has_tx_callback(self) -> bool:
        """Return whether a TX callback is registered.

        Returns:
            True if the native layer has a TX callback.
        """
        return bool(self._lib.motor_has_tx_callback(self._handle))

    def close(self) -> None:
        """Clear the CAN callback and destroy the native handle (``motor_destroy``)."""
        can_port = self._can_port
        if can_port is not None:
            can_port.clear_callback()
        self._can_port = None
        if self._handle and self._lib:
            try:
                self._lib.motor_set_tx_callback(self._handle, TX_FN(), None)
            except Exception:
                pass
            try:
                self._lib.motor_destroy(self._handle)
            except Exception:
                pass
            self._handle = None
        self._tx_cb = None

    def handle_rx_once(
        self,
        id: Optional[int] = None,
        data: Optional[Union[bytes, bytearray]] = None,
        dlc: Optional[int] = None,
        msg: Optional[Message] = None,
        timestamp: Optional[float] = None,
    ) -> bool:
        """Parse one received frame and update cached feedback state.

        Args:
            id: CAN arbitration ID (used only if ``msg`` is None).
            data: Payload bytes (used only if ``msg`` is None).
            dlc: Data length; defaults to ``len(data)``.
            msg: ``python-can`` Message from ``can_port.recv()``; preferred input.
            timestamp: Receive time [s]; taken from ``msg`` if omitted.

        Returns:
            True if the frame was recognized and caches were updated.

        Raises:
            ValueError: Neither ``msg`` nor both ``id`` and ``data`` were given.
        """
        if msg is not None:
            id = int(msg.arbitration_id)
            data = msg.data
            dlc = len(msg.data)
            if timestamp is None:
                timestamp = message_timestamp(msg)
        if id is None or data is None:
            raise ValueError("need msg= or (id, data)")
        n = int(dlc) if dlc is not None else len(data)
        raw = data if isinstance(data, (bytes, bytearray)) else bytes(data)
        arr = (c_uint8 * max(n, 1))(*raw[:n]) if n > 0 else (c_uint8 * 1)()
        ts = c_uint64(timestamp_to_ns(timestamp) if timestamp is not None else 0)
        return bool(
            self._lib.motor_handle_rx_once(
                self._handle, c_uint32(id), cast(arr, POINTER(c_uint8)), c_uint8(n), ts
            )
        )

    def get_high_speed_feedback(self, node_id: int) -> Optional[HighSpeedFeedback]:
        """Read cached high-speed feedback.

        Args:
            node_id: Motor node ID, 1–15.

        Returns:
            position [INC], velocity [RPM], current [A], timestamp [s]; or None if no cache.
        """
        out = _MotorHighSpeedFeedback()
        if not self._lib.motor_get_high_speed_feedback(self._handle, node_id, byref(out)):
            return None
        return high_speed_from_ctypes(out)

    def get_low_speed_feedback(self, node_id: int) -> Optional[LowSpeedFeedback]:
        """Read cached low-speed feedback.

        Args:
            node_id: Motor node ID, 1–15.

        Returns:
            Voltage, temperatures, status bits, timestamp [s]; or None if no cache.
        """
        out = _MotorLowSpeedFeedback()
        if not self._lib.motor_get_low_speed_feedback(self._handle, node_id, byref(out)):
            return None
        return low_speed_from_ctypes(out)

    def _set_target(self, node_id: int, value: float, mode: int, timeout: float = 0.0) -> bool:
        return bool(self._lib.motor_set_target(self._handle, node_id, value, mode, timeout))

    def set_target_rpm(self, node_id: int, rpm: float, timeout: float = 0.0) -> bool:
        """Send RPM-mode target (0x410 mode 0).

        Args:
            node_id: Motor node ID, 1–15.
            rpm: Target speed [RPM].
            timeout: Command timeout [s]; 0 disables timeout monitoring.

        Returns:
            True if the command was sent successfully.
        """
        return self._set_target(node_id, rpm, 0, timeout)

    def set_target_inc(self, node_id: int, inc: float, timeout: float = 0.0) -> bool:
        """Send position-mode target (0x410 mode 1).

        Args:
            node_id: Motor node ID, 1–15.
            inc: Target encoder increment [INC].
            timeout: Command timeout [s]; 0 disables timeout monitoring.

        Returns:
            True if the command was sent successfully.
        """
        return self._set_target(node_id, inc, 1, timeout)

    def set_target_current(self, node_id: int, current: float, timeout: float = 0.0) -> bool:
        """Send current-mode target (0x410 mode 2).

        Args:
            node_id: Motor node ID, 1–15.
            current: Target phase current [A].
            timeout: Command timeout [s]; 0 disables timeout monitoring.

        Returns:
            True if the command was sent successfully.
        """
        return self._set_target(node_id, current, 2, timeout)

    def set_enable(
        self, node_id: int, enable: bool = True, has_brake: bool = False, brake_on: bool = False
    ) -> bool:
        """Enable or disable driver output (0x420).

        Args:
            node_id: Motor node ID, 1–15.
            enable: True to enable, False to disable.
            has_brake: True to include brake control in the frame.
            brake_on: Brake intent when ``has_brake`` is True.

        Returns:
            True if the command was sent successfully.
        """
        return bool(
            self._lib.motor_set_enable(
                self._handle, node_id, int(enable), int(has_brake), int(brake_on)
            )
        )

    def set_reset(self, node_id: int, mode: int = 0, timeout: float = 1.0) -> bool:
        """Reset or enter/save calibration (0x000).

        Args:
            node_id: Motor node ID, 1–15.
            mode: 0 = normal reset; 1 = enter calibration; 2 = save calibration.
            timeout: Blocking ACK wait [s].

        Returns:
            True on successful send (and ACK when ``timeout`` > 0).
        """
        out_ok = c_int()
        if not self._lib.motor_set_reset(
            self._handle, node_id, int(mode), float(timeout), byref(out_ok)
        ):
            return False
        if timeout > 0:
            return bool(out_ok.value)
        return True

    def set_clear_error(self, node_id: int, timeout: float = 1.0) -> bool:
        """Clear all fault and alarm states (0x010).

        Args:
            node_id: Motor node ID, 1–15.
            timeout: Blocking ACK wait [s].

        Returns:
            True on successful send (and ACK when ``timeout`` > 0).
        """
        out_ok = c_int()
        if not self._lib.motor_set_clear_error(
            self._handle, node_id, 0, float(timeout), byref(out_ok)
        ):
            return False
        if timeout > 0:
            return bool(out_ok.value)
        return True

    def set_zero_offset(
        self,
        node_id: int,
        zero_offset: float = 0.0,
        save_to_flash: bool = False,
        timeout: float = 1.0,
    ) -> bool:
        """Set encoder zero offset (0x020).

        Args:
            node_id: Motor node ID, 1–15.
            zero_offset: Offset [INC]; 0 marks the current position as zero.
            save_to_flash: True to persist to non-volatile memory.
            timeout: Blocking ACK wait [s].

        Returns:
            True on successful send (and ACK when ``timeout`` > 0).
        """
        out_ok = c_int()
        if not self._lib.motor_set_zero_offset(
            self._handle,
            node_id,
            float(zero_offset),
            int(save_to_flash),
            float(timeout),
            byref(out_ok),
        ):
            return False
        if timeout > 0:
            return bool(out_ok.value)
        return True

    def set_profile_acc_dec(self, node_id: int, acceleration: float, deceleration: float) -> bool:
        """Set motion profile acceleration and deceleration (0x430).

        Args:
            node_id: Motor node ID, 1–15.
            acceleration: Profile acceleration [RPM/s] (clamped in library).
            deceleration: Profile deceleration [RPM/s] (clamped in library).

        Returns:
            True if the command was sent successfully.
        """
        return bool(
            self._lib.motor_set_profile_acc_dec(self._handle, node_id, acceleration, deceleration)
        )

    def set_profile_vel(self, node_id: int, rpm: float) -> bool:
        """Set motion profile velocity limit (0x440).

        Args:
            node_id: Motor node ID, 1–15.
            rpm: Profile velocity cap [RPM] (clamped in library).

        Returns:
            True if the command was sent successfully.
        """
        return bool(self._lib.motor_set_profile_vel(self._handle, node_id, rpm))

    def set_current_limit(self, node_id: int, current: float) -> bool:
        """Set phase current limit (0x450).

        Args:
            node_id: Motor node ID, 1–15.
            current: Current limit [A].

        Returns:
            True if the command was sent successfully.
        """
        return bool(self._lib.motor_set_current_limit(self._handle, node_id, current))

    def set_stiffness(self, node_id: int, stiffness: int) -> bool:
        """Set chassis stiffness (0x470).

        Args:
            node_id: Motor node ID, 1–15.
            stiffness: Stiffness wire value; library clamps to 10–2000.

        Returns:
            True if the command was sent successfully.
        """
        return bool(
            self._lib.motor_set_stiffness(self._handle, node_id, c_uint16(int(stiffness)))
        )


__all__ = [
    "ChassisDriver",
    "DriverStatus",
    "HighSpeedFeedback",
    "LowSpeedFeedback",
]
