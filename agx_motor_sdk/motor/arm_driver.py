"""Python bindings for arm-protocol motors (ctypes + libagx_motor_protocol_arm)."""
from enum import Enum
from typing import Any, Optional, Union

from ctypes import (
    byref,
    cast,
    c_float,
    c_int,
    c_uint8,
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
    VersionInfo,
    _arm_cache,
    _MotorHighSpeedFeedback,
    _MotorLowSpeedFeedback,
    _MotorVersionInfo,
    high_speed_from_ctypes,
    low_speed_from_ctypes,
    version_from_ctypes,
    message_timestamp,
    timestamp_to_ns,
)


class ArmDriver:
    """Arm-protocol motor driver (1 Mbps CAN).

    Units: position [rad], velocity [rad/s], current [A]. Node IDs: 1–15.
    With ``attach_can_port``, RX is passive: poll ``can_port.recv()`` in a loop;
    frames are forwarded to ``handle_rx_once`` via the port callback.
    """

    class MotorParam(Enum):
        """Parameter IDs for ``get_param`` / ``set_param`` (two ASCII bytes each).

        - ``ID``: Node ID (integer, range 1–15).
        - ``AC``: Profile acceleration [rad/s²], max 12.56.
        - ``DC``: Profile deceleration [rad/s²], max 12.56.
        - ``VV``: Profile velocity [rad/s], max 20.
        - ``IQ``: Torque-loop max current limit [A], max 12.
        - ``OI``: Collision protection current threshold [A], max 12.
        - ``OT``: Collision protection time threshold [s], max 2; OT=30 and OI=30 disables protection.
        - ``TX``: Fast feedback mode: 0=active push, 1=response (default 0, stored in flash).
        - ``TF``: MIT feedforward torque limit [N·m] (``t_am``), default ±8.
        - ``SO``: Joint zero offset [rad].
        - ``PP``: Position-loop proportional gain Kp.
        - ``KP``: Velocity-loop proportional gain Kp.
        - ``KI``: Velocity-loop integral gain Ki.
        """

        ID = (ord("i"), ord("d"))
        AC = (ord("a"), ord("c"))
        DC = (ord("d"), ord("c"))
        VV = (ord("v"), ord("v"))
        IQ = (ord("i"), ord("q"))
        OI = (ord("o"), ord("i"))
        OT = (ord("o"), ord("t"))
        TX = (ord("t"), ord("x"))
        TF = (ord("t"), ord("f"))
        SO = (ord("s"), ord("o"))
        PP = (ord("p"), ord("p"))
        KP = (ord("k"), ord("p"))
        KI = (ord("k"), ord("i"))

    def __init__(self, can_port: Optional[CanPort] = None) -> None:
        """Load libagx_motor_protocol_arm and create a native handle.

        Args:
            can_port: Optional CAN port. If it supports ``set_callback``,
                ``attach_can_port`` is called automatically.

        Raises:
            RuntimeError: Protocol library missing or ``motor_create`` failed.
        """
        self._lib: Optional[Any] = None
        self._handle = None
        self._tx_cb: Optional[TxCallback] = None
        self._can_port: Optional[CanPort] = None

        lib = _arm_cache.get()
        if lib is None:
            raise RuntimeError(
                "libagx_motor_protocol_arm not available: {0}".format(_arm_cache.load_error() or "unknown error")
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
        """Parse one received frame and update cached feedback / param / version state.

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
            Named tuple with position [rad], velocity [rad/s], current [A],
            timestamp [s]; or None if no data cached yet.
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
            Voltage, temperatures, status bits, and timestamp [s]; or None if no cache.
        """
        out = _MotorLowSpeedFeedback()
        if not self._lib.motor_get_low_speed_feedback(self._handle, node_id, byref(out)):
            return None
        return low_speed_from_ctypes(out)

    def _set_target(self, node_id: int, value: float, mode: int, timeout: float = 0.0) -> bool:
        return bool(self._lib.motor_set_target(self._handle, node_id, value, mode, timeout))

    def set_target_velocity(self, node_id: int, velocity: float, timeout: float = 0.0) -> bool:
        """Send velocity-mode target.

        Args:
            node_id: Motor node ID, 1–15.
            velocity: Target angular velocity [rad/s].
            timeout: Command timeout [s]; 0 disables timeout monitoring.

        Returns:
            True if the command was sent successfully.
        """
        return self._set_target(node_id, velocity, 0, timeout)

    def set_target_position(self, node_id: int, position: float, timeout: float = 0.0) -> bool:
        """Send position-mode target.

        Args:
            node_id: Motor node ID, 1–15.
            position: Target joint position [rad].
            timeout: Command timeout [s]; 0 disables timeout monitoring.

        Returns:
            True if the command was sent successfully.
        """
        return self._set_target(node_id, position, 1, timeout)

    def set_target_current(self, node_id: int, current: float, timeout: float = 0.0) -> bool:
        """Send current / torque-mode target.

        Args:
            node_id: Motor node ID, 1–15.
            current: Target phase current [A].
            timeout: Command timeout [s]; 0 disables timeout monitoring.

        Returns:
            True if the command was sent successfully.
        """
        return self._set_target(node_id, current, 2, timeout)

    def set_enable(self, node_id: int, enable: bool = True, has_brake: bool = False, brake_on: bool = False) -> bool:
        """Enable or disable driver output.

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
        """Reset or enter/save calibration.

        Args:
            node_id: Motor node ID, 1–15.
            mode: 0 = normal reset; 1 = enter calibration; 2 = save calibration.
            timeout: Blocking ACK wait [s]; if > 0, waits for response.

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
        """Clear all fault and alarm states.

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
        """Set mechanical zero offset.

        Args:
            node_id: Motor node ID, 1–15.
            zero_offset: Offset [rad]; 0 marks the current position as zero.
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
        """Set motion profile acceleration and deceleration.

        Args:
            node_id: Motor node ID, 1–15.
            acceleration: Profile acceleration [rad/s²].
            deceleration: Profile deceleration [rad/s²].

        Returns:
            True if the command was sent successfully.
        """
        return bool(
            self._lib.motor_set_profile_acc_dec(self._handle, node_id, acceleration, deceleration)
        )

    def set_profile_vel(self, node_id: int, velocity: float) -> bool:
        """Set motion profile velocity limit.

        Args:
            node_id: Motor node ID, 1–15.
            velocity: Profile velocity cap [rad/s].

        Returns:
            True if the command was sent successfully.
        """
        return bool(self._lib.motor_set_profile_vel(self._handle, node_id, velocity))

    def set_current_limit(self, node_id: int, current: float) -> bool:
        """Set phase current limit.

        Args:
            node_id: Motor node ID, 1–15.
            current: Current limit [A].

        Returns:
            True if the command was sent successfully.
        """
        return bool(self._lib.motor_set_current_limit(self._handle, node_id, current))

    def set_mit_control(
        self, node_id: int, p_des: float, v_des: float, kp: float, kd: float, t_ff: float, t_am: float = 16.0
    ) -> bool:
        """MIT impedance control (frame without CRC).

        Values are saturated by the protocol library before transmission.

        Args:
            node_id: Motor node ID, 1–15.
            p_des: Desired position [rad], clamped to [-12.5, 12.5].
            v_des: Desired velocity [rad/s], clamped to [-45, 45].
            kp: Position gain, clamped to [0, 500].
            kd: Derivative gain, clamped to [-5, 5].
            t_ff: Feedforward torque [N·m], mapped to [-t_am, +t_am].
            t_am: Torque mapping half-range [N·m]; default 16; <= 0 uses 16 internally.

        Returns:
            True if sent successfully. Use ``set_mit_control_crc`` if firmware needs CRC.
        """
        return bool(
            self._lib.motor_set_mit_control(
                self._handle, node_id, p_des, v_des, kp, kd, t_ff, float(t_am)
            )
        )

    def set_mit_control_crc(
        self, node_id: int, p_des: float, v_des: float, kp: float, kd: float, t_ff: float, t_am: float = 8.0
    ) -> bool:
        """MIT impedance control (frame with CRC field).

        Args:
            node_id: Motor node ID, 1–15.
            p_des: Desired position [rad], clamped to [-12.5, 12.5].
            v_des: Desired velocity [rad/s], clamped to [-45, 45].
            kp: Position gain, clamped to [0, 500].
            kd: Derivative gain, clamped to [-5, 5].
            t_ff: Feedforward torque [N·m], mapped to [-t_am, +t_am].
            t_am: Torque mapping half-range [N·m]; default 8; <= 0 uses 8 internally.

        Returns:
            True if the command was sent successfully.
        """
        return bool(
            self._lib.motor_set_mit_control_crc(
                self._handle, node_id, p_des, v_des, kp, kd, t_ff, float(t_am)
            )
        )

    def set_collision_threshold(self, node_id: int, current_threshold: float, time_threshold: float) -> bool:
        """Configure collision protection.

        Args:
            node_id: Motor node ID, 1–15.
            current_threshold: Trip current [A].
            time_threshold: Trip duration [s].

        Returns:
            True if the command was sent successfully.
        """
        return bool(
            self._lib.motor_set_collision_threshold(
                self._handle, node_id, current_threshold, time_threshold
            )
        )

    def get_param(self, node_id: int, param: MotorParam, timeout: float = 1.0) -> Optional[float]:
        """Read a motor parameter (blocking).

        Args:
            node_id: Motor node ID, 1–15.
            param: Parameter ID; units see :class:`MotorParam`.
            timeout: Response wait [s].

        Returns:
            Parameter value, or None on timeout / failure.
        """
        idx1, idx2 = param.value
        out = c_float()
        if not self._lib.motor_get_param(self._handle, node_id, idx1, idx2, float(timeout), byref(out)):
            return None
        return float(out.value)

    def set_param(self, node_id: int, param: MotorParam, value: float, timeout: float = 1.0) -> bool:
        """Write a motor parameter (blocking).

        Args:
            node_id: Motor node ID, 1–15.
            param: Parameter ID; units see :class:`MotorParam`.
            value: Value to write.
            timeout: Response wait [s].

        Returns:
            True on success.
        """
        idx1, idx2 = param.value
        return bool(
            self._lib.motor_set_param(self._handle, node_id, idx1, idx2, value, float(timeout))
        )

    def get_version(self, node_id: int, timeout: float = 1.0) -> Optional[VersionInfo]:
        """Query firmware version strings (blocking).

        Args:
            node_id: Motor node ID, 1–15.
            timeout: Response wait [s].

        Returns:
            :class:`VersionInfo` with software/hardware/motor strings, or None on failure.
        """
        out = _MotorVersionInfo()
        if not self._lib.motor_get_version(self._handle, node_id, float(timeout), byref(out)):
            return None
        return version_from_ctypes(out)


__all__ = [
    "ArmDriver",
    "DriverStatus",
    "HighSpeedFeedback",
    "LowSpeedFeedback",
    "VersionInfo",
]
