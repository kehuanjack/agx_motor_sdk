"""C++ socketcan backend (default)."""
import time
from typing import Any, Callable, Dict, Optional

from can.message import Message

try:
    from agx_motor_sdk import _native
except ImportError as exc:
    _native = None
    _native_import_error = exc
else:
    _native_import_error = None


class SocketCanPort:
    """CAN port backed by ``agx_motor_sdk._native`` (C++ socketcan + reader thread)."""

    def __init__(self, config: Dict[str, Any]) -> None:
        """Open or prepare a socketcan interface.

        Args:
            config: Dict from :func:`~agx_motor_sdk.can.backend.create_can_port_config`.
                Uses ``channel``, ``timeout``, ``auto_connect``, and ``local_loopback``.

        Raises:
            RuntimeError: Native extension unavailable or ``connect()`` failed.
        """
        if _native is None:
            raise RuntimeError(
                "agx_motor_sdk C++ CAN backend is unavailable: {0}. "
                "Install with pip/colcon build, or set AGX_MOTOR_SDK_CAN_BACKEND=python-can."
                .format(_native_import_error)
            )
        self._config = dict(config)
        self._channel = self._config.get("channel", "can0")
        self._timeout = float(self._config.get("timeout", 1.0))
        self._cb: Optional[Callable[[Message], None]] = None
        self._port = _native.SocketCanPort(
            self._channel,
            bool(self._config.get("local_loopback", False)),
        )
        if self._config.get("auto_connect", True):
            self.connect()

    def connect(self, **kwargs) -> bool:
        """Open the CAN socket and start the background reader thread.

        Returns:
            True on success.

        Raises:
            RuntimeError: Interface missing or not UP.
        """
        if not self._port.open():
            raise RuntimeError(
                "failed to open CAN port: {0}. "
                "Ensure the interface exists and is UP (e.g. scripts/can_up_1m.sh), "
                "or set AGX_MOTOR_SDK_CAN_BACKEND=python-can.".format(self._channel)
            )
        self._port.set_callback(self._on_native_frame)
        return True

    def close(self) -> None:
        """Stop the reader thread and close the socket."""
        try:
            self._port.set_callback(None)
        except Exception:
            pass
        self._port.close()

    def is_connected(self) -> bool:
        """Return whether the socket is open."""
        return self._port.is_open()

    def send(self, message: Message, **kwargs) -> None:
        """Transmit one CAN frame.

        Args:
            message: ``python-can`` Message; uses ``arbitration_id`` and ``data``.
        """
        self._port.send(int(message.arbitration_id), bytes(message.data))

    def recv(self, **kwargs) -> Optional[Message]:
        """No-op poll hook; RX is delivered asynchronously via ``set_callback``.

        With the C++ backend, call this in a loop only to keep application
        structure compatible with the python-can backend. Returns None after a short sleep.
        """
        time.sleep(min(self._timeout, 0.05))
        return None

    def set_callback(self, cb: Callable[[Message], None]) -> None:
        """Register a handler for received frames (invoked from the reader thread).

        Args:
            cb: Called with a ``python-can`` Message per received frame.
        """
        self._cb = cb

    def clear_callback(self) -> None:
        """Remove the receive handler."""
        self._cb = None

    def _on_native_frame(self, can_id: int, payload: bytes) -> None:
        if self._cb is None:
            return
        msg = Message(
            arbitration_id=int(can_id),
            data=payload,
            is_extended_id=False,
            timestamp=time.time(),
        )
        self._cb(msg)
