"""python-can transport backend (optional fallback when C++ socketcan is unavailable)."""
import errno
from platform import system
from typing import Any, Callable, Optional

import can
from can.message import Message

from .can_sys_utils import CanSystemInfoBase, LinuxSocketCanSystemInfo

_SUPPORTED_PLATFORMS = {"Linux", "Windows", "Darwin"}


def create_python_can_config(
    *,
    channel: str = "can0",
    interface: str = "socketcan",
    bitrate: int = 1_000_000,
    enable_check_can: bool = True,
    auto_connect: bool = True,
    timeout: float = 1.0,
    receive_own_messages: bool = False,
    local_loopback: bool = False,
):
    return {
        "channel": channel,
        "interface": interface,
        "bitrate": bitrate,
        "enable_check_can": enable_check_can,
        "auto_connect": auto_connect,
        "timeout": timeout,
        "receive_own_messages": receive_own_messages,
        "local_loopback": local_loopback,
    }


class PythonCanPort:
    """python-can transport backend."""

    def __new__(cls, config: dict, comm_type: str = "can"):
        platform_system = system()
        if platform_system not in _SUPPORTED_PLATFORMS:
            supported_text = ", ".join(sorted(_SUPPORTED_PLATFORMS))
            raise RuntimeError(
                "Unsupported platform: %s. Supported platforms: %s."
                % (platform_system, supported_text)
            )
        return CanCommImpl(config, comm_type)


class CanCommImpl:
    def __init__(self, config: dict, comm_type: str = "can") -> None:
        self.recv_bus = None
        self.send_bus = None
        self.sysinfo: Optional[CanSystemInfoBase] = None
        self.last_error = None
        self._config = config.copy()
        self._type = comm_type
        self._channel = self._config["channel"]
        self._interface = (
            self._config["interface"]
            if "interface" in self._config
            else self._config.get("bustype", "socketcan")
        )
        self._bitrate = self._config.get("bitrate", 1000000)
        self._enable_check_can = self._config.get("enable_check_can", False)
        self._auto_connect = self._config.get("auto_connect", False)
        self._timeout = self._config.get("timeout", 1.0)
        self._receive_own_messages = self._config.get("receive_own_messages", False)
        self._local_loopback = self._config.get("local_loopback", False)
        self._is_connected = False
        self._is_stopped = False
        self._cb: Optional[Callable[[Any], None]] = None
        if self._interface == "socketcan":
            self.sysinfo = LinuxSocketCanSystemInfo
        if self._enable_check_can and self.sysinfo is not None:
            self.check_can()
        if self._auto_connect:
            self.connect()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def set_callback(self, cb: Callable[[Any], None]) -> None:
        self._cb = cb

    def clear_callback(self) -> None:
        self._cb = None

    def _trigger_callback(self, data: Any) -> None:
        if self._cb is not None:
            self._cb(data)

    def _classify_can_error(self, exc: Exception):
        def _map_known_errno(eno):
            if eno in (errno.ENETDOWN, errno.ENOBUFS):
                return eno
            return None

        candidates = [exc, getattr(exc, "__cause__", None), getattr(exc, "__context__", None)]
        for err in candidates:
            if err is None:
                continue
            if isinstance(err, can.CanOperationError):
                mapped = _map_known_errno(getattr(err, "error_code", None))
                if mapped is not None:
                    return mapped
        for err in candidates:
            if err is None:
                continue
            if isinstance(err, OSError):
                mapped = _map_known_errno(getattr(err, "errno", None))
                if mapped is not None:
                    return mapped
        for err in candidates:
            if err is None:
                continue
            err_text = str(err).lower()
            if "no buffer space available" in err_text or "transmit buffer full" in err_text:
                return errno.ENOBUFS
            if "network is down" in err_text:
                return errno.ENETDOWN
        return None

    def connect(self, **kwargs):
        if self.recv_bus is not None and self.send_bus is not None:
            return True

        common_kwargs = dict(
            channel=self._channel,
            interface=self._interface,
            bitrate=self._bitrate,
            receive_own_messages=self._receive_own_messages,
            local_loopback=self._local_loopback,
        )
        try:
            self.recv_bus = can.interface.Bus(**common_kwargs)
            self.send_bus = self.recv_bus
            self._is_connected = True
            self._is_stopped = False
            self.last_error = None
            return True
        except Exception as exc:
            self.last_error = exc
            self.close()
            raise can.CanInitializationError(
                "Failed to open CAN bus (interface='%s', channel='%s', bitrate=%s)."
                % (self._interface, self._channel, self._bitrate)
            ) from exc

    def close(self):
        try:
            if self.recv_bus is not None:
                self.recv_bus.shutdown()
        except Exception:
            pass
        try:
            if self.send_bus is not None and self.send_bus is not self.recv_bus:
                self.send_bus.shutdown()
        except Exception:
            pass
        self.recv_bus = None
        self.send_bus = None
        self._is_connected = False
        self._is_stopped = True

    def is_connected(self) -> bool:
        return self._is_connected

    def send(self, msg: Message, timeout=None):
        if self.send_bus is None:
            self.close()
            raise RuntimeError("CAN bus is not connected.")
        try:
            self.send_bus.send(msg, timeout)
            self.last_error = None
        except Exception as exc:
            self.last_error = exc
            err_kind = self._classify_can_error(exc)
            if err_kind in (errno.ENOBUFS, errno.ENETDOWN):
                return
            self.close()
            raise self.last_error

    def recv(self):
        if self.recv_bus is None:
            self.close()
            raise RuntimeError("CAN bus is not connected.")
        try:
            msg = self.recv_bus.recv(self._timeout)
            if msg is not None and not msg.is_error_frame:
                self.last_error = None
                self._trigger_callback(msg)
                return msg
        except Exception as exc:
            self.last_error = exc
            err_kind = self._classify_can_error(exc)
            if err_kind in (errno.ENOBUFS, errno.ENETDOWN):
                return None
            self.close()
            raise self.last_error
        return None

    def check_can(self):
        self.last_error = None
        if self.sysinfo is None:
            return
        if not self.sysinfo.is_exists(self._channel):
            self.last_error = ValueError("Device '%s' does not exist." % self._channel)
            raise self.last_error
        if not self.sysinfo.is_up(self._channel):
            print("[WARN] Device is DOWN.")
        actual_bitrate = self.sysinfo.get_bitrate(self._channel)
        if (
            self._bitrate is not None
            and actual_bitrate is not None
            and actual_bitrate != self._bitrate
        ):
            print(
                "[WARN] CAN port %s bitrate is %s bps, expected %s bps."
                % (self._channel, actual_bitrate, self._bitrate)
            )
