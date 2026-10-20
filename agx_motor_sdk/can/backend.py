"""CAN port backend selection for agx_motor_sdk."""
import os
import warnings
from typing import Any, Dict, Optional


def create_can_port_config(
    *,
    channel: str = "can0",
    interface: str = "socketcan",
    bitrate: int = 1_000_000,
    enable_check_can: bool = True,
    auto_connect: bool = True,
    timeout: float = 1.0,
    receive_own_messages: bool = False,
    local_loopback: bool = False,
    backend: Optional[str] = None,
    fallback_python_can: bool = True,
) -> Dict[str, Any]:
    """Build a configuration dict for :func:`create_can_port`.

    Args:
        channel: CAN interface name, e.g. ``"can0"``.
        interface: ``python-can`` bustype when using the python-can backend.
        bitrate: Nominal bitrate [bit/s]; used for interface checks on python-can.
        enable_check_can: If True, verify the interface exists before connect.
        auto_connect: If True, call ``connect()`` when the port object is created.
        timeout: Default recv timeout [s].
        receive_own_messages: Passed to ``python-can`` when applicable.
        local_loopback: Enable socket loopback (C++ and python-can backends).
        backend: ``"cpp"`` or ``"python-can"``; defaults to env
            ``AGX_MOTOR_SDK_CAN_BACKEND`` or ``"cpp"``.
        fallback_python_can: If True, try python-can when the C++ backend fails.

    Returns:
        Config dict consumed by :func:`create_can_port` or port constructors.
    """
    return {
        "channel": channel,
        "interface": interface,
        "bitrate": bitrate,
        "enable_check_can": enable_check_can,
        "auto_connect": auto_connect,
        "timeout": timeout,
        "receive_own_messages": receive_own_messages,
        "local_loopback": local_loopback,
        "backend": backend,
        "fallback_python_can": fallback_python_can,
    }


def create_can_port(config: Optional[Dict[str, Any]] = None, **kwargs: Any):
    """Create a CAN port for use with :class:`~agx_motor_sdk.motor.arm_driver.ArmDriver`.

    Default backend is C++ socketcan (``cpp``). Set ``backend="python-can"`` or
    ``AGX_MOTOR_SDK_CAN_BACKEND=python-can`` for the pure-Python backend.

    Args:
        config: Base config dict; merged with ``kwargs``.
        **kwargs: Override any key from :func:`create_can_port_config`.

    Returns:
        A port object with ``send``, ``recv``, ``set_callback``, and ``close``.

    Raises:
        RuntimeError: Both requested backend and fallback failed to initialize.
    """
    cfg = dict(config or {})
    cfg.update(kwargs)
    backend = (
        cfg.pop("backend", None)
        or os.environ.get("AGX_MOTOR_SDK_CAN_BACKEND", "cpp")
    ).lower()

    if backend == "python-can":
        from .python_can import PythonCanPort

        return PythonCanPort(cfg)

    from .socket_can import SocketCanPort

    try:
        return SocketCanPort(cfg)
    except RuntimeError as exc:
        if cfg.get("fallback_python_can", True):
            try:
                from .python_can import PythonCanPort

                warnings.warn(
                    "agx_motor_sdk: C++ CAN backend failed ({0}); "
                    "falling back to python-can.".format(exc),
                    RuntimeWarning,
                    stacklevel=2,
                )
                return PythonCanPort(cfg)
            except Exception:
                pass
        raise exc


# Alias used by driver type hints.
CanPort = object

__all__ = ["CanPort", "create_can_port", "create_can_port_config"]
