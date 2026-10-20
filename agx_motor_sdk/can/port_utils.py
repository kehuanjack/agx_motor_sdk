"""Resolve a Python CAN port object for native drivers (handled in C++)."""

from typing import Any


def native_can_port(port: Any) -> Any:
    """Normalize a CAN port before passing to C++ drivers.

    Currently a pass-through; :func:`agx_motor_sdk._native.wrap_can_port` performs
    type detection at pybind bind time (``SocketCanPort``, ``python-can`` bus, etc.).

    Args:
        port: CAN port from :func:`~agx_motor_sdk.can.create_can_port` or compatible.

    Returns:
        The same object (future versions may unwrap or validate here).
    """
    return port
