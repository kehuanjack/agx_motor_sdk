"""CAN transport — port factories and backends.

Public API
----------

    :func:`create_can_port` — main entry; returns cpp or python-can port
    :func:`create_can_port_config` — build config dict for the factory
    :class:`PythonCanPort` — pure ``python-can`` backend (fallback)
    :func:`create_python_can_config` — config dict for python-can backend

Environment
-----------
``AGX_MOTOR_SDK_CAN_BACKEND`` — ``"cpp"`` (default) or ``"python-can"``.

Example::

    from agx_motor_sdk.can import create_can_port

    bus = create_can_port(channel="can0")
    bus.set_callback(lambda msg: print(msg.arbitration_id))
"""

from .backend import CanPort, create_can_port, create_can_port_config
from .python_can import PythonCanPort, create_python_can_config

__all__ = [
    "CanPort",
    "PythonCanPort",
    "create_can_port",
    "create_can_port_config",
    "create_python_can_config",
]
