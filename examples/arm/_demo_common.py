"""Shared CAN recv loop for low-level :class:`~agx_motor_sdk.motor.ArmDriver` examples."""

import threading
import time
from typing import Callable

from agx_motor_sdk import ArmDriver, create_can_port


def run_arm_demo(
    body: Callable[[ArmDriver, int], None],
    *,
    channel: str = "can0",
    node_id: int = 1,
) -> None:
    """Open CAN, start RX thread, run ``body(driver, node_id)``, then close."""
    bus = create_can_port(channel=channel, interface="socketcan", auto_connect=True)
    motor = ArmDriver(bus)
    stop = threading.Event()

    def recv_loop() -> None:
        while not stop.is_set():
            try:
                bus.recv()
            except Exception:
                if stop.is_set():
                    break

    thread = threading.Thread(target=recv_loop, name="can-recv", daemon=True)
    thread.start()
    time.sleep(0.005)

    try:
        body(motor, node_id)
    finally:
        stop.set()
        motor.close()
        bus.close()
