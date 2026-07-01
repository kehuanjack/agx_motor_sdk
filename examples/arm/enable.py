import time
import threading
from agx_motor_sdk import ArmDriver, create_can_port, create_can_port

bus = create_can_port(channel="can0", interface="socketcan", auto_connect=True)
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
    node_id = 1
    ver = motor.get_version(node_id, timeout=0.5)
    print(ver)

    print(motor.set_enable(node_id, True))
finally:
    stop.set()
    motor.close()
    bus.close()