import time
import threading
from agx_motor_sdk import ChassisDriver, create_can_port

bus = create_can_port(channel="can0", interface="socketcan", auto_connect=True)
motor = ChassisDriver(bus)

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
    print(motor.set_enable(node_id, True))

    while True:
        ls = motor.get_low_speed_feedback(node_id)
        if ls is not None:
            print(
                ls.bus_voltage_v,
                ls.driver_temp_deg,
                ls.motor_temp_deg,
                ls.bus_current,
                ls.status_raw,
                ls.status,
                ls.timestamp,
            )

        time.sleep(0.005)

finally:
    stop.set()
    motor.close()
    bus.close()
