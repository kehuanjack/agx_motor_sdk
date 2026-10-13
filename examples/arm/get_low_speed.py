#!/usr/bin/env python3
"""Low-level :class:`~agx_motor_sdk.motor.ArmDriver` — poll low-speed feedback.

Usage::

    python3 examples/arm/get_low_speed.py
    python3 examples/arm/get_low_speed.py can0 6
"""

import sys
import time

from _demo_common import run_arm_demo
from agx_motor_sdk import ArmDriver


def main() -> None:
    channel = sys.argv[1] if len(sys.argv) > 1 else "can0"
    node_id = int(sys.argv[2]) if len(sys.argv) > 2 else 1

    def body(motor: ArmDriver, nid: int) -> None:
        print(motor.get_version(nid, timeout=0.5))
        while True:
            ls = motor.get_low_speed_feedback(nid)
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
            time.sleep(0.05)

    run_arm_demo(body, channel=channel, node_id=node_id)


if __name__ == "__main__":
    main()
