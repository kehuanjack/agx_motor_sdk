#!/usr/bin/env python3
"""Low-level :class:`~agx_motor_sdk.motor.ArmDriver` — poll high-speed feedback.

Returns **motor space** position [rad], velocity [rad/s], current [A].
Joint-space feedback with polarity: ``ArmJointGroup.motor(i).get_state()``.

Usage::

    python3 examples/arm/get_high_speed.py
    python3 examples/arm/get_high_speed.py can0 6
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
            hs = motor.get_high_speed_feedback(nid)
            if hs is not None:
                print(hs.position, hs.velocity, hs.current, hs.timestamp)
            time.sleep(0.005)

    run_arm_demo(body, channel=channel, node_id=node_id)


if __name__ == "__main__":
    main()
