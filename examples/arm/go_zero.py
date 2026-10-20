#!/usr/bin/env python3
"""Low-level :class:`~agx_motor_sdk.motor.ArmDriver` — enable and go to zero [rad].

``set_target_position`` here is **motor space** (no YAML polarity / limits).

Usage::

    python3 examples/arm/go_zero.py
    python3 examples/arm/go_zero.py can0 6
"""

import sys

from _demo_common import run_arm_demo
from agx_motor_sdk import ArmDriver


def main() -> None:
    channel = sys.argv[1] if len(sys.argv) > 1 else "can0"
    node_id = int(sys.argv[2]) if len(sys.argv) > 2 else 1

    def body(motor: ArmDriver, nid: int) -> None:
        print(motor.get_version(nid, timeout=0.5))
        print(motor.set_enable(nid, True))
        print(motor.set_profile_vel(nid, 3.0))
        print(motor.set_target_position(nid, 0.0))

    run_arm_demo(body, channel=channel, node_id=node_id)


if __name__ == "__main__":
    main()
