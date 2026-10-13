#!/usr/bin/env python3
"""Low-level :class:`~agx_motor_sdk.motor.ArmDriver` — read version and enable.

Uses **motor space** (raw protocol units). For YAML joint modules with polarity
and limits, see ``examples/mechanism/``.

Usage (after ``sudo ./scripts/can_up_1m.sh``)::

    python3 examples/arm/enable.py
    python3 examples/arm/enable.py can0 6
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

    run_arm_demo(body, channel=channel, node_id=node_id)


if __name__ == "__main__":
    main()
