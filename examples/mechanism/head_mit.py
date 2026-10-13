#!/usr/bin/env python3
"""Mechanism layer: YAML module → connect → configure → enable → joint-space control.

Uses :class:`~agx_motor_sdk.mechanism.ArmJointGroup` with ``set_target_position``
(joint space: YAML limits + polarity applied inside the motor layer).

Usage (after ``sudo ./scripts/can_up_1m.sh``)::

    python3 examples/mechanism/head_from_yaml.py
    python3 examples/mechanism/head_from_yaml.py can0
    python3 examples/mechanism/head_from_yaml.py can0 0.0
"""

import sys
from pathlib import Path

from agx_motor_sdk.mechanism import ArmJointGroup

_YAML = Path(__file__).resolve().parent / "robot_motors.yaml"
_MODULE = "head"


def main() -> None:
    can_channel = sys.argv[1] if len(sys.argv) > 1 else None
    position_rad = float(sys.argv[2]) if len(sys.argv) > 2 else None

    if can_channel:
        group = ArmJointGroup.from_yaml(str(_YAML), _MODULE, can_channel)
    else:
        group = ArmJointGroup.from_yaml(str(_YAML), _MODULE)

    if not group.connect():
        raise SystemExit("connect failed — check CAN interface and YAML can_channel")

    group.configure()
    group.enable()

    for name in group.joint_names:
        idx = group.index_of(name)
        if idx == ArmJointGroup.INVALID_JOINT_INDEX:
            continue
        motor = group.motor(idx)
        if not motor.is_valid:
            print(f"{name}: invalid motor handle")
            continue

        if position_rad is not None:
            ok = group.set_mit(idx, position_rad, 0.0, 0.0)
            print(f"{name}: set_target_position({position_rad}) -> {ok}")

        state = motor.get_state()
        if state is None:
            print(f"{name} node_id={motor.node_id} state=None (no feedback yet)")
        else:
            print(
                f"{name} node_id={motor.node_id} "
                f"pos={state.position_rad:.4f} rad "
                f"vel={state.velocity_rad_s:.4f} rad/s "
                f"torque={state.torque_nm:.4f} N·m"
            )


if __name__ == "__main__":
    main()
