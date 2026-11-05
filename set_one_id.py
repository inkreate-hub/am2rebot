#!/usr/bin/env python
"""Flash ONE named joint's ID on the AM-ARM200 leader, in any order.

Avoids the positional footgun in `lerobot-setup-motors`, whose prompt sequence
always starts at `gripper` — restarting that loop per servo mis-assigns IDs.
Here you name the joint explicitly, so order never matters.

Connect ONLY the target servo to the driver board before running. The underlying
setup_motor() takes the first responder on the bus and its only guard is a model
-number check; since every servo here is an STS3215 it cannot tell two apart and
would silently flash the wrong one.

Usage:
    python set_one_id.py gripper
    python set_one_id.py --port /dev/ttyACM0 wrist_roll
    python set_one_id.py --list
"""

import argparse
import sys

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

# Mirrors _ARM_PROFILES["am-leader-6dof"] in
# src/lerobot/teleoperators/so_leader/so_leader.py
PROFILE = [
    ("shoulder_pan", 1),
    ("shoulder_lift", 2),
    ("elbow_flex", 3),
    ("wrist_flex", 4),
    ("wrist_yaw", 5),
    ("wrist_roll", 6),
    ("gripper", 7),
]
TARGET_ID = dict(PROFILE)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("joint", nargs="?", help="joint name to flash")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--list", action="store_true", help="show joint -> id map")
    args = ap.parse_args()

    if args.list or not args.joint:
        print("joint            target id")
        for name, mid in PROFILE:
            print(f"  {name:<14} {mid}")
        return 0

    if args.joint not in TARGET_ID:
        print(f"unknown joint '{args.joint}'. Use --list.")
        return 1

    target = TARGET_ID[args.joint]
    motors = {
        name: Motor(mid, "sts3215", MotorNormMode.DEGREES) for name, mid in PROFILE
    }
    bus = FeetechMotorsBus(args.port, motors)

    print(f"Target: '{args.joint}' -> id {target} @ 1000000 baud")
    ans = input(
        f"Is '{args.joint}' the ONLY servo connected to {args.port}? [yes/N] "
    )
    if ans.strip().lower() not in ("y", "yes"):
        print("aborted — disconnect the other servos first.")
        return 1

    # handshake=False: the bus does not yet match the profile, that's the point.
    bus._connect(handshake=False)
    try:
        # Scans every baudrate, finds the current id whatever it is, disables
        # torque, writes the new id, then programs the default baudrate.
        bus.setup_motor(args.joint)
        print(f"OK: '{args.joint}' id set to {bus.motors[args.joint].id}")
    except RuntimeError as e:
        print(f"\nFAILED: {e}")
        print(
            "  'not found'      -> servo unpowered or uncabled (check the 12V rail)\n"
            "  'model number'   -> more than one servo on the bus, or a non-STS3215"
        )
        return 1
    finally:
        # Close the port directly rather than via bus.disconnect(): that walks
        # every motor in the profile to disable torque, and with only one servo
        # on the bus it raises on the six that are absent -- after the ID write
        # has already succeeded.
        bus.port_handler.closePort()
    return 0


if __name__ == "__main__":
    sys.exit(main())
