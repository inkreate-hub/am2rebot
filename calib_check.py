#!/usr/bin/env python
"""Is the leader-arm calibration still valid after a reconnect / power cycle?

Calibration lives in TWO places and lerobot only skips re-calibrating when both
agree:

  1. JSON file  ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/<id>.json
  2. Servo EEPROM  Homing_Offset + Min_Position_Limit + Max_Position_Limit

`bus.is_calibrated` reads (2) back off the servos and compares it to (1). If they
match, `connect()` is silent; if not, it prompts and you re-calibrate. This
script performs that same comparison and shows the per-joint diff, so a drift
shows up as a specific register rather than a yes/no.

READ-ONLY: reads registers and the file, writes nothing to either.

Usage:
    python calib_check.py [--id am_leader] [--port /dev/ttyACM0]
"""

import argparse
import json
import sys
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION

PROFILE = [
    ("shoulder_pan", 1),
    ("shoulder_lift", 2),
    ("elbow_flex", 3),
    ("wrist_flex", 4),
    ("wrist_yaw", 5),
    ("wrist_roll", 6),
    ("gripper", 7),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", default="am_leader")
    ap.add_argument("--port", default="/dev/ttyACM0")
    args = ap.parse_args()

    fpath = (
        Path(HF_LEROBOT_CALIBRATION) / "teleoperators" / "so_leader" / f"{args.id}.json"
    )
    print(f"calibration file: {fpath}")
    if not fpath.is_file():
        print("  MISSING -> never calibrated under this id.")
        print("  Nothing to compare. Run lerobot-calibrate first.")
        return 2
    on_disk = json.loads(fpath.read_text())
    print(f"  present, {len(on_disk)} joints\n")

    motors = {n: Motor(i, "sts3215", MotorNormMode.DEGREES) for n, i in PROFILE}
    bus = FeetechMotorsBus(args.port, motors)
    bus._connect(handshake=False)
    try:
        live = bus.read_calibration()
    finally:
        bus.port_handler.closePort()

    hdr = f"{'joint':<15}{'offset f/m':>16}{'min f/m':>14}{'max f/m':>14}   verdict"
    print(hdr)
    print("-" * len(hdr))
    mismatched = []
    for name, _ in PROFILE:
        f = on_disk.get(name)
        m = live.get(name)
        if f is None or m is None:
            print(f"{name:<15}{'-':>16}{'-':>14}{'-':>14}   MISSING")
            mismatched.append(name)
            continue
        off_ok = f["homing_offset"] == m.homing_offset
        min_ok = f["range_min"] == m.range_min
        max_ok = f["range_max"] == m.range_max
        ok = off_ok and min_ok and max_ok
        if not ok:
            mismatched.append(name)
        c_off = "{}/{}".format(f["homing_offset"], m.homing_offset)
        c_min = "{}/{}".format(f["range_min"], m.range_min)
        c_max = "{}/{}".format(f["range_max"], m.range_max)
        print(
            f"{name:<15}{c_off:>16}{c_min:>14}{c_max:>14}"
            f"   {'OK' if ok else 'DIFFERS'}"
        )

    print()
    if mismatched:
        print(f"NOT REUSABLE: {len(mismatched)} joint(s) differ -> {mismatched}")
        print("lerobot will prompt to re-calibrate on connect.")
        return 1
    print("REUSABLE: file and servo EEPROM agree on all 7 joints.")
    print("connect() will skip calibration silently.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
