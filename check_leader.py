#!/usr/bin/env python
"""Assembly check for the AM-ARM200 leader: are all 7 Feetech servos present
and mapped to the right joint?

Read-only. Pings the bus, then reads raw Present_Position per joint. Never
writes a register, never enables torque, never touches calibration.

Usage:
    python check_leader.py [--port /dev/ttyACM0] [--watch]

--watch re-reads positions ~5x/s so you can wiggle one joint at a time and
confirm the row that moves is the row you expect.
"""

import argparse
import sys
import time

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

# Mirrors _ARM_PROFILES["am-leader-6dof"] in
# src/lerobot/teleoperators/so_leader/so_leader.py
EXPECTED = [
    ("shoulder_pan", 1),
    ("shoulder_lift", 2),
    ("elbow_flex", 3),
    ("wrist_flex", 4),
    ("wrist_yaw", 5),
    ("wrist_roll", 6),
    ("gripper", 7),
]


def scan(port):
    print(f"== Scanning {port} across all baudrates ==")
    found = FeetechMotorsBus.scan_port(port)
    if not found:
        print("\nFAIL: nothing answered on the bus at any baudrate.")
        print("  Check: 12V servo power on, data chain seated, correct port.")
        return None

    for baud, ids in sorted(found.items()):
        print(f"  baudrate {baud}: ids {sorted(ids)}")

    if len(found) > 1:
        print(
            "\nWARN: servos answered at more than one baudrate. The bus must be "
            "uniform — reflash the odd ones out before teleop."
        )

    # Verdict against the expected profile, per baudrate.
    expected_ids = {mid for _, mid in EXPECTED}
    for baud, ids in sorted(found.items()):
        ids = set(ids)
        missing = sorted(expected_ids - ids)
        extra = sorted(ids - expected_ids)
        print(f"\n== Verdict @ {baud} ==")
        print(f"  found {len(ids)}/7 expected ids")
        if missing:
            names = {mid: n for n, mid in EXPECTED}
            print(
                "  MISSING: "
                + ", ".join(f"id {m} ({names[m]})" for m in missing)
            )
        if extra:
            print(f"  UNEXPECTED ids: {extra} (likely an unprogrammed servo)")
        if not missing and not extra:
            print("  OK: ids 1-7 all present, no strays.")

    return found


def positions(port, watch):
    motors = {
        name: Motor(mid, "sts3215", MotorNormMode.DEGREES)
        for name, mid in EXPECTED
    }
    bus = FeetechMotorsBus(port, motors)
    # handshake=False: don't fail the whole run just because one servo is absent,
    # and don't require a stored calibration file.
    bus.connect(handshake=False)
    try:
        print("\n== Raw Present_Position (ticks, 0-4095) ==")
        if watch:
            print("Wiggle ONE joint at a time. Ctrl-C to stop.\n")
        while True:
            rows = []
            for name, mid in EXPECTED:
                try:
                    # normalize=False -> raw ticks, so no calibration needed.
                    val = bus.read("Present_Position", name, normalize=False)
                    rows.append(f"  id {mid}  {name:<14} {val:>5}")
                except Exception as e:
                    rows.append(f"  id {mid}  {name:<14}   ERR  {type(e).__name__}")
            if watch:
                print("\033[2J\033[H", end="")
                print("Wiggle ONE joint at a time. Ctrl-C to stop.\n")
            print("\n".join(rows))
            if not watch:
                break
            time.sleep(0.2)
    finally:
        bus.disconnect()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--watch", action="store_true")
    args = ap.parse_args()

    found = scan(args.port)
    if found is None:
        return 1
    try:
        positions(args.port, args.watch)
    except KeyboardInterrupt:
        print("\nstopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
