#!/usr/bin/env python3
"""Prove each joint NAME maps to the right servo ID, by watching what moves.

An ID scan cannot catch a swapped pair -- if elbow_flex and wrist_roll were
flashed onto each other's IDs, all 7 ids still show up present. Only physically
moving a known joint settles it.

You move the joints in the prompted order; this logs which ID actually moved and
compares that sequence against the expected one.

Read-only: PING and READ only, no writes, torque untouched.

Usage:
    python3 verify_mapping.py [--port /dev/ttyACM0] [--threshold 60]
"""

import argparse
import importlib.util
import pathlib
import sys
import time

import serial

# Reuse the protocol helpers from the checker rather than duplicating them.
_spec = importlib.util.spec_from_file_location(
    "chk", pathlib.Path(__file__).with_name("check_leader_standalone.py")
)
_chk = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_chk)

EXPECTED = _chk.EXPECTED
ADDR_PRESENT_POSITION = _chk.ADDR_PRESENT_POSITION
read_u16 = _chk.read_u16

MOVE_ORDER = [name for name, _ in EXPECTED]


RESOLUTION = 4096


def read_all(ser):
    out = {}
    for name, mid in EXPECTED:
        out[name] = read_u16(ser, mid, ADDR_PRESENT_POSITION)
    return out


def delta(a, b):
    """Shortest circular distance, in ticks.

    A joint parked on the 0/4095 seam flips between e.g. 4067 and 34; treated
    linearly that is a bogus 4033-tick 'move'. Circularly it is 63.
    """
    d = (a - b) % RESOLUTION
    return min(d, RESOLUTION - d)


def noise_floor(ser, seconds=4.0):
    """Per-joint quiescent spread, used to set per-joint thresholds."""
    print(f"Calibrating noise floor ({seconds:.0f}s) -- do NOT touch the arm ... ", end="", flush=True)
    samples = {n: [] for n, _ in EXPECTED}
    t0 = time.time()
    while time.time() - t0 < seconds:
        cur = read_all(ser)
        for n, v in cur.items():
            if v is not None:
                samples[n].append(v)
        time.sleep(0.05)
    floor = {}
    for n, h in samples.items():
        if len(h) < 2:
            floor[n] = 0
            continue
        ref = h[-1]
        floor[n] = max(delta(v, ref) for v in h)
    print("done")
    for n, _ in EXPECTED:
        print(f"    {n:<14} noise {floor[n]:>5} ticks")
    return floor


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument(
        "--threshold", type=int, default=60, help="ticks of motion to count as moved"
    )
    args = ap.parse_args()

    with serial.Serial(args.port, 1_000_000, timeout=0.05, write_timeout=0.2) as ser:
        time.sleep(0.2)
        base = read_all(ser)
        missing = [n for n, v in base.items() if v is None]
        if missing:
            print(f"cannot read: {missing} -- fix comms first")
            return 1

        print("Move ONE joint at a time, in this order. Return it roughly back after.")
        print("Press Ctrl-C when done.\n")
        results = []
        for expected in MOVE_ORDER:
            exp_id = dict(EXPECTED)[expected]
            print(f"  -> move '{expected}' (expect id {exp_id}) ... ", end="", flush=True)
            moved = None
            while moved is None:
                cur = read_all(ser)
                for name, mid in EXPECTED:
                    if cur[name] is None or base[name] is None:
                        continue
                    if abs(cur[name] - base[name]) >= args.threshold:
                        moved = (name, mid)
                        break
                time.sleep(0.05)
            got_name, got_id = moved
            ok = got_name == expected
            print(f"id {got_id} ({got_name}) {'OK' if ok else '<-- MISMATCH'}")
            results.append((expected, exp_id, got_name, got_id, ok))
            # Re-baseline so the next joint is measured from rest.
            time.sleep(1.2)
            base = read_all(ser)

        print("\n== Mapping verdict ==")
        bad = [r for r in results if not r[4]]
        for exp_n, exp_i, got_n, got_i, ok in results:
            print(
                f"  {exp_n:<14} expected id {exp_i} -> moved id {got_i} "
                f"({got_n}) {'OK' if ok else 'MISMATCH'}"
            )
        if bad:
            print(f"\n{len(bad)} joint(s) mismapped -- re-flash those with set_one_id.py")
            return 1
        print("\nAll 7 joints map correctly. Leader arm assembly verified.")
        return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nstopped.")
