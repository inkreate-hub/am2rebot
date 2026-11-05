#!/usr/bin/env python
"""Live timestamped joint-position reader for the AM-ARM200 leader arm.

This is the Subtask 1 deliverable: a plain-Python reader that streams the 6 arm
joints plus the gripper. No ROS 2, no MuJoCo, no follower -- pure serial reads
over the Feetech bus.

Read-only. Torque is never enabled, nothing is written to servo EEPROM, and the
arm stays limp so you can move it by hand.

MODES

  (default)     Human-readable live table, redrawn in place.
  --csv         Machine-readable CSV to stdout. This is the mode Subtask 2
                should consume. Header first, then one row per sample.
  --csv -o FILE Same, written to FILE as well as being counted on stderr.
  --once        Single sample, then exit. Useful for scripting and smoke tests.

UNITS

  Arm joints: degrees. Zero is mid-travel, set by calibration -- NOT a
  mechanical hard stop. Positive directions are documented in SIGNS below and
  in RUNBOOK.md.

  Gripper: normalised 0-100 (0 = closed), not degrees. This is the
  MotorNormMode.RANGE_0_100 convention from the am-leader-6dof profile.

TIMESTAMPS

  Two are emitted per sample, deliberately:
    t_mono  -- time.monotonic(), seconds since an arbitrary origin. Use this for
               intervals and rates; it cannot jump backwards on an NTP step.
    t_unix  -- time.time(), wall clock. Use this to correlate with a video file
               or another process.
  Also dt_ms, the measured bus read time for that sample, which is what actually
  limits achievable rate.

EXAMPLES

    ./lerobot_alohamini/.venv/bin/python read_joints.py
    ./lerobot_alohamini/.venv/bin/python read_joints.py --csv -o run.csv
    ./lerobot_alohamini/.venv/bin/python read_joints.py --once
    ./lerobot_alohamini/.venv/bin/python read_joints.py --fps 50 --seconds 30
"""

import argparse
import csv
import sys
import time

# Joint order is fixed and matches the am-leader-6dof profile: servo ids 1-7 in
# this sequence. Subtask 2 can rely on this ordering.
JOINT_ORDER = [
    "shoulder_pan",   # id 1
    "shoulder_lift",  # id 2
    "elbow_flex",     # id 3
    "wrist_flex",     # id 4
    "wrist_yaw",      # id 5
    "wrist_roll",     # id 6
    "gripper",        # id 7, normalised 0-100
]

# Sign convention: which physical direction makes the reported value increase.
#
# Filled in by `characterize_joints.py --signs`, which asks you to move each
# joint one named way and records what the value actually did. It is NOT
# derivable from the servo tables or the calibration -- it depends on how this
# particular arm was assembled and which way each servo horn was seated.
#
# MEASURED 2026-11-05 on this arm, id=am_leader. Stated as the direction that
# makes the value INCREASE, since that is what downstream code reasons about.
#
# Six of the seven run opposite to the intuitive direction -- only
# shoulder_lift increases when you lift the arm. Do not guess these.
SIGNS = {
    "shoulder_pan": "positive = base rotates CLOCKWISE seen from above",
    "shoulder_lift": "positive = upper arm LIFTS up, away from the table",
    "elbow_flex": "positive = elbow FLEXES closed (forearm folds in)",
    "wrist_flex": "positive = wrist pitches DOWN",
    "wrist_yaw": "positive = wrist yaws CLOCKWISE seen from above",
    "wrist_roll": "positive = gripper rolls COUNTER-CLOCKWISE seen from behind the wrist",
    # This one comes from the profile, not from assembly: MotorNormMode
    # RANGE_0_100 on id 7, and the measured sweep ran 0.1 -> 99.0 opening.
    "gripper": "0 = closed, 100 = open (normalised, not degrees)",
}

# The raw observations, kept so the conclusions above can be audited.
# Each is the motion that was asked for, and what the value did.
SIGNS_RAW = {
    "shoulder_pan": "decreases when base rotates counter-clockwise (from above)",
    "shoulder_lift": "increases when the upper arm lifts away from the table",
    "elbow_flex": "decreases when the forearm extends straight",
    "wrist_flex": "decreases when the wrist pitches up",
    "wrist_yaw": "decreases when the wrist yaws counter-clockwise (from above)",
    "wrist_roll": "decreases when the gripper rolls clockwise (from behind)",
}


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--id", default="am_leader", help="calibration id")
    parser.add_argument("--profile", default="am-leader-6dof")
    parser.add_argument("--fps", type=float, default=30.0, help="target sample rate")
    parser.add_argument("--seconds", type=float, default=0.0, help="0 = until Ctrl-C")
    parser.add_argument("--csv", action="store_true", help="emit CSV instead of a table")
    parser.add_argument("-o", "--out", metavar="FILE", help="also write CSV to FILE")
    parser.add_argument("--once", action="store_true", help="one sample, then exit")
    parser.add_argument("--raw", action="store_true", help="append raw servo ticks")
    parser.add_argument("--signs", action="store_true", help="print sign convention and exit")
    args = parser.parse_args()

    if args.signs:
        print("Joint order (servo id 1-7) and sign convention:\n")
        for i, name in enumerate(JOINT_ORDER, start=1):
            print(f"  {i}. {name:<15} {SIGNS[name]}")
        print("\nArm joints are degrees about a calibrated mid-travel zero.")
        print("The gripper is normalised 0-100, not degrees.")
        if any(v == "unverified" for v in SIGNS.values()):
            print("\nSome signs are unverified. Record them with:")
            print("  python characterize_joints.py --signs")
        else:
            print("\nMeasured 2026-11-05. Raw observations behind these:\n")
            for name, obs in SIGNS_RAW.items():
                print(f"  {name:<15} {obs}")
        return 0

    # Imported here so --signs and --help work without the venv's lerobot.
    from lerobot.teleoperators.so_leader import SOLeader
    from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig

    leader = SOLeader(
        SOLeaderTeleopConfig(port=args.port, id=args.id, arm_profile=args.profile)
    )
    # calibrate=False: the stored calibration is loaded and reused. It must NOT
    # re-run the sweep -- a fresh sweep yields different (not wrong) offsets and
    # would invalidate every number recorded so far.
    leader.connect(calibrate=False)
    if not leader.is_calibrated:
        print(
            "WARNING: arm reports uncalibrated. Values will be raw-ish and the "
            "zero is meaningless. See RUNBOOK.md section 2.",
            file=sys.stderr,
        )

    fields = ["t_mono", "t_unix", "dt_ms", *JOINT_ORDER]
    if args.raw:
        fields += [f"{n}_ticks" for n in JOINT_ORDER]

    writers = []
    out_file = None
    if args.csv:
        writers.append(csv.writer(sys.stdout))
    if args.out:
        out_file = open(args.out, "w", newline="")
        writers.append(csv.writer(out_file))
    for w in writers:
        w.writerow(fields)

    if not args.csv and not args.once:
        print(f"\nReading {args.port} at {args.fps:g} Hz. Ctrl-C to stop.")
        print("Arm is limp - move it by hand. Degrees, except gripper (0-100).\n")

    period = 1.0 / args.fps if args.fps > 0 else 0.0
    deadline = time.perf_counter() + args.seconds if args.seconds > 0 else float("inf")
    n = 0
    t_first = None

    try:
        while time.perf_counter() < deadline:
            t0 = time.perf_counter()
            action = leader.get_action()
            dt_ms = (time.perf_counter() - t0) * 1e3
            t_mono = time.monotonic()
            t_unix = time.time()
            if t_first is None:
                t_first = t_mono

            vals = [action[f"{name}.pos"] for name in JOINT_ORDER]
            row = [f"{t_mono:.6f}", f"{t_unix:.6f}", f"{dt_ms:.2f}"]
            row += [f"{v:.3f}" for v in vals]
            if args.raw:
                ticks = leader.bus.sync_read("Present_Position", normalize=False)
                row += [str(int(ticks[name])) for name in JOINT_ORDER]

            for w in writers:
                w.writerow(row)
            if args.csv:
                sys.stdout.flush()

            n += 1

            if args.once:
                if not args.csv:
                    print(f"{'joint':<15}{'value':>10}   units")
                    print("-" * 45)
                    for name, v in zip(JOINT_ORDER, vals):
                        unit = "0-100" if name == "gripper" else "deg"
                        print(f"{name:<15}{v:>10.3f}   {unit}")
                    print(f"\nt_mono={t_mono:.6f}  t_unix={t_unix:.6f}  read={dt_ms:.2f} ms")
                break

            if not args.csv and sys.stdout.isatty():
                cells = "  ".join(f"{n_[:9]}:{v:8.2f}" for n_, v in zip(JOINT_ORDER, vals))
                print(f"\r{cells}  [{dt_ms:5.1f}ms]", end="", flush=True)

            if period:
                slack = period - (time.perf_counter() - t0)
                if slack > 0:
                    time.sleep(slack)
    except KeyboardInterrupt:
        pass
    finally:
        leader.disconnect()
        if out_file:
            out_file.close()

    if not args.once:
        elapsed = (time.monotonic() - t_first) if t_first else 0.0
        rate = n / elapsed if elapsed > 0 else 0.0
        where = args.out if args.out else "stdout"
        print(
            f"\n\n{n} samples in {elapsed:.1f}s -> {rate:.1f} Hz actual"
            + (f", written to {where}" if args.csv or args.out else ""),
            file=sys.stderr,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
