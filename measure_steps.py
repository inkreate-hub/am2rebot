#!/usr/bin/env python
"""Driven step-response test: command exact 1/2/5 deg moves and measure them.

Covers the acceptance criterion "small 1, 2 and 5 deg joint movements are
detectable" properly. Hand-moving a joint by 1 deg is not realistic, and it
conflates two different questions: whether the SENSOR can resolve 1 deg, and
whether a HUMAN can produce 1 deg. Only the first is being asked about.

So this drives the joint instead. The STS3215s on the leader are servos, not
bare encoders -- we command a known step, read back what happened, and repeat
it N times to get repeatability and hysteresis as real numbers.

WHAT IT MEASURES

  command error    commanded step vs achieved step, per step size
  repeatability    stddev of the achieved position over N repeats
  backlash         gap between approaching the same commanded position from
                   below and from above. Mechanical slop, normally much larger
                   than the 0.0879 deg encoder quantisation.

THIS ONE WRITES TO THE SERVOS. Unlike every other script here it enables
torque. It writes Goal_Position and Torque_Enable, which are RAM registers --
NOT EEPROM -- so nothing persists past a power cycle and no calibration is
touched.

SAFETY

  - One joint at a time. Required argument, no default, no "all" mode.
  - Goal_Position is set to the CURRENT position before torque is enabled.
    Skipping that makes the servo snap to whatever stale goal is in its
    register, which is the one way this could move violently.
  - Steps are refused if they would leave the calibrated travel range.
  - Torque is disabled in a finally block, including on Ctrl-C.
  - Total excursion is capped by --max-excursion.

Start with a joint that carries no weight -- wrist_roll or wrist_yaw -- and
keep a hand near the power before the first run.

    ./lerobot_alohamini/.venv/bin/python measure_steps.py --joint wrist_yaw
    ./lerobot_alohamini/.venv/bin/python measure_steps.py --joint elbow_flex --reps 10
"""

import argparse
import statistics
import sys
import time

from read_joints import JOINT_ORDER

DEG_PER_TICK = 360.0 / 4096

# Below this the servo talks but cannot drive. STS3215 is 7.4 V nominal; 5 V is
# the adapter's USB rail with no external supply attached.
MIN_DRIVE_VOLTS = 6.0

# Measured travel per joint, from find_leader_ranges.py. Used to refuse a step
# that would drive the joint out of its calibrated range.
TRAVEL = {
    "shoulder_pan": (-118.1, 119.7),
    "shoulder_lift": (-102.6, 103.6),
    "elbow_flex": (-92.3, 92.9),
    "wrist_flex": (-92.4, 92.5),
    "wrist_yaw": (-78.5, 80.0),
    "wrist_roll": (-170.1, 172.3),
}


def settle(leader, name: str, timeout: float = 1.5, still_for: float = 0.25) -> float:
    """Wait until the joint stops moving, then return an averaged position.

    Polls until the reading has not changed by more than one tick for
    `still_for` seconds, so the measurement is of a settled pose rather than of
    the servo still travelling.
    """
    t0 = time.perf_counter()
    last = leader.get_action()[f"{name}.pos"]
    stable_since = t0
    while time.perf_counter() - t0 < timeout:
        time.sleep(0.02)
        now = leader.get_action()[f"{name}.pos"]
        if abs(now - last) > DEG_PER_TICK:
            stable_since = time.perf_counter()
        last = now
        if time.perf_counter() - stable_since >= still_for:
            break
    # Average a short burst to kill any residual jitter.
    vals = []
    for _ in range(20):
        vals.append(leader.get_action()[f"{name}.pos"])
        time.sleep(0.005)
    return statistics.fmean(vals)


def goto(leader, name: str, target: float, settle_first: bool = True) -> float:
    leader.bus.write("Goal_Position", name, target)
    return settle(leader, name) if settle_first else 0.0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--joint", required=True, choices=[n for n in JOINT_ORDER if n != "gripper"],
                        help="one joint only; there is deliberately no 'all' mode")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--id", default="am_leader")
    parser.add_argument("--profile", default="am-leader-6dof")
    parser.add_argument("--steps", type=float, nargs="+", default=[1.0, 2.0, 5.0])
    parser.add_argument("--reps", type=int, default=5)
    parser.add_argument("--max-excursion", type=float, default=15.0,
                        help="refuse to command more than this far from the start pose")
    parser.add_argument("-o", "--out", metavar="FILE", help="append a markdown report")
    parser.add_argument("--yes", action="store_true", help="skip the torque confirmation")
    args = parser.parse_args()

    name = args.joint

    from lerobot.teleoperators.so_leader import SOLeader
    from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig

    leader = SOLeader(
        SOLeaderTeleopConfig(port=args.port, id=args.id, arm_profile=args.profile)
    )
    leader.connect(calibrate=False)

    # Pre-flight: no external servo supply is the failure mode that looks like
    # a successful test reporting zeros. The servo MCU runs happily off the
    # adapter's 5 V, accepts Goal_Position and reports Torque_Enable=1, but the
    # motor driver has no headroom to turn the output. Catch it up front.
    volts = leader.bus.read("Present_Voltage", name, normalize=False) / 10.0
    if volts < MIN_DRIVE_VOLTS:
        print(
            f"\nRefusing to run: {name} reports {volts:.1f} V.\n"
            f"\nThat is USB bus power, not a servo supply. STS3215 is 7.4 V nominal\n"
            f"(12 V for the -12V variant). At this voltage the servo answers on the\n"
            f"bus and accepts Goal_Position, but cannot drive the output -- the test\n"
            f"would report 0.000 deg for every step and call it 'not detectable',\n"
            f"which would be a measurement artefact, not a property of the arm.\n"
            f"\nConnect the external supply to the Waveshare adapter and re-run.\n"
            f"Reading joint positions does NOT need this -- read_joints.py works\n"
            f"fine on USB power, because a leader arm is backdriven by hand.",
            file=sys.stderr,
        )
        leader.disconnect()
        return 1

    home = settle(leader, name)
    lo, hi = TRAVEL[name]
    worst = max(abs(s) for s in args.steps)
    if not (lo < home - worst and home + worst < hi):
        print(
            f"\nRefusing to run: {name} is at {home:.1f} deg, too close to its "
            f"travel limits ({lo:.1f} .. {hi:.1f}) for a +-{worst:g} deg test.\n"
            f"Move the joint nearer the middle of its range by hand and re-run.",
            file=sys.stderr,
        )
        leader.disconnect()
        return 1

    print(f"\njoint        {name}")
    print(f"start pose   {home:.3f} deg   (travel {lo:.1f} .. {hi:.1f})")
    print(f"steps        {', '.join(f'{s:g}' for s in args.steps)} deg, {args.reps} reps each")
    print(f"excursion    capped at +-{args.max_excursion:g} deg from the start pose")
    print("\nTHIS WILL ENABLE TORQUE AND MOVE THE JOINT.")
    print("Goal_Position is pre-set to the current pose so it cannot snap.")
    print("Torque is RAM-only; no EEPROM or calibration is touched.")
    if not args.yes:
        try:
            if input("\nProceed? [y/N] ").strip().lower() not in ("y", "yes"):
                print("aborted")
                leader.disconnect()
                return 0
        except EOFError:
            print("aborted (no tty)")
            leader.disconnect()
            return 0

    report: list[str] = [
        "",
        f"## Driven step response — `{name}`",
        "",
        f"Measured {time.strftime('%Y-%m-%d %H:%M:%S')}, start pose {home:.3f} deg, "
        f"{args.reps} reps per step.",
        "",
    ]
    rc = 0

    try:
        # CRITICAL ORDER: park the goal at the present position first, so
        # enabling torque holds the current pose instead of chasing a stale
        # register value.
        leader.bus.write("Goal_Position", name, home)
        leader.bus.enable_torque(name)
        time.sleep(0.2)

        report.append("| step | mean achieved | error | repeatability (stddev) | detectable |")
        report.append("|---|---|---|---|---|")
        print(f"\n{'step':>6}{'achieved':>11}{'error':>9}{'stddev':>9}{'ticks':>8}  verdict")
        print("-" * 60)

        for step in args.steps:
            if abs(step) > args.max_excursion:
                print(f"{step:>6g}  skipped - exceeds --max-excursion")
                continue
            achieved = []
            for _ in range(args.reps):
                goto(leader, name, home)          # return to the reference
                at_home = settle(leader, name)
                goto(leader, name, home + step)   # the step under test
                at_step = settle(leader, name)
                achieved.append(at_step - at_home)
            mean = statistics.fmean(achieved)
            sd = statistics.stdev(achieved) if len(achieved) > 1 else 0.0
            err = mean - step
            # Detectable means the move is distinguishable from not moving:
            # the achieved delta must exceed its own spread by a clear margin.
            detectable = abs(mean) > max(sd * 3, DEG_PER_TICK * 2)
            verdict = "detectable" if detectable else "NOT reliably detectable"
            print(
                f"{step:>6g}{mean:>11.3f}{err:>+9.3f}{sd:>9.4f}"
                f"{mean / DEG_PER_TICK:>8.1f}  {verdict}"
            )
            report.append(
                f"| {step:g} deg | {mean:.3f} | {err:+.3f} | {sd:.4f} | {verdict} |"
            )

            # A dead-flat zero is not a measurement, it is a broken setup --
            # the joint is jammed, stalled, or unpowered. Stop rather than
            # produce two more rows of the same artefact.
            if abs(mean) < DEG_PER_TICK:
                print(
                    f"\n{name} did not move at all for a {step:g} deg command.\n"
                    f"That is a setup fault, not a resolution limit. Check:\n"
                    f"  - servo supply voltage (now {volts:.1f} V, need >= {MIN_DRIVE_VOLTS:g} V)\n"
                    f"  - the joint is not against a hard stop or jammed\n"
                    f"  - Torque_Limit on this joint (elbow_flex ships at 500 vs 1000)\n"
                    f"Aborting so the remaining steps do not report the same artefact.",
                    file=sys.stderr,
                )
                rc = 1
                break

        # Backlash: same commanded target, reached from each side.
        print("\nbacklash (same target, approached from each side)")
        swing = max(args.steps) * 2
        from_below, from_above = [], []
        for _ in range(args.reps):
            goto(leader, name, home - swing)
            from_below.append(goto(leader, name, home))
            goto(leader, name, home + swing)
            from_above.append(goto(leader, name, home))
        mb, ma = statistics.fmean(from_below), statistics.fmean(from_above)
        gap = abs(mb - ma)
        print(f"  from below {mb:.3f}   from above {ma:.3f}")
        print(f"  -> backlash {gap:.3f} deg ({gap / DEG_PER_TICK:.1f} ticks)")
        report += [
            "",
            "| approach | settled position |",
            "|---|---|",
            f"| from below (−{swing:g} deg) | {mb:.3f} |",
            f"| from above (+{swing:g} deg) | {ma:.3f} |",
            f"| **backlash** | **{gap:.3f} deg** ({gap / DEG_PER_TICK:.1f} ticks) |",
            "",
        ]

    except KeyboardInterrupt:
        print("\n\ninterrupted")
        rc = 130
    finally:
        # Always, on every path. The arm must be left limp.
        try:
            leader.bus.disable_torque(name)
            print("torque disabled")
        except Exception as exc:  # noqa: BLE001 - must not mask the real error
            print(f"WARNING: could not disable torque on {name}: {exc}", file=sys.stderr)
            print("Power-cycle the arm before handling it.", file=sys.stderr)
        leader.disconnect()

    if args.out:
        with open(args.out, "a") as fh:
            fh.write("\n".join(report) + "\n")
        print(f"report appended to {args.out}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
