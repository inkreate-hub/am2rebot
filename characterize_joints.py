#!/usr/bin/env python
"""Characterise the AM-ARM200 leader: noise floor, step detectability, backlash.

Covers the Subtask 1 acceptance criteria that need measured evidence rather than
assertion:

  --noise      Static noise floor. Arm held still, N samples per joint, reports
               stddev and peak-to-peak in degrees. This is the number that
               decides whether a 1 deg move is distinguishable from sitting
               still -- everything else is guesswork without it.

  --steps      Guided 1 / 2 / 5 deg step test, one joint at a time. You move the
               joint, it measures what actually changed and says whether the
               step cleared the noise floor measured above.

  --backlash   Return-to-position hysteresis. Approach the same physical pose
               from one side, then the other, and report the gap. This is the
               mechanical slop in the gearbox and linkage, and it is normally
               far larger than the sensor noise.

  --signs      Record which physical direction makes each joint's value
               increase. Not derivable from the servo tables -- it depends on
               how this arm was assembled. Writes the result as a dict you can
               paste into read_joints.py.

  --all        noise, then signs, then steps, then backlash.

Read-only: torque is never enabled and nothing is written to servo EEPROM.

    ./lerobot_alohamini/.venv/bin/python characterize_joints.py --noise
    ./lerobot_alohamini/.venv/bin/python characterize_joints.py --all -o report.md
"""

import argparse
import statistics
import sys
import time

from read_joints import JOINT_ORDER

ARM_JOINTS = [n for n in JOINT_ORDER if n != "gripper"]

# STS3215: 12-bit absolute encoder over 360 deg.
TICKS_PER_REV = 4096
DEG_PER_TICK = 360.0 / TICKS_PER_REV  # 0.0879 deg -- the hard quantisation floor


def ask(prompt: str) -> bool:
    """Prompt, return False if the user wants to skip/quit."""
    try:
        reply = input(f"{prompt} [ENTER=go, s=skip, q=quit] ").strip().lower()
    except EOFError:
        return False
    if reply == "q":
        raise KeyboardInterrupt
    return reply != "s"


def sample(leader, n: int, fps: float = 100.0) -> dict[str, list[float]]:
    """Collect n samples of every joint."""
    out: dict[str, list[float]] = {name: [] for name in JOINT_ORDER}
    period = 1.0 / fps
    for _ in range(n):
        action = leader.get_action()
        for name in JOINT_ORDER:
            out[name].append(action[f"{name}.pos"])
        time.sleep(period)
    return out


def mean_of(leader, n: int = 30) -> dict[str, float]:
    """Averaged current position, to beat the noise down when locating a pose."""
    data = sample(leader, n)
    return {name: statistics.fmean(vals) for name, vals in data.items()}


def do_noise(leader, report: list[str], n: int) -> dict[str, float]:
    print("\n=== STATIC NOISE FLOOR ===")
    print(f"Do not touch the arm. Sampling {n} times per joint.\n")
    if not ask("Arm still?"):
        return {}
    data = sample(leader, n)

    print(f"{'joint':<15}{'mean':>10}{'stddev':>9}{'p2p':>8}{'ticks p2p':>11}")
    print("-" * 53)
    report.append("| joint | mean | stddev (deg) | peak-to-peak (deg) |")
    report.append("|---|---|---|---|")
    floors: dict[str, float] = {}
    for name in JOINT_ORDER:
        vals = data[name]
        sd = statistics.stdev(vals) if len(vals) > 1 else 0.0
        p2p = max(vals) - min(vals)
        floors[name] = p2p
        unit_p2p = p2p / DEG_PER_TICK if name != "gripper" else float("nan")
        print(
            f"{name:<15}{statistics.fmean(vals):>10.3f}{sd:>9.4f}{p2p:>8.3f}"
            f"{unit_p2p:>11.1f}"
        )
        report.append(
            f"| {name} | {statistics.fmean(vals):.3f} | {sd:.4f} | {p2p:.3f} |"
        )
    print(f"\nEncoder quantisation is {DEG_PER_TICK:.4f} deg/tick "
          f"({TICKS_PER_REV} ticks/rev), the floor nothing can beat.")
    report.append("")
    report.append(
        f"Encoder quantisation: {DEG_PER_TICK:.4f} deg/tick ({TICKS_PER_REV} ticks/rev)."
    )
    return floors


def do_signs(leader, report: list[str]) -> dict[str, str]:
    print("\n=== SIGN CONVENTION ===")
    print("For each joint, move it the way named, hold, then press ENTER.")
    print("Recording whether the reported value went up or down.\n")

    # Each entry: the motion to ask for, phrased so it is unambiguous to a human
    # standing in front of the arm.
    motions = {
        "shoulder_pan": "rotate the BASE counter-clockwise (seen from above)",
        "shoulder_lift": "LIFT the upper arm up, away from the table",
        "elbow_flex": "EXTEND the forearm, opening the elbow out straight",
        "wrist_flex": "pitch the wrist UP",
        "wrist_yaw": "yaw the wrist counter-clockwise (seen from above)",
        "wrist_roll": "roll the gripper clockwise (seen from behind the wrist)",
    }

    results: dict[str, str] = {}
    report.append("| joint | motion asked for | value |")
    report.append("|---|---|---|")
    for name, motion in motions.items():
        if not ask(f"\n{name}: {motion}.\n  Hold it there and"):
            results[name] = "skipped"
            continue
        before = mean_of(leader)[name]
        print("  ...measuring, keep holding")
        time.sleep(0.3)
        after = mean_of(leader)[name]
        # The user is holding a static pose; compare against where it started.
        # Ask them to return and re-measure for a real delta.
        print(f"  now return {name} roughly to where it started, then")
        if not ask("  "):
            results[name] = "skipped"
            continue
        back = mean_of(leader)[name]
        delta = after - back
        if abs(delta) < 0.5:
            verdict = "INCONCLUSIVE - joint barely moved"
        else:
            direction = "increases" if delta > 0 else "decreases"
            verdict = f"value {direction} when you {motion}"
        results[name] = verdict
        print(f"  -> {verdict}  (delta {delta:+.2f} deg)")
        report.append(f"| {name} | {motion} | {verdict} |")

    print("\nPaste into read_joints.py SIGNS:\n")
    print("SIGNS = {")
    for name in ARM_JOINTS:
        v = results.get(name, "unverified")
        if "increases" in v:
            print(f'    "{name}": "positive = {motions[name]}",')
        elif "decreases" in v:
            print(f'    "{name}": "negative = {motions[name]}",')
        else:
            print(f'    "{name}": "unverified",  # {v}')
    print('    "gripper": "0 = closed, 100 = open (normalised, not degrees)",')
    print("}")
    return results


def do_steps(leader, report: list[str], floors: dict[str, float]) -> None:
    print("\n=== STEP DETECTABILITY (1 / 2 / 5 deg) ===")
    print("Per joint: move it by roughly the named amount, both directions.")
    print("Measured against the noise floor, so 'detectable' means a real")
    print("signal rather than something lost in jitter.\n")

    report.append("| joint | nominal | measured + | measured - | verdict |")
    report.append("|---|---|---|---|---|")
    for name in ARM_JOINTS:
        floor = floors.get(name, 0.0)
        if not ask(f"\n--- {name} (noise floor {floor:.3f} deg). Test this joint?"):
            continue
        for nominal in (1.0, 2.0, 5.0):
            base = mean_of(leader)[name]
            if not ask(f"  move {name} about +{nominal:g} deg, hold, then"):
                continue
            plus = mean_of(leader)[name] - base
            if not ask(f"  now about -{nominal:g} deg back past the start, hold, then"):
                continue
            minus = mean_of(leader)[name] - base
            # A step is only meaningful if it clears the noise band with margin.
            ok_p = abs(plus) > max(floor, DEG_PER_TICK) * 2
            ok_m = abs(minus) > max(floor, DEG_PER_TICK) * 2
            verdict = "detectable" if (ok_p and ok_m) else "NOT reliably detectable"
            print(f"  {nominal:g} deg -> measured {plus:+.3f} / {minus:+.3f}  {verdict}")
            report.append(
                f"| {name} | {nominal:g} deg | {plus:+.3f} | {minus:+.3f} | {verdict} |"
            )


def do_backlash(leader, report: list[str]) -> None:
    print("\n=== BACKLASH / HYSTERESIS ===")
    print("Pick a visible physical reference per joint -- a mark, a hard stop,")
    print("a feature you can return to by eye. Approach it from one side, then")
    print("from the other. The gap between the two readings is the slop.\n")
    print("This is mechanical, and normally much larger than sensor noise.\n")

    report.append("| joint | from one side | from the other | backlash (deg) |")
    report.append("|---|---|---|---|")
    for name in ARM_JOINTS:
        if not ask(f"\n--- {name}. Test backlash on this joint?"):
            continue
        if not ask("  approach your reference pose from BELOW / one side, then"):
            continue
        a = mean_of(leader, 50)[name]
        print(f"  reading A: {a:.3f}")
        if not ask("  now move well away and approach the SAME pose from the other side, then"):
            continue
        b = mean_of(leader, 50)[name]
        print(f"  reading B: {b:.3f}")
        gap = abs(a - b)
        print(f"  -> backlash {gap:.3f} deg ({gap / DEG_PER_TICK:.1f} ticks)")
        report.append(f"| {name} | {a:.3f} | {b:.3f} | {gap:.3f} |")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--id", default="am_leader")
    parser.add_argument("--profile", default="am-leader-6dof")
    parser.add_argument("--samples", type=int, default=200, help="samples for the noise floor")
    parser.add_argument("--noise", action="store_true")
    parser.add_argument("--signs", action="store_true")
    parser.add_argument("--steps", action="store_true")
    parser.add_argument("--backlash", action="store_true")
    parser.add_argument("--all", action="store_true")
    parser.add_argument("-o", "--out", metavar="FILE", help="write a markdown report")
    args = parser.parse_args()

    if not any([args.noise, args.signs, args.steps, args.backlash, args.all]):
        parser.error("pick at least one of --noise --signs --steps --backlash --all")

    from lerobot.teleoperators.so_leader import SOLeader
    from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig

    leader = SOLeader(
        SOLeaderTeleopConfig(port=args.port, id=args.id, arm_profile=args.profile)
    )
    leader.connect(calibrate=False)

    report: list[str] = [
        "# AM-ARM200 leader characterisation",
        "",
        f"Generated {time.strftime('%Y-%m-%d %H:%M:%S')} from `characterize_joints.py`.",
        f"Port `{args.port}`, calibration id `{args.id}`, profile `{args.profile}`.",
        "",
    ]
    floors: dict[str, float] = {}

    try:
        if args.noise or args.all:
            report.append("## Static noise floor")
            report.append("")
            floors = do_noise(leader, report, args.samples)
            report.append("")
        if args.signs or args.all:
            report.append("## Sign convention")
            report.append("")
            do_signs(leader, report)
            report.append("")
        if args.steps or args.all:
            if not floors:
                print("\nNote: no noise floor measured, using encoder quantisation instead.")
                floors = dict.fromkeys(JOINT_ORDER, DEG_PER_TICK)
            report.append("## Step detectability")
            report.append("")
            do_steps(leader, report, floors)
            report.append("")
        if args.backlash or args.all:
            report.append("## Backlash")
            report.append("")
            do_backlash(leader, report)
            report.append("")
    except KeyboardInterrupt:
        print("\n\nstopped early")
    finally:
        leader.disconnect()

    if args.out:
        with open(args.out, "w") as fh:
            fh.write("\n".join(report) + "\n")
        print(f"\nreport written to {args.out}")
    else:
        print("\n(use -o FILE to save a markdown report)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
