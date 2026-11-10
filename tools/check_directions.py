#!/usr/bin/env python
"""Per-joint teleop diagnosis: which joint, which direction, where it breaks.

Run this when teleop "doesn't work right" but it is not obvious why. It walks
one joint at a time and separates the four failure modes, which are
indistinguishable while all six joints move at once:

    leader moves | target changes | sim moves | meaning
    -------------|----------------|-----------|---------------------------
    yes          | no             | no        | mapping bug
    yes          | yes            | no        | sim / controller bug
    yes          | yes            | yes       | works; sign may still be wrong
    wrong joint reacts            |           | joint-order / id mapping bug

For each joint it measures the leader delta on EVERY axis, so a move that
shows up on the wrong name is caught rather than silently mis-attributed --
the same coupling trap that made `verify_mapping.py` produce false positives
on the physical arm.

Direction is the one thing no measurement settles, because "correct" means
"matches the operator's intent". So it asks, once per joint.

    tools/check_directions.py              # headless, read the numbers
    tools/check_directions.py --window     # with the viewer, to see it

Read-only on the leader: torque is never enabled.
"""

import argparse
import os
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

if "--window" not in sys.argv:
    os.environ.setdefault("MUJOCO_GL", "egl")

from ah import ARM_JOINTS  # noqa: E402
from ah.arm import SafetyLimits  # noqa: E402
from ah.backends import MuJoCoBackend  # noqa: E402
from ah.leader import AMLeaderReader  # noqa: E402
from ah.mapping import JointMapper  # noqa: E402

#: How to ask for a move, per joint. Phrased for someone standing at the arm.
PROMPTS = {
    "shoulder_pan": "rotate the BASE to your LEFT",
    "shoulder_lift": "LIFT the upper arm UP, away from the table",
    "elbow_flex": "FOLD the forearm IN, closing the elbow",
    "wrist_flex": "pitch the wrist UP",
    "wrist_yaw": "yaw the wrist to your LEFT",
    "wrist_roll": "roll the gripper CLOCKWISE, seen from behind",
}

#: Below this a joint counts as not having moved, degrees. Well clear of the
#: 0.088 deg encoder quantisation and the measured 0.000 deg static noise.
MOVED_DEG = 3.0


def settle_sim(arm, reader, mapper, seconds=1.2, hz=30.0):
    """Run the teleop loop briefly so the sim catches up with the leader."""
    for _ in range(int(seconds * hz)):
        sample = reader.read()
        if sample is not None:
            arm.set_joint_positions(mapper(sample.position))
        arm.step()
        time.sleep(1.0 / hz)


def mean_leader(reader, n=15):
    """Averaged leader pose, to keep hand tremor out of the delta."""
    acc: dict[str, list[float]] = {}
    for _ in range(n):
        sample = reader.read()
        if sample is not None:
            for name, value in sample.position.items():
                acc.setdefault(name, []).append(value)
        time.sleep(0.01)
    return {name: statistics.fmean(v) for name, v in acc.items() if v}


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--window", action="store_true")
    parser.add_argument("--port", default=None,
                        help="serial device; auto-detected if omitted")
    parser.add_argument("--leader-id", default="am_leader")
    args = parser.parse_args()

    arm = MuJoCoBackend(
        limits=SafetyLimits(max_step_deg=8.0, max_speed_dps=240.0), scenery=True
    )
    arm.connect()
    arm.settle(0.5)

    mapper = JointMapper.from_ranges(arm.model_joint_limits())

    reader = AMLeaderReader(
        port=args.port or (AMLeaderReader.find_port() or "/dev/ttyACM0"),
        leader_id=args.leader_id,
    )
    try:
        reader.connect()
    except Exception as exc:  # noqa: BLE001
        print(f"cannot open leader on {args.port}: {exc}", file=sys.stderr)
        arm.disconnect()
        return 1

    viewer = None
    if args.window:
        import mujoco.viewer

        from view_sim import look_at_arm

        viewer = mujoco.viewer.launch_passive(
            arm.model, arm.data, show_left_ui=False, show_right_ui=False
        )
        look_at_arm(viewer.cam, arm.model)

    print("\nOne joint at a time. Move ONLY the joint named, then press ENTER.")
    print("Leave the others alone -- the tool checks that too.\n")

    results: dict[str, dict] = {}
    try:
        for joint in ARM_JOINTS:
            print(f"\n{'=' * 66}\n{joint}: {PROMPTS[joint]}")
            try:
                input("  park the arm comfortably, then press ENTER to mark the start ")
            except EOFError:
                break

            settle_sim(arm, reader, mapper)
            if viewer:
                viewer.sync()
            lead_before = mean_leader(reader)
            sim_before = arm.get_joint_positions()
            tgt_before = dict(arm._targets)

            try:
                input(f"  now {PROMPTS[joint]}, hold it, then press ENTER ")
            except EOFError:
                break

            settle_sim(arm, reader, mapper)
            if viewer:
                viewer.sync()
            lead_after = mean_leader(reader)
            sim_after = arm.get_joint_positions()
            tgt_after = dict(arm._targets)

            lead_delta = {
                n: lead_after.get(n, 0.0) - lead_before.get(n, 0.0) for n in lead_before
            }
            moved = {n: d for n, d in lead_delta.items() if abs(d) >= MOVED_DEG}
            biggest = max(lead_delta, key=lambda n: abs(lead_delta[n])) if lead_delta else None

            d_lead = lead_delta.get(joint, 0.0)
            d_tgt = tgt_after[joint] - tgt_before[joint]
            d_sim = sim_after[joint] - sim_before[joint]

            print(f"\n  leader {joint:<15} {d_lead:+8.2f} deg")
            print(f"  target {joint:<15} {d_tgt:+8.2f} deg")
            print(f"  sim    {joint:<15} {d_sim:+8.2f} deg")
            if len(moved) > 1:
                others = ", ".join(
                    f"{n} {d:+.1f}" for n, d in moved.items() if n != joint
                )
                print(f"  ALSO MOVED on the leader: {others}")
            if biggest and biggest != joint and abs(lead_delta[biggest]) > abs(d_lead):
                print(
                    f"  !! largest leader change was {biggest} "
                    f"({lead_delta[biggest]:+.1f}), not {joint} -- "
                    f"either you moved the wrong joint, or the id mapping is wrong"
                )

            # Where does it break?
            if abs(d_lead) < MOVED_DEG:
                verdict, ask = "NO LEADER MOVEMENT - joint not moved, or servo dead", False
            elif abs(d_tgt) < 0.5:
                verdict, ask = "MAPPING BUG - leader moved, target did not", False
            elif abs(d_sim) < 0.5:
                verdict, ask = "SIM BUG - target moved, simulated joint did not", False
            else:
                verdict, ask = "moves", True
            print(f"  -> {verdict}")

            same = None
            if ask:
                try:
                    reply = input(
                        "  did the SIM arm move the same way as your hand? [y/n/s] "
                    ).strip().lower()
                except EOFError:
                    reply = "s"
                if reply.startswith("y"):
                    same = True
                elif reply.startswith("n"):
                    same = False

            results[joint] = {
                "lead": d_lead, "tgt": d_tgt, "sim": d_sim,
                "verdict": verdict, "same": same,
            }
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        if viewer:
            viewer.close()
        reader.disconnect()
        arm.disconnect()

    print(f"\n{'=' * 78}\nSUMMARY\n")
    print(f"{'joint':<15}{'leader':>9}{'target':>9}{'sim':>9}  {'direction':<12} verdict")
    print("-" * 78)
    flips = []
    for joint in ARM_JOINTS:
        r = results.get(joint)
        if r is None:
            print(f"{joint:<15}{'-':>9}{'-':>9}{'-':>9}  {'skipped':<12}")
            continue
        if r["same"] is True:
            direction = "correct"
        elif r["same"] is False:
            direction = "INVERTED"
            flips.append(joint)
        else:
            direction = "unknown"
        print(
            f"{joint:<15}{r['lead']:>+9.2f}{r['tgt']:>+9.2f}{r['sim']:>+9.2f}"
            f"  {direction:<12} {r['verdict']}"
        )

    if flips:
        print(f"\nRe-run teleop with:\n  --flip {','.join(flips)}")
    else:
        print("\nNo inversions recorded.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
