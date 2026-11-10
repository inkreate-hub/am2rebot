#!/usr/bin/env python
"""Sweep the AM-ARM200 leader and record each joint's output range.

Why not `lerobot-find-joint-limits`? That tool requires a connected follower and
a URDF, because what it actually measures is end-effector workspace bounds. We
only want the leader's own per-joint output span, in the exact units the leader
emits, so it can be mapped onto the B601's joint limits. No follower needed.

Read-only: torque is never enabled, nothing is written to EEPROM, the arm stays
limp. Move it by hand.

Output is pasted straight into the mapping shim as `joint_ranges`.

    ./lerobot_alohamini/.venv/bin/python find_leader_ranges.py
    ./lerobot_alohamini/.venv/bin/python find_leader_ranges.py --seconds 60
"""

import argparse
import sys
import time

from lerobot.teleoperators.so_leader import SOLeader
# SOLeaderConfig is the bare base dataclass; `id` and `calibration_dir` live on
# TeleoperatorConfig. SOLeaderTeleopConfig is the registered mix of both, and is
# what the CLI actually builds.
from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig

# The B601 follower's own joint limits, for side-by-side comparison.
# Source: RebotB601FollowerConfig.joint_limits (degrees).
B601_LIMITS = {
    "shoulder_pan": (-150, 150),
    "shoulder_lift": (-200, 1),
    "elbow_flex": (-200, 1),
    "wrist_flex": (-80, 90),
    "wrist_yaw": (-90, 90),
    "wrist_roll": (-90, 90),
    "gripper": (-270, 0),
}

parser = argparse.ArgumentParser()
parser.add_argument("--port", default=None,
                    help="serial device; auto-detected if omitted")
parser.add_argument("--id", default="am_leader")
parser.add_argument("--profile", default="am-leader-6dof")
parser.add_argument("--seconds", type=float, default=45.0)
parser.add_argument("--fps", type=float, default=30.0)
args = parser.parse_args()

leader = SOLeader(SOLeaderTeleopConfig(port=args.port, id=args.id, arm_profile=args.profile))
leader.connect(calibrate=False)

names = list(leader.bus.motors)
lo = dict.fromkeys(names, float("inf"))
hi = dict.fromkeys(names, float("-inf"))

print(f"\nSweep every joint through its full travel. {args.seconds:.0f}s, Ctrl-C to stop early.")
print("Push each joint to where you actually want the follower to reach.")
print("Don't forget the gripper: fully closed AND fully open.\n")

deadline = time.perf_counter() + args.seconds
period = 1.0 / args.fps
try:
    while time.perf_counter() < deadline:
        action = leader.get_action()
        for key, val in action.items():
            name = key.removesuffix(".pos")
            lo[name] = min(lo[name], val)
            hi[name] = max(hi[name], val)
        if sys.stdout.isatty():  # \r redraw only makes sense on a terminal
            live = "  ".join(f"{n[:9]}:{hi[n] - lo[n]:6.1f}" for n in names)
            print(f"\rspan  {live}", end="", flush=True)
        time.sleep(period)
except KeyboardInterrupt:
    print("\n(stopped early)")

leader.disconnect()

print("\n\n" + "=" * 74)
print(f"{'joint':<15}{'min':>9}{'max':>9}{'span':>9}   {'B601 limit':>14}  fits?")
print("-" * 74)
for name in names:
    span = hi[name] - lo[name]
    bmin, bmax = B601_LIMITS[name]
    # A span smaller than ~10 means the joint was barely moved -- not a real limit.
    note = "NOT SWEPT" if span < 10 else ("ok" if lo[name] >= bmin and hi[name] <= bmax else "needs map")
    print(f"{name:<15}{lo[name]:9.1f}{hi[name]:9.1f}{span:9.1f}   {f'({bmin}, {bmax})':>14}  {note}")
print("=" * 74)

print("\nMeasured leader ranges, for the shim:\n")
print("joint_ranges = {")
for name in names:
    print(f'    "{name}": [{lo[name]:.1f}, {hi[name]:.1f}],')
print("}")
print("\nAnything marked NOT SWEPT is a measurement gap, not a mechanical limit --")
print("re-run and move that joint.")
