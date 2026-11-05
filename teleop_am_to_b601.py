#!/usr/bin/env python
"""Teleoperate a reBot B601 follower from an AM-ARM200 leader.

`lerobot-teleoperate` cannot do this pair: it wires the leader's action dict
straight to the follower, and the two arms disagree on zero convention and on
scale (see am2rebot_mapping.py). This script runs the same loop with the mapping
inserted between them.

THREE MODES, in the order you should use them:

  --dry-run         No follower at all. Reads the leader, prints what the
                    follower *would* be commanded. Safe with no B601 connected,
                    no PSU, nothing powered. Use this first.

  --joints NAME     Drive only the named joints; every other joint is held at
                    its current follower position. This is how you verify
                    directions one joint at a time instead of discovering a
                    sign error with all seven moving at once.

  (no flags)        Full teleop, all seven joints.

Examples:

    # 1. no hardware beyond the leader -- check the numbers look sane
    ./lerobot_alohamini/.venv/bin/python teleop_am_to_b601.py --dry-run

    # 2. follower powered, one joint, slow. Repeat per joint.
    ./lerobot_alohamini/.venv/bin/python teleop_am_to_b601.py \
        --follower-port /dev/ttyACM1 --joints shoulder_pan --max-step 5

    # 3. everything, once directions are confirmed
    ./lerobot_alohamini/.venv/bin/python teleop_am_to_b601.py \
        --follower-port /dev/ttyACM1 --max-step 5

Keep a hand on the follower's power for 2 and 3.
"""

import argparse
import sys
import time

from am2rebot_mapping import B601_LIMITS, AmToB601Mapping

JOINTS = list(B601_LIMITS)

parser = argparse.ArgumentParser()
parser.add_argument("--leader-port", default="/dev/ttyACM0")
parser.add_argument("--leader-id", default="am_leader")
parser.add_argument("--follower-port", default="/dev/ttyACM1")
parser.add_argument("--follower-id", default="b601_follower")
parser.add_argument("--can-adapter", default="damiao", choices=["damiao", "socketcan"])
parser.add_argument("--dry-run", action="store_true", help="no follower; print targets only")
parser.add_argument(
    "--joints",
    nargs="+",
    metavar="NAME",
    choices=JOINTS,
    help=f"drive only these joints, hold the rest. one of: {', '.join(JOINTS)}",
)
parser.add_argument(
    "--max-step",
    type=float,
    default=5.0,
    help="max degrees any joint may move per control step. The B601 default is "
    "UNLIMITED; leave this set until directions are proven. 0 disables.",
)
parser.add_argument("--fps", type=float, default=30.0)
parser.add_argument("--seconds", type=float, default=0.0, help="0 = until Ctrl-C")
args = parser.parse_args()

from lerobot.teleoperators.so_leader import SOLeader  # noqa: E402
from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig  # noqa: E402

mapping = AmToB601Mapping()
active = set(args.joints) if args.joints else set(JOINTS)
if args.joints:
    print(f"driving only: {', '.join(sorted(active))}")
    print(f"holding:      {', '.join(n for n in JOINTS if n not in active)}")

leader = SOLeader(
    SOLeaderTeleopConfig(port=args.leader_port, id=args.leader_id, arm_profile="am-leader-6dof")
)
leader.connect(calibrate=False)

follower = None
if not args.dry_run:
    from lerobot.robots.rebot_b601_follower import (  # noqa: E402
        RebotB601Follower,
        RebotB601FollowerRobotConfig,
    )

    # max_relative_target is the follower's own per-step clamp. It defaults to
    # None -- no cap -- on a 24 V arm, so a sign error becomes a fast swing.
    follower = RebotB601Follower(
        RebotB601FollowerRobotConfig(
            port=args.follower_port,
            id=args.follower_id,
            can_adapter=args.can_adapter,
            max_relative_target=(args.max_step if args.max_step > 0 else None),
        )
    )
    follower.connect()
    print("\nfollower connected. B601 does not persist calibration -- it zeroed")
    print("against whatever pose it was physically holding just now.\n")
else:
    print("\nDRY RUN -- no follower will be commanded.\n")

period = 1.0 / args.fps
deadline = time.perf_counter() + args.seconds if args.seconds > 0 else float("inf")
held: dict[str, float] = {}
n = 0

try:
    while time.perf_counter() < deadline:
        leader_action = leader.get_action()
        target = mapping(leader_action)

        if args.joints:
            # Hold inactive joints at the first value we mapped for them, so a
            # joint under test moves and nothing else does.
            for key in list(target):
                name = key.removesuffix(".pos")
                if name not in active:
                    held.setdefault(key, target[key])
                    target[key] = held[key]

        if follower is not None:
            follower.send_action(target)

        n += 1
        if sys.stdout.isatty() and n % 5 == 0:
            cells = " ".join(
                f"{name[:9]}:{target[f'{name}.pos']:7.1f}"
                + ("*" if name in active else " ")
                for name in JOINTS
            )
            print(f"\r{cells}", end="", flush=True)
        time.sleep(period)
except KeyboardInterrupt:
    print("\nstopped")
finally:
    leader.disconnect()
    if follower is not None:
        follower.disconnect()

    # Final snapshot, so a piped or logged run is not silent.
    if n:
        print(f"\n\n{n} steps. last mapped target:\n")
        print(f"{'joint':<15}{'leader':>9}{'-> follower':>14}   driven")
        print("-" * 50)
        for name in JOINTS:
            lv = leader_action[f"{name}.pos"]
            fv = target[f"{name}.pos"]
            print(f"{name:<15}{lv:9.1f}{fv:14.1f}   {'yes' if name in active else 'held'}")
    print("\ndisconnected")
