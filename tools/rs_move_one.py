#!/usr/bin/env python
"""Stage 5: the first torque the arm ever sees. One joint, bounded, logged.

    # look before you leap -- prints the plan, enables nothing
    ./lerobot_alohamini/.venv/bin/python tools/rs_move_one.py --port can0 \
        --joint wrist_yaw --dry-run

    # the real thing: +-5 deg around the present pose, 1 deg per command
    ./lerobot_alohamini/.venv/bin/python tools/rs_move_one.py --port can0 \
        --joint wrist_yaw --amplitude 5 --kp 8 --kd 1

Start with a WRIST joint, not a shoulder. The wrists are RS00 (14 N.m); the
shoulders are RS06 (36 N.m) and carry the arm's weight. If anything about the
scale factor or the gains is still wrong, you want to find out on the small
motor. Keep a hand on the DC switch.

HOW IT AVOIDS A SNAP ON ENABLE

MIT mode is a torque law: the motor applies kp*(q_target - q) + kd*(0 - qdot)
from whatever command it last received. Enabling torque with a stale target one
whole revolution away is how arms get broken. So the sequence is:

  1. read the present position with torque off
  2. enable the one motor
  3. immediately command the present position with kp = 0 -- pure damping, no
     restoring force, so nothing can snap
  4. ramp kp to the requested value over `--engage-seconds`
  5. only then start stepping

and torque is disabled in a `finally`, on clean exit, Ctrl-C and exception
alike.

WHAT IT CHECKS

Every command is capped at `--max-step-deg` and the achieved motion is compared
against the commanded motion. A ratio near 9 or 1/9 is the PMAX scale error
from stage 3 showing up under torque -- the run aborts rather than continuing.

See docs/B601_BRINGUP.md.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from lerobot.motors.robstride.tables import ControlMode  # noqa: E402
from rs_probe import CAN_IDS, SHOULDER_JOINTS, build_bus  # noqa: E402

#: `_encode_mit_packet` maps kd onto 0..5 with `_float_to_uint`, which CLAMPS.
#: lerobot's RebotB601FollowerConfig suggests mit_kd = 12.0 for the shoulder
#: joints -- that is the Damiao path, and here it would silently become 5.
KD_MAX = 5.0
KP_MAX = 500.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="can0")
    parser.add_argument("--interface", default="socketcan")
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--shoulder-model", default="O6")
    parser.add_argument("--wrist-model", default="O0")
    parser.add_argument("--recv-id-offset", type=int, default=0)

    parser.add_argument("--joint", required=True, choices=sorted(CAN_IDS))
    parser.add_argument("--amplitude", type=float, default=5.0,
                        help="degrees either side of the present position")
    parser.add_argument("--kp", type=float, default=8.0, help="0..500")
    parser.add_argument("--kd", type=float, default=1.0, help="0..5, CLAMPED")
    parser.add_argument("--max-step-deg", type=float, default=1.0,
                        help="largest change per command")
    parser.add_argument("--settle", type=float, default=0.3,
                        help="seconds to hold each step before logging")
    parser.add_argument("--engage-seconds", type=float, default=1.0,
                        help="kp ramp from 0 to --kp before any motion")
    parser.add_argument("--abort-deg", type=float, default=15.0,
                        help="kill torque if the error ever exceeds this")
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--dry-run", action="store_true",
                        help="print the plan, never enable torque")
    args = parser.parse_args()

    if args.kd > KD_MAX:
        print(f"--kd {args.kd} exceeds the encoder's 0..{KD_MAX} range and "
              f"would be silently clamped. Pass {KD_MAX} or less.",
              file=sys.stderr)
        return 2
    if not 0.0 <= args.kp <= KP_MAX:
        print(f"--kp must be 0..{KP_MAX}", file=sys.stderr)
        return 2

    name = args.joint
    if name in SHOULDER_JOINTS:
        print(f"!! {name} is a shoulder joint (RS06, 36 N.m) and carries the "
              f"arm's weight.\n   Do a wrist joint first. Support the arm so "
              f"nothing falls if torque cuts.")
        if input("type 'shoulder' to continue: ").strip() != "shoulder":
            return 1

    bus = build_bus(args)
    bus.connect(handshake=True)
    print(f"handshake ok. driving {name} (CAN id {CAN_IDS[name]}) as "
          f"{bus._motor_types[name].name}\n")

    try:
        # Mode only -- switch_to_mode sets the operation mode without
        # energising anything. Torque comes later, for one motor.
        bus.switch_to_mode(ControlMode.MIT)
        bus.disable_torque()
        time.sleep(0.1)

        home = float(bus.read("Present_Position", name))
        plan = _plan(home, args.amplitude, args.cycles)
        print(f"home {home:+.2f} deg, plan: "
              + " -> ".join(f"{p:+.1f}" for p in plan))
        print(f"steps of at most {args.max_step_deg} deg, kp {args.kp}, "
              f"kd {args.kd}, abort at {args.abort_deg} deg error\n")

        if args.dry_run:
            print("--dry-run: torque was never enabled.")
            return 0

        return _run(bus, args, name, home, plan)
    finally:
        try:
            bus.disable_torque()
            print("\ntorque disabled.")
        finally:
            bus.disconnect(disable_torque=True)


def _plan(home: float, amplitude: float, cycles: int) -> list[float]:
    """Waypoints: out, back through home, the other way, back. Both directions
    matter -- a one-sided test cannot show backlash or an asymmetric gain."""
    out: list[float] = []
    for _ in range(max(1, cycles)):
        out += [home + amplitude, home, home - amplitude, home]
    return out


def _run(bus, args, name: str, home: float, plan: list[float]) -> int:
    target = home
    bus.enable_torque([name])
    # Command the present position with NO restoring force, then ramp. Nothing
    # can snap while kp is 0.
    bus.write("Kp", name, 0.0)
    bus.write("Kd", name, args.kd)
    bus.write("Goal_Position", name, target)

    steps = max(1, int(args.engage_seconds / 0.02))
    for i in range(steps):
        bus.write("Kp", name, args.kp * (i + 1) / steps)
        bus.write("Goal_Position", name, target)
        time.sleep(0.02)
    print(f"engaged: kp ramped to {args.kp} over {args.engage_seconds:.1f} s")

    print(f"\n{'target':>9}{'measured':>10}{'err':>8}{'moved':>8}"
          f"{'ratio':>8}{'torque':>8}{'temp':>7}")
    print("-" * 58)

    worst_err = 0.0
    for waypoint in plan:
        while abs(waypoint - target) > 1e-6:
            before = float(bus.read("Present_Position", name))
            delta = max(-args.max_step_deg,
                        min(args.max_step_deg, waypoint - target))
            target += delta
            bus.write("Goal_Position", name, target)
            time.sleep(args.settle)

            state = bus.sync_read_all_states([name])[name]
            pos = float(state["position"])
            err = pos - target
            moved = pos - before
            worst_err = max(worst_err, abs(err))
            ratio = moved / delta if abs(delta) > 1e-6 else float("nan")

            print(f"{target:>9.2f}{pos:>10.2f}{err:>8.2f}{moved:>8.2f}"
                  f"{ratio:>8.2f}{state['torque']:>8.2f}"
                  f"{state['temp_mos']:>7.1f}")

            if abs(err) > args.abort_deg:
                print(f"\nABORT: error {err:+.2f} deg exceeds "
                      f"--abort-deg {args.abort_deg}.")
                return 1
            if abs(ratio) > 4.0 or (abs(moved) > 0.2 and abs(ratio) < 0.25):
                print(f"\nABORT: moved {moved:+.2f} deg for a "
                      f"{delta:+.2f} deg command, ratio {ratio:.2f}.\n"
                      f"A ratio near 9 or 1/9 is the PMAX model error -- go "
                      f"back to stage 3 (rs_probe.py --backdrive {name}).")
                return 1

    print(f"\nworst tracking error: {worst_err:.2f} deg")
    if worst_err > 2.0:
        print("GATE 5 NOT PASSED: tracking is loose. Raise --kp a little, or "
              "the joint is fighting gravity with too little gain.")
        return 1
    print(f"GATE 5 PASSED for {name}: tracked +-{args.amplitude} deg, no "
          f"fault, no scale error.")
    print("Next joint. Wrists before shoulders.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
