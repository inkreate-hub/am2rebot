#!/usr/bin/env python
"""Stages 2-4: handshake, read, settle the PMAX scale factor, zero, measure limits.

Everything here is read-only except `--zero` (which moves no joint) and
`--ranges` (which you drive by hand). Torque is never commanded.

    # stage 2: is the bus healthy and are the readings stable?
    ./lerobot_alohamini/.venv/bin/python tools/rs_probe.py --port can0

    # stage 3: THE important one -- measure the 9x scale ambiguity by hand
    ./lerobot_alohamini/.venv/bin/python tools/rs_probe.py --port can0 \
        --backdrive shoulder_pan

    # stage 4: zero from the sit-down pose, then record real joint limits
    ./lerobot_alohamini/.venv/bin/python tools/rs_probe.py --port can0 --zero
    ./lerobot_alohamini/.venv/bin/python tools/rs_probe.py --port can0 --ranges

WHY --backdrive EXISTS

`RobstrideMotorsBus` encodes and decodes every position as a 16-bit fraction of
+-PMAX, looked up per motor model. `O0` has PMAX 12.57 rad; `O6` has 112.5 rad.
A wrong model string is therefore an 8.95x error on commands *and* on readback.

Which way the mapping goes is a guess: Seeed's wiki says J1-J3 are RS06 and
J4-J7 are RS00, lerobot's table is named O0..O6, and the correspondence is
documented nowhere. Because the same constant scales both directions, the
factor can be measured with the motors de-energised -- move the joint by a
physically known angle and see which candidate PMAX reproduces it. No torque,
no risk, and it settles the question before anything is energised.

See docs/B601_BRINGUP.md for the full staged plan.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np  # noqa: E402

from lerobot.motors.motors_bus import Motor, MotorNormMode  # noqa: E402
from lerobot.motors.robstride.robstride import RobstrideMotorsBus  # noqa: E402
from lerobot.motors.robstride.tables import (  # noqa: E402
    MOTOR_LIMIT_PARAMS,
    MotorType,
)

#: CAN id per joint, 1..7, from Seeed's B601-RS wiki.
CAN_IDS: dict[str, int] = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_yaw": 5,
    "wrist_roll": 6,
    "gripper": 7,
}

#: The three shoulder joints carry the larger motor (RS06 per Seeed).
SHOULDER_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex")

#: Starting guess, and deliberately the SAFE direction of the error. If the
#: host's PMAX is too large, commands compress toward zero and the joint
#: under-moves; too small and a 10 deg command becomes 90 deg. Stage 3 replaces
#: these with measurements.
DEFAULT_SHOULDER_MODEL = "O6"
DEFAULT_WRIST_MODEL = "O0"


def build_bus(args: argparse.Namespace) -> RobstrideMotorsBus:
    """Construct the bus with one model string per joint and no calibration.

    `recv_id` is the subtle one. lerobot decodes feedback into its state cache
    via `_id_to_name[data[0]]`, keyed on `recv_id if recv_id is not None else
    id`. Seeed document the RS motors as plain ids 1..7, while lerobot's
    *Damiao* B601 config pairs each joint with a feedback id 0x11..0x17. Run
    `tools/can_check.py` first and use whatever `data[0]` actually contains:
    `--recv-id-offset 0` for the device id, `16` for 0x10+id.
    """
    motors: dict[str, Motor] = {}
    for name, can_id in CAN_IDS.items():
        model = args.shoulder_model if name in SHOULDER_JOINTS else args.wrist_model
        motors[name] = Motor(
            id=can_id,
            model=model,
            norm_mode=MotorNormMode.DEGREES,
            motor_type_str=model,
            recv_id=(can_id + args.recv_id_offset) if args.recv_id_offset else None,
        )

    return RobstrideMotorsBus(
        port=args.port,
        motors=motors,
        can_interface=args.interface,
        # Classic CAN. Seeed document 1 Mbit/s classic; this is the argument
        # motorbridge does not offer, and the reason this is the transport.
        use_can_fd=False,
        bitrate=args.bitrate,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="can0")
    parser.add_argument("--interface", default="socketcan",
                        help="'slcan' for a serial dongle")
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--shoulder-model", default=DEFAULT_SHOULDER_MODEL,
                        help=f"model string for {SHOULDER_JOINTS}")
    parser.add_argument("--wrist-model", default=DEFAULT_WRIST_MODEL)
    parser.add_argument("--recv-id-offset", type=int, default=0,
                        help="0 = feedback keyed on the device id; 16 = 0x10+id")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument("--backdrive", metavar="JOINT",
                        help="stage 3: hand-move this joint and report the "
                             "travel under every candidate PMAX")
    parser.add_argument("--zero", action="store_true",
                        help="stage 4: set the present pose as zero on all "
                             "seven motors (park in the sit-down pose first)")
    parser.add_argument("--ranges", action="store_true",
                        help="stage 4: record joint limits by hand")
    args = parser.parse_args()

    if args.backdrive and args.backdrive not in CAN_IDS:
        print(f"unknown joint {args.backdrive!r}; expected one of "
              f"{list(CAN_IDS)}", file=sys.stderr)
        return 2

    bus = build_bus(args)
    print(f"=== {args.port} @ {args.bitrate} bit/s, classic CAN "
          f"(use_can_fd=False)")
    print(f"models: {', '.join(SHOULDER_JOINTS)} = {args.shoulder_model}; "
          f"wrists + gripper = {args.wrist_model}")

    try:
        bus.connect(handshake=True)
    except Exception as exc:  # noqa: BLE001
        print(f"\nconnect/handshake failed: {exc}", file=sys.stderr)
        print(
            "\nThe handshake polls every motor and raises if any is silent or "
            "reports a fault. Run tools/can_check.py to tell those apart -- it "
            "also detects motors sitting in RobStride's private protocol "
            "instead of MIT comm mode, which looks identical from here.",
            file=sys.stderr,
        )
        return 1

    print("handshake ok: 7/7 answered, no faults\n")

    try:
        # Torque is never enabled by this tool. Make that true rather than
        # assumed -- the motors may have been left energised by an earlier run,
        # and record_ranges_of_motion re-enables torque on its way out.
        bus.disable_torque()

        if args.zero:
            return stage_zero(bus)
        if args.ranges:
            return stage_ranges(bus)
        if args.backdrive:
            return stage_backdrive(bus, args)
        return stage_read(bus, args)
    finally:
        try:
            bus.disable_torque()
        finally:
            bus.disconnect(disable_torque=True)


# --------------------------------------------------------------- stage 2: read


def stage_read(bus: RobstrideMotorsBus, args: argparse.Namespace) -> int:
    """Stream every motor's state. Gate 2 wants this stable and fault-free."""
    period = 1.0 / max(args.hz, 0.1)
    deadline = time.monotonic() + args.seconds
    first: dict[str, float] = {}
    drift: dict[str, float] = {name: 0.0 for name in CAN_IDS}

    print(f"{'t':>6}  " + "".join(f"{n.split('_')[-1][:6]:>8}" for n in CAN_IDS))
    t0 = time.monotonic()
    while time.monotonic() < deadline:
        states = bus.sync_read_all_states()
        row = f"{time.monotonic() - t0:>6.1f}  "
        for name in CAN_IDS:
            pos = float(states[name]["position"])
            first.setdefault(name, pos)
            drift[name] = max(drift[name], abs(pos - first[name]))
            row += f"{pos:>8.2f}"
        print(row)
        time.sleep(period)

    states = bus.sync_read_all_states()
    print(f"\n{'joint':<15}{'pos':>9}{'vel':>9}{'torque':>9}"
          f"{'temp C':>9}{'drift':>9}")
    print("-" * 69)
    for name in CAN_IDS:
        s = states[name]
        print(f"{name:<15}{s['position']:>9.2f}{s['velocity']:>9.2f}"
              f"{s['torque']:>9.2f}{s['temp_mos']:>9.1f}{drift[name]:>9.2f}")

    worst = max(drift.values())
    print(f"\nworst drift while stationary: {worst:.2f} deg")
    if worst > 1.0:
        print("GATE 2 FAILED: readings are not stable with torque off. Either "
              "the arm is moving under gravity, or frames are being dropped "
              "and you are seeing a stale cache.")
        return 1
    print("GATE 2 PASSED: 7/7 healthy, readings stable, no torque applied.")
    return 0


# ---------------------------------------------------- stage 3: the scale factor


def stage_backdrive(bus: RobstrideMotorsBus, args: argparse.Namespace) -> int:
    """Hand-move one joint and report its travel under every candidate PMAX.

    The whole point is that this needs no torque. The encode and the decode use
    the same PMAX, so the readback is wrong by exactly the factor a command
    would be wrong by -- which makes a hand-driven joint a perfectly good
    instrument for measuring it.
    """
    name = args.backdrive
    configured = bus._motor_types[name]
    pmax_cfg = MOTOR_LIMIT_PARAMS[configured][0]

    print(f"=== back-drive {name} (CAN id {CAN_IDS[name]}), "
          f"configured as {configured.name}, PMAX {pmax_cfg} rad")
    print("TORQUE IS OFF. Move the joint by hand through a known angle -- a "
          "quarter turn, or stop to stop with a protractor.")
    print("Ctrl-C when done.\n")

    lo = hi = float(bus.read("Present_Position", name))
    try:
        while True:
            pos = float(bus.read("Present_Position", name))
            lo, hi = min(lo, pos), max(hi, pos)
            print(f"\r  now {pos:>9.2f}   span {hi - lo:>9.2f} deg "
                  f"(as {configured.name})   ", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()

    span_cfg = hi - lo
    if span_cfg < 1.0:
        print(f"\nonly {span_cfg:.2f} deg of travel recorded -- move the joint "
              f"further, this cannot discriminate between candidates.")
        return 1

    # Ticks are the invariant: the raw 16-bit code the motor actually sent.
    ticks = np.radians(span_cfg) / (2.0 * pmax_cfg) * 65535.0

    print(f"\nrecorded {span_cfg:.2f} deg as {configured.name} "
          f"= {ticks:.0f} ticks of the 16-bit position field\n")
    print(f"{'model':<8}{'PMAX rad':>10}{'implied travel':>17}"
          f"{'resolution':>14}")
    print("-" * 49)
    for candidate in MotorType:
        pmax = MOTOR_LIMIT_PARAMS[candidate][0]
        implied = span_cfg * (pmax / pmax_cfg)
        res = np.degrees(2.0 * pmax / 65535.0)
        marker = "  <-- configured" if candidate is configured else ""
        print(f"{candidate.name:<8}{pmax:>10.2f}{implied:>14.1f} deg"
              f"{res:>11.4f} deg{marker}")

    print(
        "\nPick the row matching the angle you actually moved. Note that O0-O5 "
        "and ELO5 all share PMAX 12.57, so they are indistinguishable here -- "
        "this test separates the 12.57 family from O6's 112.5, which is the "
        "8.95x ambiguity that matters.\n"
        "Write the answer into RS_MOTOR_MODELS / LEROBOT_MODEL_NAMES in "
        "ah/backends/real_b601.py, and say it was measured."
    )
    return 0


# ------------------------------------------------- stage 4: zero and the limits


def stage_zero(bus: RobstrideMotorsBus) -> int:
    """Set the present pose as zero on all seven motors.

    The B601 keeps no persistent calibration -- it zeroes against whatever pose
    it is physically holding, every session. So this is not a one-time setup
    step; it is part of every power-up, and the pose it is done in defines what
    every later angle means.
    """
    print("=== zeroing all seven motors against the CURRENT pose")
    print("The arm must be parked in its sit-down pose. Every joint angle from "
          "here on is measured from wherever it is standing right now.")
    if input("type 'zero' to confirm: ").strip() != "zero":
        print("aborted, nothing written")
        return 1

    before = bus.sync_read("Present_Position")
    bus.set_zero_position()
    time.sleep(0.2)
    after = bus.sync_read("Present_Position")

    print(f"\n{'joint':<15}{'before':>10}{'after':>10}")
    print("-" * 35)
    for name in CAN_IDS:
        print(f"{name:<15}{float(before[name]):>10.2f}{float(after[name]):>10.2f}")

    worst = max(abs(float(after[n])) for n in CAN_IDS)
    print(f"\nworst residual: {worst:.2f} deg")
    if worst > 1.0:
        print("the motors did not all take the zero command -- do not trust "
              "any angle until they read ~0 here")
        return 1
    print("zeroed. This holds until power is cut.")
    return 0


def stage_ranges(bus: RobstrideMotorsBus) -> int:
    """Record each joint's real travel by hand.

    This is where the real joint limits come from. They are NOT the MuJoCo
    model's: `rs_arm.xml` has shoulder_lift and elbow_flex at -11.5..179.9 deg,
    lerobot's config has them at -200..1 -- different zero conventions, about
    180 deg apart. `JointMapper.from_ranges` reads the follower's limits from
    the live backend, so the mapping offsets re-derive themselves once the real
    backend reports these.

    ⚠️ lerobot's `record_ranges_of_motion` calls `enable_torque()` on its way
    out, which would energise all seven motors the moment you press ENTER. This
    wrapper disables it again immediately, and the caller's `finally` disables
    it a second time.
    """
    print("=== recording joint ranges by hand")
    print("Zero the arm first (--zero) or these numbers mean nothing.")
    print("Torque is off. Move every joint to BOTH stops, gently, then ENTER.")
    print("Values are truncated to whole degrees by lerobot -- that is its "
          "behaviour, not a measurement limit.\n")

    try:
        mins, maxes = bus.record_ranges_of_motion(display_values=True)
    finally:
        bus.disable_torque()

    print(f"\n{'joint':<15}{'min':>9}{'max':>9}{'span':>9}")
    print("-" * 42)
    for name in CAN_IDS:
        lo, hi = float(mins[name]), float(maxes[name])
        print(f"{name:<15}{lo:>9.1f}{hi:>9.1f}{hi - lo:>9.1f}")

    print("\nPaste into RealB601Backend.model_joint_limits() as MEASURED "
          "values:\n")
    print("    REAL_LIMITS: dict[str, tuple[float, float]] = {")
    for name in CAN_IDS:
        print(f'        "{name}": ({float(mins[name]):.1f}, '
              f'{float(maxes[name]):.1f}),')
    print("    }")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
