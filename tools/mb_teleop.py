#!/usr/bin/env python
"""Teleoperate the B601-RS follower from the AM-ARM200 leader, over motorbridge.

WHY THIS EXISTS AND `teleop_am_to_b601.py` DOES NOT WORK

That script drives the follower through lerobot's `RebotB601Follower`, which
calls `bus.add_damiao_motor(send_id, recv_id, MOTOR_MODELS[name])` against a
table hard-coded to Damiao part numbers -- `4340P` for the shoulders, `4310`
for the wrists. This arm is the **RS** variant: RobStride RS06 and RS00, which
need `add_robstride_motor` and the model strings `rs-06` / `rs-00`. It is not a
tuning mismatch, it is the wrong motor driver. So the follower side here is
`mb_bringup`, the same code path that drove six joints under power on
2026-11-12 and replayed a hand-recorded trajectory through them.

THE LIMITS IN `am2rebot_mapping.py` ARE SIGN-FLIPPED, DO NOT USE THEM

Its `B601_LIMITS` puts `shoulder_lift` and `elbow_flex` at (-200, 1): running
from a large negative angle up to roughly zero. Measured on this arm, torque
off, by hand:

    shoulder_lift   -0.35 .. +208.79
    elbow_flex      -0.18 .. +241.79

Both run from roughly zero *upwards*. There is no negative travel to speak of.
Feeding the leader through that table maps its mid-pose to about -100 deg on
both shoulders, so every frame of a teleop session would command two 36 N.m
joints into the stop they are already resting against -- the same stop that
read 58.7 N.m on the morning of 2026-11-12. This module reads
`calibration/b601_ranges.json` instead, which is the file `--move` and
`--replay` already refuse to run without.

ALIGNMENT IS BY POSE, NOT BY CENTRE

`--align centre` maps the middle of the leader's travel to the middle of the
follower's. That is the textbook fit and it is wrong to *start* with here:
`shoulder_lift`'s centre is +104 deg, so engaging teleop would immediately
hoist the arm 104 deg from the pose it is parked in.

`--align pose` (the default) computes each offset at engage time from where the
two arms actually are, so the first commanded target equals the follower's
current position and the approach move is zero. Match the leader's shape to the
follower's by hand first, then engage, exactly as you would on any
leader/follower rig.

DIRECTIONS ARE NOT VERIFIED ON THIS HARDWARE

`DEFAULT_DIRECTIONS` in `ah/mapping.py` was established by driving the *sim*
from the live leader. The physical arm has since been zeroed against a
different pose and measured end to end. A sign that was right in the model is
not thereby right here. Verify one joint at a time with `--joints`, and flip
with `--invert` rather than editing a constant you will forget about.

**Nothing here can detect a wrong sign for you, and `--max-engage-deg` is not
that check.** Pose alignment sets each offset so the first target equals the
follower's current position *whatever the direction is*, so a reversed joint
engages with a gap of zero like any other. It shows up the moment you move the
leader: the follower goes the other way. That is why joint one at a time, with
your hand on the DC switch, is the procedure and not a suggestion.

THE ORDER TO USE THIS IN

    # 1. no CAN at all. Leader only. Move it and watch the mapped targets.
    mb_teleop.py --dry-run

    # 2. one joint, follower powered. Confirm it follows the right way.
    mb_teleop.py --joints shoulder_pan

    # 3. add joints as each one proves out
    mb_teleop.py --joints shoulder_pan,shoulder_lift

    # 4. all six
    mb_teleop.py

The gripper is excluded throughout: it has no measured travel, because it was
swept while disassembled and returned 434 deg of free motor rotation.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import mb_bringup as mb  # noqa: E402
from ah.mapping import DEFAULT_DIRECTIONS, LEADER_RANGES  # noqa: E402

#: The six arm joints, in CAN id order. The gripper is deliberately absent --
#: see the module docstring.
JOINTS: tuple[str, ...] = (
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_yaw", "wrist_roll",
)

#: How little room a joint can have on one side of its mapped travel before
#: preflight calls it cornered. 5 deg is roughly one second of leader motion:
#: enough that the operator would read it as "this joint does not work" rather
#: than "this joint is at the end of its range".
CORNERED_DEG = 5.0

#: Short column headings. Truncating the real names gives `shou shou elbo wris
#: wris wris`, and a live run showed the full-width table running off the right
#: of the terminal with the follower block cut in half -- which is the half you
#: are actually watching.
ABBREV: dict[str, str] = {
    "shoulder_pan": "pan", "shoulder_lift": "lift", "elbow_flex": "elbow",
    "wrist_flex": "wflex", "wrist_yaw": "wyaw", "wrist_roll": "wroll",
}


class JointMap:
    """One joint's leader -> follower conversion, fitted from measured travel.

        follower_deg = clamp(direction * scale * leader_deg + offset)

    `scale` and the clamp come from the two measured ranges and are fixed for
    the session. `offset` is set once at engage time under `--align pose`, so
    it depends on where both arms were standing when you started.
    """

    def __init__(self, name: str, leader: tuple[float, float],
                 follower: tuple[float, float], direction: int,
                 margin: float) -> None:
        self.name = name
        self.direction = direction
        self.leader = leader
        # Hold off the measured endpoints. Those endpoints are where the joint
        # "stopped feeling free" under a hand, not the true mechanical stop,
        # and a joint that parks on its stop breaks away at a torque spike.
        lo, hi = follower
        if hi - lo > 2 * margin:
            lo, hi = lo + margin, hi - margin
        self.follower = (lo, hi)
        l_span = leader[1] - leader[0]
        f_span = hi - lo
        self.scale = 1.0 if l_span <= f_span else f_span / l_span
        self.offset = 0.0

    def raw(self, leader_deg: float) -> float:
        """The target before clamping. Used to report what would be lost."""
        return self.direction * self.scale * leader_deg + self.offset

    def apply(self, leader_deg: float) -> float:
        lo, hi = self.follower
        return min(hi, max(lo, self.raw(leader_deg)))

    def align_pose(self, leader_deg: float, follower_deg: float) -> None:
        """Set the offset so this leader angle maps exactly to this follower one."""
        self.offset = follower_deg - self.direction * self.scale * leader_deg

    def align_centre(self) -> None:
        l_mid = (self.leader[0] + self.leader[1]) / 2.0
        f_mid = (self.follower[0] + self.follower[1]) / 2.0
        self.offset = f_mid - self.direction * self.scale * l_mid

    def image(self) -> tuple[float, float]:
        """Where the leader's full travel lands, before clamping."""
        a, b = self.raw(self.leader[0]), self.raw(self.leader[1])
        return (min(a, b), max(a, b))

    def lost(self) -> float:
        """Fraction of the leader's travel that clips, 0.0 - 1.0."""
        lo, hi = self.image()
        span = hi - lo
        if span <= 0:
            return 0.0
        inside = min(hi, self.follower[1]) - max(lo, self.follower[0])
        return max(0.0, 1.0 - max(0.0, inside) / span)


#: Which joints have been watched moving the right way, and under which sign.
#:
#: This exists because "the joint moved" and "the joint moved the right way"
#: are different facts, and only the second one makes it safe to add a joint to
#: the set. Keeping it on disk rather than in anyone's head means the tool can
#: refuse, instead of the operator having to remember which of six joints were
#: checked three hours ago.
DIRECTIONS_PATH = REPO_ROOT / "calibration" / "teleop_directions.json"


def _load_directions() -> dict[str, dict]:
    try:
        return json.loads(DIRECTIONS_PATH.read_text()).get("joints", {})
    except (OSError, ValueError):
        return {}


def _confirm_directions(names: list[str], maps: dict[str, JointMap]) -> int:
    """Record that these joints were watched following the leader correctly."""
    bad = [n for n in names if n not in maps]
    if bad:
        print(f"--confirm: not driven this run: {', '.join(bad)}",
              file=sys.stderr)
        return 2
    doc = {"joints": _load_directions()}
    today = time.strftime("%Y-%m-%d")
    for n in names:
        doc["joints"][n] = {"direction": maps[n].direction, "verified": today}
    doc["note"] = ("Joints watched following the leader the correct way on "
                   "the physical arm. 'direction' is the sign that was "
                   "verified -- running with the opposite sign is NOT covered "
                   "by this entry.")
    doc["joints"] = {n: doc["joints"][n] for n in JOINTS if n in doc["joints"]}
    tmp = DIRECTIONS_PATH.with_suffix(".tmp")
    tmp.write_text(json.dumps(doc, indent=2) + "\n")
    tmp.replace(DIRECTIONS_PATH)
    print(f"recorded {', '.join(names)} as direction-verified "
          f"-> {DIRECTIONS_PATH}")
    return 0


def _direction_status(maps: dict[str, JointMap]) -> dict[str, str]:
    """Per joint: 'ok', 'unverified', or 'sign-changed'."""
    known = _load_directions()
    out = {}
    for n, jm in maps.items():
        entry = known.get(n)
        if entry is None:
            out[n] = "unverified"
        elif entry.get("direction") != jm.direction:
            out[n] = "sign-changed"
        else:
            out[n] = "ok"
    return out


def build_maps(args, ranges: dict[str, dict[str, float]]) -> dict[str, JointMap]:
    inverted = {n.strip() for n in args.invert.split(",") if n.strip()}
    bad = inverted - set(JOINTS)
    if bad:
        raise SystemExit(f"--invert: not arm joints: {', '.join(sorted(bad))}")
    maps = {}
    for name in JOINTS:
        if name not in ranges:
            continue
        d = DEFAULT_DIRECTIONS.get(name, 1)
        if name in inverted:
            d = -d
        maps[name] = JointMap(
            name,
            LEADER_RANGES[name],
            (ranges[name]["min_deg"], ranges[name]["max_deg"]),
            d, args.margin_deg,
        )
    return maps


def _refuse(msg: str) -> int:
    """Print a refusal to stderr, but after everything already on stdout.

    stdout is block-buffered through a pipe while stderr is not, so an
    unflushed refusal lands ABOVE the tables it is talking about -- which is
    how the first --preflight run read, with the verdict printed before the
    evidence.
    """
    sys.stdout.flush()
    print(msg, file=sys.stderr)
    sys.stderr.flush()
    return 1


def _print_map(maps: dict[str, JointMap]) -> None:
    print(f"\n{'joint':<15}{'dir':>4}{'scale':>7}{'offset':>9}"
          f"{'leader span':>16}{'-> follower':>18}{'clip':>7}")
    print("-" * 76)
    for name, jm in maps.items():
        lo, hi = jm.image()
        print(f"{name:<15}{jm.direction:>+4}{jm.scale:>7.2f}{jm.offset:>9.1f}"
              f"{jm.leader[0]:>8.1f} ..{jm.leader[1]:>6.1f}"
              f"{lo:>10.1f} ..{hi:>7.1f}{jm.lost() * 100:>6.0f}%")
    print("-" * 76)
    print("'clip' is how much of the leader's travel falls outside the "
          "follower's measured\ntravel and will be held at the limit. Within "
          "reason that is fine -- it is the\nsafety clamp doing its job, not "
          "an error.")


def _open_leader(args):
    """Connect the AM-ARM200. Read-only: nothing on the leader is ever driven."""
    from lerobot.teleoperators.so_leader import SOLeader
    from lerobot.teleoperators.so_leader.config_so_leader import (
        SOLeaderTeleopConfig,
    )

    leader = SOLeader(SOLeaderTeleopConfig(
        port=args.leader_port, id=args.leader_id,
        arm_profile="am-leader-6dof"))
    leader.connect(calibrate=False)
    return leader


def teleop(args) -> int:
    ranges = mb._load_ranges()
    if not ranges:
        print("no measured ranges. Run --ranges first:\n"
              f"    {Path(__file__).parent / 'mb_bringup.py'} --ranges",
              file=sys.stderr)
        return 2

    wanted = ([n.strip() for n in args.joints.split(",") if n.strip()]
              if args.joints else list(JOINTS))
    bad = [n for n in wanted if n not in JOINTS]
    if bad:
        print(f"not arm joints: {', '.join(bad)}\n"
              f"the gripper is excluded on purpose -- it has no measured "
              f"travel.", file=sys.stderr)
        return 2

    maps = build_maps(args, ranges)
    missing = [n for n in wanted if n not in maps]
    if missing:
        print(f"no measured travel for: {', '.join(missing)}\n"
              f"Sweep them before driving them:\n"
              f"    mb_bringup.py --ranges --joint {missing[0]}",
              file=sys.stderr)
        return 2
    maps = {n: maps[n] for n in JOINTS if n in wanted}

    print(f"leader {args.leader_port} (id {args.leader_id}), "
          f"follower {args.port}")
    print(f"driving: {', '.join(maps)}")
    held = [n for n in JOINTS if n not in maps]
    if held:
        print(f"holding: {', '.join(held)}  (commanded to stay where they are)")

    leader = _open_leader(args)
    ctrl = motors = None
    engaged = False
    try:
        action = leader.get_action()
        lead = {n: action[f"{n}.pos"] for n in JOINTS}

        if args.dry_run:
            if args.align == "centre":
                for jm in maps.values():
                    jm.align_centre()
            else:
                # No follower to read. Align to the measured mid-travel so the
                # numbers are at least in the right part of the range, and say
                # so -- a live run aligns to the real pose instead.
                for jm in maps.values():
                    jm.align_centre()
                print("\n--dry-run: no follower is read, so --align pose "
                      "cannot be honoured.\n            Showing centre "
                      "alignment. A live run will differ.")
            _print_map(maps)
            return _dry_loop(leader, maps, args)

        mb_sdk, ctrl, all_motors = mb._open(args)
        motors = {n: all_motors[n] for n in JOINTS}
        mb._reporting(motors.values(), True)
        start = mb._sample_all(ctrl, motors)
        missing_fb = [n for n in maps if "pos_deg" not in start.get(n, {})]
        if missing_fb:
            print(f"\nno feedback from: {', '.join(missing_fb)}. "
                  f"Run --scan.", file=sys.stderr)
            return 2

        if args.align == "centre":
            for jm in maps.values():
                jm.align_centre()
        else:
            for n, jm in maps.items():
                jm.align_pose(lead[n], start[n]["pos_deg"])
        _print_map(maps)

        first = {n: jm.apply(lead[n]) for n, jm in maps.items()}
        for n in JOINTS:
            if n not in first:
                first[n] = start[n]["pos_deg"]
        gap = max(abs(first[n] - start[n]["pos_deg"]) for n in motors)
        print(f"\n{'joint':<15}{'now':>9}{'first target':>14}{'gap':>9}"
              f"{'room down':>11}{'room up':>9}")
        print("-" * 67)
        for n in JOINTS:
            now = start[n]["pos_deg"]
            room = ""
            if n in maps:
                lo, hi = maps[n].image()
                room = f"{first[n] - lo:>11.1f}{hi - first[n]:>9.1f}"
            print(f"{n:<15}{now:>9.2f}{first[n]:>14.2f}"
                  f"{first[n] - now:>+9.2f}{room}"
                  + ("" if n in maps else "   held"))
        print(f"\nlargest move to engage: {gap:.2f} deg")

        # WHY THIS EXISTS -- 2026-11-12. A six-joint run came back as
        # "elbow_flex did not work". Nothing was wrong with it: the follower
        # elbow was parked at 0.04 deg against a mapped image bottoming out at
        # 0.0, and its direction is -1, so the leader was being pushed the one
        # way that had 2.8 deg of room instead of the way that had 185. Every
        # number needed to see that was already printed above, spread across
        # two tables, and none of it said "this joint is cornered".
        #
        # A joint parked on its limit is not an error -- it is where the arm
        # happens to be standing -- so this warns and does not block.
        cornered = []
        for n, jm in maps.items():
            lo, hi = jm.image()
            down, up = first[n] - lo, hi - first[n]
            if min(down, up) < CORNERED_DEG and max(down, up) > CORNERED_DEG:
                cornered.append((n, "down" if down < up else "up",
                                 min(down, up), max(down, up)))
        if cornered:
            print()
            for n, way, tight, free in cornered:
                other = "up" if way == "down" else "down"
                print(f"!! {n} is PARKED ON ITS LIMIT -- {tight:.1f} deg of "
                      f"room {way}, {free:.0f} deg {other}.")
            print("!! Those joints will not move if you drive the leader the "
                  "wrong way. They are not\n!! broken and this is not a "
                  "calibration fault -- it is the clamp holding them at the\n"
                  "!! end of their measured travel. Drive the leader the "
                  "other way, or re-park the arm.")

        status = _direction_status(maps)
        print(f"\n{'joint':<15}{'sign':>6}  direction")
        print("-" * 44)
        for n, st in status.items():
            note = {"ok": "verified on hardware",
                    "unverified": "NEVER WATCHED MOVING",
                    "sign-changed": "verified under the OPPOSITE sign"}[st]
            print(f"{n:<15}{maps[n].direction:>+6}  {note}")
        unsafe = [n for n, st in status.items() if st != "ok"]
        # A SINGLE unverified joint is not refused, because driving one joint
        # at a bounded step rate IS how a direction gets verified -- refusing
        # it would make the refusal's own advice impossible to follow. What is
        # refused is carrying an unknown sign into a multi-joint run, where a
        # reversed joint drives into a stop while five others are also moving.
        if unsafe and len(maps) > 1 and not args.skip_direction_check:
            return _refuse(
                f"\nREFUSED: {len(unsafe)} of {len(maps)} joints have never "
                f"been watched following the\nleader: {', '.join(unsafe)}\n"
                f"\nDo them ONE at a time -- that run is allowed, and it is "
                f"how this gets\ncleared -- then record each one:\n"
                f"    {Path(__file__).name} --joints {unsafe[0]}\n"
                f"    {Path(__file__).name} --joints {unsafe[0]} "
                f"--confirm {unsafe[0]}\n"
                f"\nAdd --invert {unsafe[0]} first if it follows backwards. "
                f"Override with\n--skip-direction-check, but a reversed "
                f"joint under power drives itself\ninto a stop at full "
                f"commanded speed.")
        if unsafe and len(maps) == 1:
            print(f"\n!! {unsafe[0]} has an UNVERIFIED direction. That is what "
                  f"this run is for.\n   Move the leader a little and watch "
                  f"which way it goes. If it is backwards,\n   stop and re-run "
                  f"with --invert {unsafe[0]}.")
        if gap > args.max_engage_deg:
            msg = (f"\nREFUSED: {gap:.2f} deg exceeds --max-engage-deg "
                   f"{args.max_engage_deg:.0f}.\n"
                   f"Move the LEADER until its shape matches the follower's, "
                   f"then run this again.")
            if args.align == "pose":
                msg += ("\nUnder --align pose the gap is only ever the "
                        "endpoint margin lifting a\njoint off its stop, so "
                        "this big a gap means a joint is parked outside\nits "
                        "own measured travel. Re-sweep it rather than widening "
                        "this limit.")
            return _refuse(msg)

        if args.preflight:
            gm = {} if args.no_gravity_model else mb._load_gravity_model()
            if gm:
                tq_line = ("residual vs fitted gravity model ("
                           + ", ".join(sorted(gm)) + ")")
            else:
                tq_line = ("fixed ceilings -- NO gravity model fitted, so "
                           "the guard is blind\n              in folded poses. "
                           "Run a session, then --fit-gravity")
            print(f"\n--preflight: everything above was read from the live "
                  f"arm. Nothing was\nenabled and no target was sent. The "
                  f"guards that will apply:\n"
                  f"    torque    {tq_line}\n"
                  f"    temp      {args.max_temp:.0f} degC\n"
                  f"    tracking  {args.abort_deg:.0f} deg from target\n"
                  f"    rate      {args.max_step_deg:.1f} deg per control "
                  f"step at {args.fps:.0f} Hz\n"
                  f"    velocity  {args.vlim_dps:.0f} dps, "
                  f"{args.acc_dpss:.0f} dps^2, clamped in firmware\n"
                  f"    on fault  "
                  + (f"freeze until {args.hold_max_temp:.0f} degC or Ctrl-C"
                     if args.hold_on_fault < 0 else
                     "release torque immediately" if args.hold_on_fault == 0
                     else f"hold {args.hold_on_fault:.0f} s, then release")
                  + "\n"
                  f"\nRun the same command without --preflight to engage.")
            return 0

        print(f"\n!! {len(motors)} JOINTS GO UNDER POWER. The follower will "
              f"track your hand\n   on the leader in real time, as fast as "
              f"you move it, bounded only by\n   --max-step-deg "
              f"{args.max_step_deg:.1f} per {1000.0 / args.fps:.0f} ms and "
              f"--vlim-dps {args.vlim_dps:.0f}.\n"
              f"   Support the arm. Keep a hand on the DC switch -- it is the "
              f"only abort\n   that works if the stream stalls.\n"
              + (f"   Ctrl-C and any fault FREEZE the arm in place. It holds "
                 f"until {args.hold_max_temp:.0f} degC\n   or a second "
                 f"Ctrl-C, so take the weight before you release it."
                 if args.hold_on_fault < 0 else
                 f"   Ctrl-C and any fault hold for "
                 f"{args.hold_on_fault:.0f} s, then DROP torque."))
        if sys.stdin.isatty():
            if input("type 'teleop' to continue: ").strip() != "teleop":
                print("aborted.")
                return 1

        for m in motors.values():
            m.ensure_mode(mb._mode_of(args), args.mode_timeout_ms)
        hold = {n: start[n]["pos_deg"] for n in JOINTS}
        mb._command_all(motors, args, hold)
        for m in motors.values():
            m.enable()
        engaged = True
        mb._command_all(motors, args, hold, hold=0.4)
        print("\nenabled, holding. Move the leader. Ctrl-C to stop.\n")
        rc = _live_loop(leader, ctrl, motors, maps, args, hold)
        if rc == 0 and args.confirm is not None:
            names = ([n.strip() for n in args.confirm.split(",") if n.strip()]
                     or list(maps))
            rc = _confirm_directions(names, maps)
        return rc
    except KeyboardInterrupt:
        print("\n\ninterrupted.")
        return 1
    finally:
        if engaged:
            for n, m in motors.items():
                try:
                    m.disable()
                except Exception as exc:  # noqa: BLE001
                    print(f"\n!! could not disable {n}: {exc}\n"
                          f"!! CUT THE DC SWITCH.", file=sys.stderr)
            print("torque disabled.")
        if motors is not None:
            mb._reporting(motors.values(), False)
        if ctrl is not None:
            mb._shutdown(ctrl)
        try:
            leader.disconnect()
        except Exception as exc:  # noqa: BLE001
            # If the leader dropped off USB, disconnect() fails too -- it tries
            # to write Torque_Enable to a device that is no longer there. Let
            # that mask the real error and the operator reads a traceback about
            # the cleanup instead of about what actually went wrong.
            print(f"\nleader did not close cleanly: {exc}", file=sys.stderr)
        print("disconnected.")


def _dry_loop(leader, maps: dict[str, JointMap], args) -> int:
    """Leader only. Prints what the follower would be told, and never sends it."""
    period = 1.0 / max(1.0, args.fps)
    names = list(maps)
    w = 7 * len(names)
    print(f"\n{'leader deg':^{w}} | {'follower target deg':^{w}}")
    print("".join(f"{ABBREV[n]:>7}" for n in names) + " | "
          + "".join(f"{ABBREV[n]:>7}" for n in names))
    print("-" * (2 * w + 3))
    deadline = (time.monotonic() + args.seconds
                if args.seconds > 0 else float("inf"))
    n = 0
    clamped: set[str] = set()
    try:
        while time.monotonic() < deadline:
            try:
                action = leader.get_action()
            except Exception as exc:  # noqa: BLE001
                print(f"\n\nLEADER STOPPED ANSWERING after {n} frames: {exc}")
                print(_leader_lost_advice(args))
                return 1
            lead = {j: action[f"{j}.pos"] for j in names}
            targ = {}
            for j in names:
                targ[j] = maps[j].apply(lead[j])
                if abs(maps[j].raw(lead[j]) - targ[j]) > 1e-6:
                    clamped.add(j)
            if n % 5 == 0:
                line = ("".join(f"{lead[j]:>7.1f}" for j in names) + " | "
                        + "".join(f"{targ[j]:>7.1f}" for j in names))
                if sys.stdout.isatty():
                    sys.stdout.write(f"\r\033[2K{line}")
                    sys.stdout.flush()
                else:
                    print(line)
            n += 1
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    print(f"\n\n{n} frames read. Nothing was sent to the follower.")
    if clamped:
        print(f"hit a travel limit while you were moving: "
              f"{', '.join(sorted(clamped))}\n"
              f"That is the clamp working. It means the leader can ask for "
              f"more than the follower has.")
    return 0


def _leader_lost_advice(args) -> str:
    return (
        f"\nThe leader's USB adapter has re-enumerated several times on this\n"
        f"machine. Check whether {args.leader_port} came back with a new\n"
        f"timestamp -- `ls -l {args.leader_port}` -- and if so just run this\n"
        f"again. Nothing is wrong with the arm or the calibration; the\n"
        f"calibration is keyed by --leader-id, not by port, so a renumbered\n"
        f"port does not invalidate it.")


def _freeze(ctrl, motors, args, hold: dict[str, float]) -> int:
    """Hold the arm where it is, then release. See `mb._settle_and_release`.

    This used to be its own loop, which re-ran the guards every pass and let an
    abort cut the hold short -- reasonable when the only caller was a leader
    dropout, wrong once the guards themselves started calling it. The shared
    version also re-targets to the MEASURED position rather than streaming
    `hold`, which is what makes it safe to use after a stall: the commanded
    error that caused the stall goes away, the torque holding the arm up does
    not.

    `hold` is no longer read. It stays in the signature because the call sites
    read better with the thing being held named at the call.

    Note the return value is deliberately DISCARDED by the Ctrl-C path in
    `_live_loop`: a freeze always reports 1, because it is normally reached
    from a fault, but an operator stopping their own session on purpose has
    not failed at anything and `--confirm` should still fire.
    """
    return mb._settle_and_release(ctrl, motors, args, engaged=True)


def _live_loop(leader, ctrl, motors, maps: dict[str, JointMap], args,
               hold: dict[str, float]) -> int:
    """The control loop. Every target is rate-limited, clamped, then guarded."""
    period = 1.0 / max(1.0, args.fps)
    names = list(maps)
    deadline = (time.monotonic() + args.seconds
                if args.seconds > 0 else float("inf"))
    print("".join(f"{ABBREV[n]:>8}" for n in names)
          + f"{'maxerr':>9}{'maxtq':>8}{'maxT':>6}{'clamp':>7}")
    print("-" * (8 * len(names) + 30))

    worst_err = worst_tq = worst_temp = 0.0
    clamped = 0
    # Owned here, not inside the guard: a joint that is legitimately carrying
    # the arm's weight sits above its warn ceiling for the whole session, and
    # the notice belongs once per run rather than 20 times a second.
    warned: set[str] = set()
    i = 0
    t0 = time.monotonic()
    due = t0
    # Ctrl-C is how a teleop session ENDS, not how it fails: there is no other
    # way to stop one. Catching it here rather than letting it reach the
    # caller means the run still prints its summary and still counts as a
    # success, which is what --confirm keys off.
    stopped = False
    # Gravity samples are collected from ORDINARY USE -- see the note above
    # `PITCH_CHAIN` in mb_bringup. Teleop already drives the arm through the
    # loaded poses, under a hand, with the operator choosing where it goes, so
    # there is no reason to make it drive itself through them again to learn
    # what they cost. Opening this append is the entire cost;
    # `_log_gravity_sample` decides per frame whether the frame is usable and
    # is written so it cannot raise into the control loop.
    gfh = None
    if not args.no_gravity_log:
        try:
            gfh = open(mb.GRAVITY_SAMPLES_PATH, "a")
        except OSError as exc:  # noqa: BLE001
            print(f"(not logging gravity samples: {exc})")
    try:
        while time.monotonic() < deadline:
            try:
                action = leader.get_action()
            except Exception as exc:  # noqa: BLE001
                # OBSERVED, not hypothetical: the leader's USB adapter dropped
                # out mid-run on 2026-11-12 and sync_read raised for all seven
                # ids.
                #
                # Letting that propagate would unwind into the `finally` and
                # disable() every motor at once, which is the worst possible
                # response while the arm is holding itself up -- it drops
                # torque on six joints simultaneously and the arm falls. The
                # leader going quiet says nothing at all about the follower,
                # which is still answering and still tracking its last target
                # perfectly well.
                #
                # So freeze: keep streaming the target the arm is already at,
                # under the same guards, and give the operator time to take
                # the weight.
                print(f"\n\nLEADER STOPPED ANSWERING: {exc}")
                print(_leader_lost_advice(args))
                return _freeze(ctrl, motors, args, hold)
            for n in names:
                want = maps[n].apply(action[f"{n}.pos"])
                if abs(maps[n].raw(action[f"{n}.pos"]) - want) > 1e-6:
                    clamped += 1
                # Rate limit per control step. The leader can be yanked; the
                # follower weighs 10 kg and must not try to keep up.
                delta = want - hold[n]
                hold[n] += max(-args.max_step_deg,
                               min(args.max_step_deg, delta))
            mb._command_all(motors, args, hold)

            s = mb._sample_all(ctrl, motors)
            mb._log_gravity_sample(gfh, s, hold)
            err = max((abs(s[n]["pos_deg"] - hold[n])
                       for n in hold if "pos_deg" in s.get(n, {})), default=0.0)
            tq = max((abs(s[n].get("torque", 0.0)) for n in s), default=0.0)
            tp = max((s[n].get("temp", 0.0) for n in s), default=0.0)
            worst_err = max(worst_err, err)
            worst_tq = max(worst_tq, tq)
            worst_temp = max(worst_temp, tp)
            if i % 5 == 0 and sys.stdout.isatty():
                sys.stdout.write(
                    f"\r\033[2K" + "".join(f"{hold[n]:>8.1f}" for n in names)
                    + f"{err:>9.2f}{tq:>8.2f}{tp:>6.0f}{clamped:>7}")
                sys.stdout.flush()
            if mb._guard(ctrl, motors, args, hold, sampled=s, warned=warned):
                # NOT a bare `return`. Returning here unwinds into the caller's
                # `finally`, which disables all six at once -- and on
                # 2026-11-12 that turned a 0.28 N.m overshoot on a joint that
                # was not even being driven into a fall from 143 deg. Stop
                # pushing, hold, announce, then release.
                return _freeze(ctrl, motors, args, hold)
            i += 1
            # Fixed cadence against a running deadline, not `sleep(period)`:
            # `_sample_all` alone costs ~21 ms, so sleeping a full period on
            # top of the loop body would halve the rate the leader is read at.
            # If a pass overruns, skip the sleep rather than accumulating lag.
            due += period
            now = time.monotonic()
            if due < now:
                # Already behind. Forfeit the lost time instead of banking it
                # -- otherwise one slow pass makes the next dozen free-run.
                due = now
            else:
                time.sleep(due - now)
    except KeyboardInterrupt:
        stopped = True
    finally:
        if gfh is not None:
            gfh.close()

    dt = time.monotonic() - t0
    if stopped:
        print("\n\nstopped by operator.")
        # Ctrl-C used to fall straight through to the caller's `finally`,
        # which disables all six -- the same instant drop the fault path was
        # fixed for, reached by the one route the operator takes EVERY time.
        # An arm parked at rest does not care. An arm the operator stopped
        # mid-reach does. So stopping freezes first; a second Ctrl-C, once
        # they have the weight, releases.
        _freeze(ctrl, motors, args, hold)
    print(f"\n{i} frames in {dt:.1f} s ({i / max(dt, 1e-3):.1f} Hz)")
    print(f"worst tracking error {worst_err:.2f} deg, peak torque "
          f"{worst_tq:.2f} N.m, peak {worst_temp:.0f} degC")
    if clamped:
        print(f"{clamped} joint-frames hit a travel limit -- the leader was "
              f"asking for more than the follower has.")
    if gfh is not None and mb.GRAVITY_SAMPLES_PATH.exists():
        n = sum(1 for _ in mb.GRAVITY_SAMPLES_PATH.open())
        have = mb._load_gravity_model()
        print(f"\n{n} gravity samples logged in total"
              + (" -- model is fitted." if have else
                 f", {mb.MIN_FIT_SAMPLES} needed to fit.")
              + "\n  mb_bringup.py --fit-gravity   "
                "(reads the log, moves nothing)")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--leader-port", default="/dev/ttyACM0")
    p.add_argument("--leader-id", default="am_leader",
                   help="lerobot calibration id. Do not re-calibrate.")
    p.add_argument("--port", default="can0", help="follower SocketCAN interface")
    p.add_argument("--host-id", type=lambda s: int(s, 0), default=0xFD)
    p.add_argument("--dry-run", action="store_true",
                   help="leader only. No CAN, no torque, nothing sent.")
    p.add_argument("--preflight", action="store_true",
                   help="read BOTH arms, print the real engage table, the "
                        "direction ledger and every guard, then stop without "
                        "enabling anything. The check to run before all six.")
    p.add_argument("--confirm", nargs="?", const="", default=None,
                   metavar="A,B",
                   help="after a clean run, record these joints as "
                        "direction-verified. Bare --confirm records every "
                        "joint driven this run.")
    p.add_argument("--skip-direction-check", action="store_true",
                   help="drive joints that have never been watched following "
                        "the leader. A reversed joint drives into a stop.")
    p.add_argument("--joints", metavar="A,B",
                   help="drive only these; the rest are held where they are")
    p.add_argument("--invert", default="", metavar="A,B",
                   help="flip the direction of these joints for this run")
    p.add_argument("--align", choices=("pose", "centre"), default="pose",
                   help="pose: zero move to engage (default). "
                        "centre: mid-travel to mid-travel, can be a big move.")
    p.add_argument("--margin-deg", type=float, default=3.0,
                   help="keep this far off each measured endpoint")
    p.add_argument("--max-engage-deg", type=float, default=10.0,
                   help="refuse to start if any joint must move further")
    p.add_argument("--max-step-deg", type=float, default=1.0,
                   help="max degrees any joint may move per control step")
    p.add_argument("--hold-on-fault", type=float, default=-1.0, metavar="SEC",
                   help="on any fault, hold the arm where it is. Default -1 "
                        "FREEZES indefinitely -- until --hold-max-temp or "
                        "Ctrl-C -- because a 5 s countdown expired on an "
                        "extended arm and it fell anyway. A positive number "
                        "restores the fixed window. 0 releases immediately.")
    p.add_argument("--hold-max-temp", type=float, default=mb.STALL_TEMP_C,
                   metavar="DEGC",
                   help="degC at which a freeze gives up and releases. "
                        "Holding is not free: a loaded joint standing still "
                        "dissipates that load continuously.")
    p.add_argument("--fps", type=float, default=20.0)
    p.add_argument("--seconds", type=float, default=0.0, help="0 = until Ctrl-C")
    p.add_argument("--vlim-dps", type=float, default=30.0)
    p.add_argument("--acc-dpss", type=float, default=90.0)
    p.add_argument("--stream-hz", type=float, default=20.0)
    p.add_argument("--abort-deg", type=float, default=15.0)
    p.add_argument("--max-torque", type=float, default=None)
    p.add_argument("--max-temp", type=float, default=mb.STALL_TEMP_C)
    p.add_argument("--no-gravity-model", action="store_true",
                   help="guard on fixed torque ceilings even if a fitted "
                        "gravity_model.json exists")
    p.add_argument("--no-gravity-log", action="store_true",
                   help="do not append pose/torque samples for --fit-gravity")
    p.add_argument("--mode-timeout-ms", type=int, default=1000)
    p.add_argument("--retries", type=int, default=5)
    p.add_argument("--kp", type=float, default=8.0)
    p.add_argument("--kd", type=float, default=1.0)
    p.add_argument("--csp", action="store_true",
                   help="cyclic synchronous position. Smoother for teleop, "
                        "but no acceleration ceiling -- prove POS_VEL first.")
    p.add_argument("--mit", action="store_true",
                   help="MIT impedance. No ceiling of any kind. Opt-in only.")
    args = p.parse_args()
    if args.mit and args.csp:
        p.error("--mit and --csp are different control laws; pick one")
    return teleop(args)


if __name__ == "__main__":
    raise SystemExit(main())
