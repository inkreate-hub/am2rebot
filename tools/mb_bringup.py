#!/usr/bin/env python
"""B601-RS bring-up over Seeed's own MotorBridge SDK. Scan first, move one joint.

    # 0. what does the installed SDK actually expose? No CAN, no arm needed.
    python tools/mb_bringup.py --introspect

    # 1. read-only: who is on the bus, where is each joint? No torque.
    python tools/mb_bringup.py --scan

    # 2. zero from the SIT-DOWN pose. Moves nothing, but redefines every angle.
    python tools/mb_bringup.py --zero

    # 3. first motion: one wrist joint, +-5 deg, velocity-limited
    python tools/mb_bringup.py --move --joint wrist_yaw --amplitude 5

WHY THIS EXISTS ALONGSIDE rs_probe.py / rs_move_one.py

Those drive the arm through lerobot's `RobstrideMotorsBus`, which has one
serious liability documented at length in docs/B601_BRINGUP.md: it encodes
every position as a 16-bit fraction of +-PMAX looked up from a table keyed on
model names `O0`..`O6`, and the mapping from Seeed's RS00/RS06 part numbers onto
those names is *inferred, not documented*. Guess wrong on a shoulder and every
command is off by 8.95x, in the direction that turns a 10 deg request into 90 deg
on a 36 N.m motor.

MotorBridge is Seeed's own path for this arm and takes the model as `"rs-00"` /
`"rs-06"` directly, so that entire ambiguity disappears -- the SDK owns the
scale constant and it is keyed on the real part number.

It also offers POS_VEL, which `RobstrideMotorsBus` does not implement (its
tables.py says "only the MIT protocole is implemented for now"). That matters
more than it sounds. MIT is a bare torque law: the motor applies
kp*(q_target - q) + kd*(0 - qdot) and enforces no velocity or position limit
whatsoever -- it will drive through a hard stop, and a fast multi-joint move can
trip the LRS-600 into hiccup mode, which drops torque on all seven motors at
once and the arm falls. In POS_VEL the *motor* clamps its own velocity and
acceleration. For the first motion the arm has ever made, that is the right
trade: give the limit to the firmware instead of to a Python loop with a
30 ms period.

So: MotorBridge for bring-up and for establishing ground truth, lerobot for
teleop later once the numbers are known. `--mit` is available for comparing the
two paths on one joint.

SAFETY PROPERTIES, DELIBERATE

  * The default action is `--scan`, which never energises anything.
  * `--move` enables exactly ONE motor. Never `enable_all()`.
  * The target starts at the measured present position, so enabling cannot snap.
  * Every command is capped at `--max-step-deg` from the last one.
  * Achieved-vs-commanded motion is checked for the ~9x scale signature anyway.
    The SDK should make it impossible; this is belt and braces, and it is also
    how a wrong CAN id or a wrong model string shows up.
  * Shoulders require typing a word. They are RS06, 36 N.m, and hold the arm up.
  * Torque is dropped in a `finally` on every exit path.

PRECONDITIONS

  * `can0` up at 1 Mbit/s classic CAN -- `tools/can_up.sh can0`. If you do not
    have a can0, run `tools/can_identify.sh` first; not every "USB-CAN" adapter
    can produce one.
  * Arm parked in its sit-down pose. It has no persistent calibration and zeroes
    against whatever pose it is holding.
  * A hand on the DC switch.

See docs/B601_BRINGUP.md for the full staged plan and the gate on each stage.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

#: CAN id per joint, 1..7, from Seeed's B601-RS wiki. Same table as rs_probe.py.
CAN_IDS: dict[str, int] = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_yaw": 5,
    "wrist_roll": 6,
    "gripper": 7,
}

#: The three shoulder joints carry the larger motor (RS06, 36 N.m) and the
#: weight of the arm. Everything else is RS00, 14 N.m.
SHOULDER_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex")

#: MotorBridge takes the real part number, which is the whole point of using it.
#: Still an assertion about the hardware rather than a measurement: Seeed's wiki
#: says J1-J3 are RS06 and J4-J7 are RS00. `--scan` is what confirms it, because
#: a wrong model here no longer corrupts the scale -- it only mislabels a row.
MOTOR_MODELS: dict[str, str] = {
    j: ("rs-06" if j in SHOULDER_JOINTS else "rs-00") for j in CAN_IDS
}

#: Rated continuous torque, N.m, from Seeed's spec.
RATED_TORQUE_NM: dict[str, float] = {"rs-06": 36.0, "rs-00": 14.0}

#: WHY THESE EXIST -- 2026-11-12, and it cost a motor 62 degC.
#:
#: `elbow_flex` was commanded -5 deg from a parked pose, drove into a hard
#: stop at -1.4 deg, and ramped torque 7 -> 14 -> 30 -> 49 -> 58.7 N.m (1.6x
#: its 36 N.m rating) across eight steps while the MOS temperature went
#: 27 -> 62 degC. Then its protection tripped: torque 0.00, position frozen,
#: and the remaining 24 steps logged a motor that was no longer there.
#:
#: Not one guard fired. `--abort-deg` is 15 and the error only reached 5.3.
#: The ratio guard reads `abs(moved) > 0.2 and abs(ratio) < 0.25`, and that
#: leading term EXEMPTS a motor that is not moving -- which is the definition
#: of a stall. The guards were written to catch a position-scale error, and a
#: stall is the opposite failure: perfect scale, no motion, rising torque.
#:
#: So torque and temperature are now watched directly. They are the two
#: signals that move early in a stall -- torque crossed 30 N.m while the
#: motor was still at 27 degC, a full five steps before anything got hot.
#:
#: WHY THERE ARE NOW TWO CEILINGS -- 2026-11-12 evening, and it cost a fall.
#:
#: The single ceiling above was `rated * 0.5` and its abort message said "it is
#: driving into something". During the first elbow teleop run `shoulder_lift`
#: tripped it at 18.28 N.m -- 1.5% over -- while it was NOT BEING DRIVEN AT
#: ALL. It was in the held set, target unchanged, tracking error +0.00. The
#: operator had extended `elbow_flex` to 143.7 deg, which swung the forearm and
#: wrist mass out from the base, and the shoulder's gravity moment rose until
#: standing still cost 18.28 N.m. Nothing was obstructed. The arm was holding
#: itself up, which is its job.
#:
#: The premise under `* 0.5` was "a bounded few-degree bring-up move on an arm
#: that is not carrying a payload". Teleop breaks that premise: the payload is
#: the arm's own outstretched mass, and this geometry genuinely needs more than
#: half of rated at the shoulder. A ceiling below the static weight of the
#: machine is not a safety limit, it is a guarantee of nuisance aborts in
#: exactly the poses where an abort is most expensive.
#:
#: So torque alone no longer decides. A STALL and a GRAVITY HOLD look identical
#: on the torque channel and completely different everywhere else:
#:
#:     signal          stall (morning)        gravity hold (evening)
#:     torque          high and rising        high and steady
#:     position error  5.3 deg and growing    0.00 deg
#:     velocity        ~0 against a command    tracking normally
#:
#: `GRAVITY_TORQUE_FRAC` is now only the threshold at which the OTHER two
#: channels get consulted. Crossing it with a large error and no motion is a
#: stall and aborts. Crossing it while tracking is the arm's weight and warns.
#: `HARD_TORQUE_FRAC` aborts regardless -- nothing legitimate on an unloaded
#: arm needs 85% of rated continuous, and the morning stall would have crossed
#: it at step three, long before 58.7 N.m.
GRAVITY_TORQUE_FRAC = 0.5
HARD_TORQUE_FRAC = 0.85

#: What counts as "not getting there" and "not moving", for the stall test.
#: Teleop's rate-limited target leads the follower by a degree or so at normal
#: speed, so 2.0 sits above the honest lag and well under the 5.3 deg the real
#: stall reached.
STALL_ERR_DEG = 2.0
STALL_VEL_DPS = 2.0

#: Margin over a MEASURED gravity peak when that measurement exists -- see
#: `--gravity-map`. A pose the arm has already held is allowed 25% more before
#: it is treated as suspicious, which covers a payload in the gripper and the
#: difference between a slow sweep and a hand moving at speed.
GRAVITY_MARGIN = 1.25

STALL_TEMP_C = 50.0

#: RobStride's default host/master id. MotorBridge's own scan tries
#: 0xFD, 0xFF, 0xFE, 0x00, 0xAA in that order, so 0xFD is the documented default
#: rather than a guess. Note this is NOT the Damiao-style 0x10+id pairing that
#: lerobot's DM config uses -- carrying that assumption across was a live risk.
DEFAULT_HOST_ID = 0xFD

#: `Mode` values, confirmed by introspection. `ensure_mode` is annotated
#: `Mode | int`, so the bare ints are accepted and no enum lookup is needed.
MODE_MIT = 1
MODE_POS_VEL = 2       # profile position: motor clamps velocity AND accel
MODE_CSP = 5           # ROBSTRIDE_POS_VEL_CSP, cyclic synchronous position


# --- SDK compatibility ---------------------------------------------------
# The wheel is a binding over a Rust ABI and its Python surface is versioned
# independently of the docs. Rather than assume a signature and fail at the
# worst possible moment -- mid-move, with torque on -- these shims probe once
# and are exercised by --introspect before any hardware is involved.

def _require_sdk():
    try:
        import motorbridge  # noqa: F401
    except ImportError:
        print(
            "motorbridge is not installed. Install it into the venv:\n"
            "    uv pip install --python ./lerobot_alohamini/.venv/bin/python "
            "motorbridge\n"
            "The Linux wheel also needs libusb-1.0.so.0 present.",
            file=sys.stderr,
        )
        raise SystemExit(2)
    return __import__("motorbridge")


def _mode_of(args) -> int:
    if args.mit:
        return MODE_MIT
    return MODE_CSP if args.csp else MODE_POS_VEL


def _state_dict(state) -> dict[str, float]:
    """Normalise a feedback object into plain floats in DEGREES.

    The SDK works in radians throughout; this module is the only place the two
    conventions meet, and every number above this line is degrees.
    """
    if state is None:
        return {}

    def pick(*names):
        for n in names:
            if isinstance(state, dict):
                if n in state:
                    return state[n]
            elif hasattr(state, n):
                return getattr(state, n)
        return None

    # Field names are MotorState's, confirmed by introspection against
    # motorbridge 0.3.9:
    #   MotorState(can_id, arbitration_id, status_code, pos, vel, torq,
    #              t_mos, t_rotor)
    # The aliases are kept as a cushion for a future version renaming them.
    pos = pick("pos", "position", "angle")
    vel = pick("vel", "velocity", "speed")
    tau = pick("torq", "torque", "tau", "effort")
    tmp = pick("t_mos", "temp_mos", "temperature", "temp")
    rotor = pick("t_rotor")
    status = pick("status_code")
    out: dict[str, float] = {}
    if pos is not None:
        out["pos_deg"] = math.degrees(float(pos))
    if vel is not None:
        out["vel_dps"] = math.degrees(float(vel))
    if tau is not None:
        out["torque"] = float(tau)
    if tmp is not None:
        out["temp"] = float(tmp)
    if rotor is not None:
        out["temp_rotor"] = float(rotor)
    if status is not None:
        out["status"] = float(status)
    return out


# --- stages --------------------------------------------------------------

def introspect() -> int:
    """Print the SDK's real API surface. Touches no CAN bus and no hardware."""
    mb = _require_sdk()
    print(f"motorbridge {getattr(mb, '__version__', '(no __version__)')}")
    print(f"  module file: {getattr(mb, '__file__', '?')}\n")
    print("top-level names:")
    print("  " + ", ".join(sorted(n for n in dir(mb) if not n.startswith("_"))))

    for cls_name in ("Controller", "Motor", "Mode"):
        cls = getattr(mb, cls_name, None)
        print(f"\n{cls_name}: {'MISSING' if cls is None else cls}")
        if cls is None:
            continue
        members = sorted(n for n in dir(cls) if not n.startswith("_"))
        print("  " + ", ".join(members))
        for meth in ("add_robstride_motor", "ensure_mode", "send_pos_vel",
                     "robstride_send_pos_vel_pp", "robstride_send_pos_vel_csp",
                     "send_mit", "get_state", "set_zero_position",
                     "robstride_ping", "robstride_get_fault_report",
                     "poll_feedback_once"):
            fn = getattr(cls, meth, None)
            if fn is None:
                continue
            try:
                import inspect
                print(f"    {meth}{inspect.signature(fn)}")
            except (TypeError, ValueError):
                print(f"    {meth}(<signature unavailable: native binding>)")
    print("\nCompare against the assumptions in this file's header before "
          "energising anything.")
    return 0


def _link_down_reason(port: str) -> str | None:
    """Why this interface cannot carry traffic, or None if it looks fine.

    Worth a sysfs read before every CAN action, because a down link and a dead
    arm produce the SAME output from a scan -- seven SILENT rows -- and the
    advice for them is completely different. The scan used to answer that with
    a six-item list of possibilities; two of these three cases are facts.
    """
    base = Path("/sys/class/net") / port
    if not base.exists():
        return (f"no interface '{port}'. The USB-CAN adapter is not plugged "
                f"in, or its driver did not bind.\n"
                f"  lsusb | grep 1d50       # 1d50:606f is the candleLight\n"
                f"  ip -brief link show type can")
    try:
        # IFF_UP is bit 0. A CAN link that is administratively down reports
        # `state STOPPED` and counts nothing, which looks exactly like silence.
        if not int(base.joinpath("flags").read_text().strip(), 16) & 1:
            return (f"'{port}' exists but is DOWN. Nothing can be sent or "
                    f"received until it is up:\n"
                    f"    sudo ip link set {port} up type can bitrate 1000000")
    except (OSError, ValueError):
        return None
    return None


def _open(args):
    """Controller + the seven motor handles. Enables nothing."""
    reason = _link_down_reason(args.port)
    if reason:
        print(f"LINK NOT USABLE: {reason}", file=sys.stderr)
        raise SystemExit(2)
    mb = _require_sdk()
    ctrl = mb.Controller(args.port)
    motors = {
        name: ctrl.add_robstride_motor(CAN_IDS[name], args.host_id,
                                       MOTOR_MODELS[name])
        for name in CAN_IDS
    }
    return mb, ctrl, motors


def scan(args) -> int:
    """Stage 1+2 equivalent: who answers, and what do they read? No torque."""
    mb, ctrl, motors = _open(args)
    try:
        _reporting(motors.values(), True)
        time.sleep(0.2)
        print(f"bus {args.port}, host id 0x{args.host_id:02X}, "
              f"active reporting on, no torque enabled\n")
        print(f"{'joint':<15}{'id':>3}{'model':>8}{'pos deg':>10}"
              f"{'vel dps':>9}{'torque':>8}{'temp':>7}  reply")
        print("-" * 72)

        answered = 0
        faults: list[str] = []
        for name in CAN_IDS:
            motor = motors[name]

            # `robstride_ping` is a direct presence query that returns a
            # (value, value) tuple, rather than inferring presence from whether
            # a feedback frame happened to land. A silent motor raises here.
            reachable = False
            try:
                motor.robstride_ping()
                reachable = True
            except Exception:  # noqa: BLE001 - SDK raises its own types
                pass

            s = _state_dict(_poll(motor, args.retries, ctrl))
            if s or reachable:
                answered += 1
                print(f"{name:<15}{CAN_IDS[name]:>3}{MOTOR_MODELS[name]:>8}"
                      f"{s.get('pos_deg', float('nan')):>10.2f}"
                      f"{s.get('vel_dps', float('nan')):>9.2f}"
                      f"{s.get('torque', float('nan')):>8.2f}"
                      f"{s.get('temp', float('nan')):>7.1f}"
                      f"  {'ok' if s else 'ping only'}")
                try:
                    fault, warn = motor.robstride_get_fault_report()
                    if fault or warn:
                        faults.append(f"{name}: fault 0x{fault:X} "
                                      f"warn 0x{warn:X}")
                except Exception:  # noqa: BLE001
                    pass
            else:
                print(f"{name:<15}{CAN_IDS[name]:>3}{MOTOR_MODELS[name]:>8}"
                      f"{'-':>10}{'-':>9}{'-':>8}{'-':>7}  SILENT")

        if faults:
            print("\nFAULTS REPORTED -- clear these before energising "
                  "(--clear-faults), and fix the cause first:")
            for f in faults:
                print(f"  {f}")

        print(f"\n{answered}/7 answered.")
        if answered == 7 and faults:
            print("GATE NOT PASSED: all seven are present but at least one "
                  "reports a fault. Find out why before applying torque.")
            return 1
        if answered == 7:
            print("GATE PASSED. Positions should be stable and repeatable -- "
                  "run this twice without touching the arm and compare.\n"
                  "Next: park in the sit-down pose, then --zero.")
            return 0
        if answered == 0:
            print("Nothing answered at all. In order of likelihood:\n"
                  f"  - the link is ERROR-PASSIVE from an earlier failed run "
                  f"and its TX queue is jammed. THIS REPORTS A HEALTHY BUS AS "
                  f"DEAD, and the state survives until the link is bounced:\n"
                  f"        sudo ip link set {args.port} down && \\\n"
                  f"        sudo ip link set {args.port} up type can "
                  f"bitrate 1000000\n"
                  f"    Check first:  ip -s -d link show {args.port}\n"
                  "  - can0 is up but not actually wired to the arm, or the "
                  "120 ohm termination is missing\n"
                  "  - arm not powered (48 V)\n"
                  "  - motors are in RobStride's private protocol rather than "
                  "MIT comm mode\n"
                  "  - wrong host id: try --host-id 0xFF / 0xFE / 0x00 / 0xAA\n"
                  f"Confirm frames exist at all with:  candump -tz {args.port}")
            return 1
        print("PARTIAL BUS -- do not proceed. A missing shoulder motor means "
              "the arm is held by six and dropped by one. Find the silent "
              "joint's wiring first.\n"
              "Run --sweep to tell an id problem apart from an unreachable "
              "motor; they need opposite fixes.")
        return 1
    finally:
        _reporting(motors.values(), False)
        _shutdown(ctrl)


#: Where `--ranges` writes and `--move` reads. Tracked in git: the travel of
#: this arm is a hardware fact, and the whole point is that it survives the
#: session that measured it.
RANGES_PATH = Path(__file__).resolve().parent.parent / "calibration" \
    / "b601_ranges.json"

#: Keep commanded motion this far inside measured travel. The measurement is
#: hand-driven, so its endpoints are wherever the joint stopped feeling free --
#: not the true mechanical stop, which is a little further and much harder.
RANGE_MARGIN_DEG = 3.0


def ranges(args) -> int:
    """Record each joint's travel BY HAND. Never enables torque.

    This is the gate that was skipped on 2026-11-12, and skipping it cost
    `elbow_flex` 58.7 N.m into a hard stop. From the parked pose that joint had
    1.4 degrees of negative travel; it was commanded 5.

    The measurement costs nothing and risks nothing: torque stays off, you move
    the arm, and the encoders report where it went. Run it, move every joint
    slowly through its full comfortable travel in both directions, Ctrl-C.

        python tools/mb_bringup.py --ranges                      # all seven
        python tools/mb_bringup.py --ranges --joint elbow_flex   # just one

    One at a time is the better way to do it. Sweeping all seven at once means
    watching seven rows and hoping none was missed, and on 2026-11-12 two were
    -- `shoulder_lift` and `elbow_flex` came out under a degree and silently
    did not make it into the file. With `--joint` there is one number on screen
    and it is either growing or it is not.

    A `--joint` run REPLACES that joint's entry and leaves the other six alone,
    so the file fills up across runs. A full run owns all seven.

    What comes out is a floor under `--move`, which afterwards refuses to plan
    a target outside measured travel.
    """
    mb, ctrl, motors = _open(args)
    # Which joints this run is responsible for. Everything outside the scope
    # keeps whatever the file already says about it; everything inside is
    # rewritten from what we measure here, including being dropped if it turns
    # out thin. A stale wide range is the dangerous direction -- it is what
    # lets `--move` plan into a stop -- so a joint we just watched not move
    # must not keep an old measurement saying it has room.
    scope = [args.joint] if args.joint else list(CAN_IDS)
    motors = {n: m for n, m in motors.items() if n in scope}
    try:
        _reporting(motors.values(), True)
        time.sleep(0.3)
        what = f"'{args.joint}'" if args.joint else "each joint"
        print(f"TORQUE IS OFF AND STAYS OFF. Move {what} by hand, slowly, "
              f"through its full travel both ways.\nCtrl-C when done.\n")

        lo: dict[str, float] = {}
        hi: dict[str, float] = {}
        now: dict[str, float] = {}
        samples = 0
        next_save = 0.0
        next_print = 0.0
        tty = sys.stdout.isatty()

        # SAMPLE ALL SEVEN PER PASS, not one at a time. `_poll` sleeps 50 ms
        # per motor while it waits on a request/response round trip, which for
        # seven joints is ~3 Hz -- slow enough to sail straight past the
        # endpoint of a hand-driven sweep and record the travel SHORT. Short
        # travel is the dangerous direction: it is what lets `--move` plan into
        # a stop. Active reporting is already on and every motor is streaming
        # at ~10 Hz, so draining the socket and reading all seven cached states
        # costs one pass and needs no request at all.
        print(f"{'joint':<15}{'now':>9}{'min':>9}{'max':>9}{'travel':>9}")
        print("-" * 51)
        try:
            while True:
                for name, s in _sample_all(ctrl, motors).items():
                    if "pos_deg" not in s:
                        continue
                    p = s["pos_deg"]
                    now[name] = p
                    lo[name] = min(lo.get(name, p), p)
                    hi[name] = max(hi.get(name, p), p)
                samples += 1

                if time.monotonic() >= next_save:
                    _write_ranges(lo, hi, scope)
                    next_save = time.monotonic() + 2.0

                if tty:
                    # Redraw in place. The final row carries NO newline: with
                    # the cursor on the last line of the window, writing one
                    # scrolls the buffer, and the next `\033[nA` then lands a
                    # line short. That drift is what smeared this table into
                    # four rows repeating while the last three sat below it.
                    # So move up N-1, return to column 0, and leave the cursor
                    # parked at the end of the last row.
                    if samples > 1:
                        # `\033[0A` is NOT a no-op -- ECMA-48 reads a zero
                        # parameter as 1, so a single-joint sweep would walk
                        # up the screen one line per pass. Emit the cursor-up
                        # only when there is actually a row to go up to.
                        up = f"\033[{len(scope) - 1}A" if len(scope) > 1 else ""
                        sys.stdout.write(f"{up}\r")
                    rows = (f"\033[2K{_range_row(n, now, lo, hi)}"
                            for n in scope)
                    sys.stdout.write("\n".join(rows))
                    sys.stdout.flush()
                elif time.monotonic() >= next_print:
                    # Piped or captured: cursor movement does not collapse
                    # anything, it just interleaves. Print a plain block now
                    # and then instead of smearing one every pass.
                    print(f"[{samples} samples]")
                    for name in scope:
                        print(f"  {_range_row(name, now, lo, hi)}")
                    next_print = time.monotonic() + 5.0
        except KeyboardInterrupt:
            print(f"\n\n{samples} samples.\n")

        # The data is already on disk by now -- the loop has been saving every
        # two seconds. Everything below is reporting, so a second Ctrl-C
        # through here costs nothing. That was not true before: the file used
        # to be written once, at the end, and an interrupt anywhere in this
        # block threw away the entire sweep.
        thin = _write_ranges(lo, hi, scope)
        if not lo:
            print("Nothing was recorded.", file=sys.stderr)
            return 1

        print(f"{'joint':<15}{'min':>9}{'max':>9}{'travel':>9}")
        print("-" * 42)
        for name in scope:
            if name not in lo:
                continue
            print(f"{name:<15}{lo[name]:>9.2f}{hi[name]:>9.2f}"
                  f"{hi[name] - lo[name]:>9.2f}")

        recorded = _load_ranges()
        if not recorded:
            print("\nNOTHING WAS MEASURED -- nothing moved more than 1 deg.\n"
                  "No file written, because a range nobody measured is worse "
                  "than no range at all.\nRun it again and move the arm.",
                  file=sys.stderr)
            return 1

        print(f"\nwrote {RANGES_PATH} "
              f"({len(recorded)}/{len(CAN_IDS)} joints on file)")

        if thin:
            print(f"\n!! NOT recorded, moved under 1 deg: {', '.join(thin)}\n"
                  f"   --move will refuse these until you sweep them.")
        missing = [n for n in CAN_IDS if n not in recorded]
        if missing:
            print(f"\nstill unmeasured: {', '.join(missing)}\n"
                  f"   python tools/mb_bringup.py --ranges "
                  f"--joint {missing[0]}")
        return 0
    finally:
        _reporting(motors.values(), False)
        _shutdown(ctrl)


#: Pause between re-arming reporting and draining the replies. Measured over
#: 3 s windows with the arm powered and idle -- passes/s against the fraction
#: of those passes that returned a NEW reading:
#:
#:      5 ms  159 Hz  30% fresh      30 ms   32 Hz  65% fresh
#:     10 ms   89 Hz  43% fresh      50 ms   20 Hz  80% fresh
#:     20 ms   47 Hz  62% fresh
#:
#: Polling harder buys less than it looks: at 5 ms most passes re-read a frame
#: that has not changed. 20 ms is near the peak of new samples per second at a
#: fraction of the bus traffic -- roughly 8% load, against ~18% at 10 ms.
SAMPLE_GAP_S = 0.02


def _range_row(name: str, now: dict, lo: dict, hi: dict) -> str:
    """One line of the live sweep table."""
    if name not in now:
        return f"{name:<15}{'no data':>9}"
    span = hi[name] - lo[name]
    mark = " <-" if span >= 2 * RANGE_MARGIN_DEG else ""
    return (f"{name:<15}{now[name]:>9.2f}{lo[name]:>9.2f}"
            f"{hi[name]:>9.2f}{span:>9.2f}{mark}")


def _sample_all(ctrl, motors, gap: float = SAMPLE_GAP_S) -> dict[str, dict]:
    """One fresh reading from every motor. Degrees.

    Re-arms reporting first, because `robstride_set_active_report` is one-shot
    -- see `_reporting`. Without that re-arm this returns the same frozen
    values forever, which is how a ten-minute range sweep recorded 0.00 travel
    on all seven joints while the arm was visibly being moved by hand.

    Re-arming all seven and then draining once costs one `gap` per pass rather
    than one per joint, which is the difference between ~29 Hz and ~3 Hz.
    """
    for motor in motors.values():
        try:
            motor.robstride_set_active_report(True)
        except Exception:  # noqa: BLE001
            pass
    time.sleep(gap)
    for _ in range(24):
        try:
            ctrl.poll_feedback_once()
        except Exception:  # noqa: BLE001
            break
    out = {}
    for name, motor in motors.items():
        try:
            s = _state_dict(motor.get_state())
        except Exception:  # noqa: BLE001
            continue
        if s:
            out[name] = s
    return out


# --------------------------------------------------------------------------
# Flight recorder
#
# WHY -- the arm has fallen three times and each diagnosis was reconstructed
# afterwards from whatever happened to be on screen. That worked because the
# causes were in the software and the software was printing. It will not work
# for the remaining ones, which are electrical and silent: a PSU hiccup, a
# motor's own protection latching, a rail dip. Those drop torque on the way
# down and leave nothing behind.
#
# So every frame is written: what the leader asked for, what was commanded,
# and what all seven motors reported back -- position, velocity, torque,
# temperature and the STATUS WORD, which is the one channel that distinguishes
# "we gave up" from "the motor gave up".
#
# It records through the freeze as well. That is deliberate and it is the whole
# point: the last fall happened at the END of a hold, so a recorder that stops
# at the abort would capture everything except the event.
#
# CSV rather than JSONL because the postmortem reads columns, and a column of
# torques across seven joints is the shape of the question being asked. ~45
# columns at 20 Hz is about 2 MB an hour.
# --------------------------------------------------------------------------

#: Per-joint columns, in order. `st` is the status word.
REC_FIELDS = ("lead", "tgt", "pos", "vel", "tq", "tmp", "st")


class FlightRecorder:
    """Append-only per-frame log. Never raises into the control loop."""

    def __init__(self, path: Path, joints: list[str]) -> None:
        self.path = path
        self.joints = joints
        self.frame = 0
        self.t0 = time.monotonic()
        self._pending: list[str] = []
        self.fh = None
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            self.fh = path.open("w")
            cols = ["t", "frame", "event"]
            for j in joints:
                cols += [f"{j}_{f}" for f in REC_FIELDS]
            self.fh.write(",".join(cols) + "\n")
        except OSError:
            self.fh = None

    def event(self, tag: str) -> None:
        """Tag the NEXT frame. Several tags in one frame are joined by ';'.

        Tagging the next frame rather than writing a row of its own keeps one
        row per control pass, so the postmortem can index by frame without
        worrying about which rows carry motor data.
        """
        self._pending.append(tag.replace(",", ";"))

    def write(self, sampled: dict, targets: dict, leader: dict) -> None:
        if self.fh is None:
            return
        row = [f"{time.monotonic() - self.t0:.3f}", str(self.frame),
               ";".join(self._pending)]
        self._pending.clear()
        for j in self.joints:
            st = sampled.get(j) or {}
            row += [
                _fmt(leader.get(j)), _fmt(targets.get(j)),
                _fmt(st.get("pos_deg")), _fmt(st.get("vel_dps")),
                _fmt(st.get("torque")), _fmt(st.get("temp")),
                # Blank, not 0: a motor that did not answer is a different
                # fact from a motor reporting status 0, and conflating them
                # is exactly how a dropped joint hides in a log.
                "" if st.get("status") is None else f"{int(st['status'])}",
            ]
        self.frame += 1
        try:
            self.fh.write(",".join(row) + "\n")
        except (OSError, ValueError):
            pass

    def close(self) -> None:
        if self.fh is not None:
            try:
                self.fh.flush()
                self.fh.close()
            except OSError:
                pass
            self.fh = None


def _fmt(v) -> str:
    return "" if v is None or not math.isfinite(float(v)) else f"{float(v):.3f}"


#: The live recorder, if any. A module global because `_settle_and_release`
#: lives here and the teleop loop lives in mb_teleop, and both have to write to
#: the same file. Passing it through every signature would mean touching every
#: call site to carry something that is unambiguously one-per-process.
REC: FlightRecorder | None = None


def _rec_event(tag: str) -> None:
    if REC is not None:
        REC.event(tag)


def _write_ranges(lo: dict[str, float], hi: dict[str, float],
                  scope: list[str]) -> list[str]:
    """Persist travel measured so far. Returns the joints too thin to record.

    CALLED FROM INSIDE THE SAMPLING LOOP, not only at the end. A sweep is ten
    minutes of somebody moving an arm by hand, and on 2026-11-12 a whole one
    was lost because the file was written once, after the loop: the process
    was interrupted somewhere in the summary path and every sample went with
    it. Writing continuously means an interrupt at any instant costs at most
    the last couple of seconds.

    `scope` is what this run measured. Joints outside it keep what the file
    already says, so a one-joint sweep tops the file up instead of replacing
    it. Joints inside it are rewritten from this run alone -- including being
    DROPPED if they came out thin. That asymmetry is deliberate: a leftover
    wide range on a joint we just watched sit still is exactly the input that
    lets `--move` plan into a hard stop.

    Joints that moved under a degree are omitted rather than stored as a
    zero-width range -- see the comment in `ranges`.
    """
    out = {n: v for n, v in _load_ranges().items() if n not in scope}
    thin = []
    for name in scope:
        if name not in lo:
            continue
        span = hi[name] - lo[name]
        if span < 1.0:
            thin.append(name)
            continue
        out[name] = {"min_deg": round(lo[name], 2),
                     "max_deg": round(hi[name], 2),
                     "travel_deg": round(span, 2)}
    if not out:
        return thin
    out = {n: out[n] for n in CAN_IDS if n in out}
    RANGES_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = RANGES_PATH.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(
        {"recorded": time.strftime("%Y-%m-%dT%H:%M:%S"),
         "note": "hand-driven, torque off. Endpoints are where the joint "
                 "stopped feeling free, not the true mechanical stop.",
         "joints": out}, indent=2) + "\n")
    # Atomic, so an interrupt mid-write cannot leave a half-parsed file
    # standing where `--move` expects a measurement.
    tmp.replace(RANGES_PATH)
    return thin


def _load_ranges() -> dict[str, dict[str, float]]:
    if not RANGES_PATH.exists():
        return {}
    try:
        return json.loads(RANGES_PATH.read_text()).get("joints", {})
    except (OSError, ValueError):
        return {}


#: Peak torque each joint was MEASURED holding, per `--gravity-map`. A property
#: of this arm's mass and geometry, so it belongs beside the travel ranges
#: rather than in `recordings/`.
GRAVITY_PATH = RANGES_PATH.parent / "gravity_torque.json"


def _load_gravity() -> dict[str, dict[str, float]]:
    if not GRAVITY_PATH.exists():
        return {}
    try:
        return json.loads(GRAVITY_PATH.read_text()).get("joints", {})
    except (OSError, ValueError):
        return {}


def _ceilings(args) -> dict[str, tuple[float, float, float]]:
    """`(stall, warn, hard)` torque ceiling per joint, N.m.

    Three numbers because they answer three different questions, and an early
    version of this collapsed two of them and created a blind spot:

    `stall` -- above this, torque accompanied by a large tracking error and no
    motion is a stall and aborts. HALF OF RATED, FIXED. Deliberately NOT raised
    by the gravity map: a gravity load appears with zero error by definition,
    so a measurement of what the arm can hold says nothing whatever about what
    is acceptable while FAILING to reach a target. Raising this with the
    measurement is what the first version did, and a mapped shoulder then sat
    blind to a genuine stall anywhere between 18 and 26 N.m.

    `warn` -- above this, high torque is worth mentioning once. Raised to cover
    any load this arm has actually been measured carrying, because the whole
    point of the map is to stop narrating the machine's own weight back at the
    operator as though it were news.

    `hard` -- above this, abort, no questions, no discriminator. Nothing on an
    unloaded arm needs 85% of rated continuous. Never raised by anything: a
    measurement claiming a joint needs that much to stand still is evidence
    something is wrong with the arm, not a licence to allow it.

    An explicit `--max-torque` overrides all three. The operator has said a
    number; second-guessing it with a hidden higher one would be worse than
    obeying it.
    """
    measured = _load_gravity()
    out: dict[str, tuple[float, float, float]] = {}
    for name, model in MOTOR_MODELS.items():
        rated = RATED_TORQUE_NM[model]
        hard = rated * HARD_TORQUE_FRAC
        stall = rated * GRAVITY_TORQUE_FRAC
        warn = stall
        peak = measured.get(name, {}).get("peak_nm")
        if peak is not None:
            warn = max(warn, abs(float(peak)) * GRAVITY_MARGIN)
        if args.max_torque is not None:
            stall = warn = hard = args.max_torque
        out[name] = (min(stall, hard), min(warn, hard), hard)
    return out


# --------------------------------------------------------------------------
# Pose-aware gravity model
#
# WHY A SCALAR CEILING CANNOT BE MADE CORRECT -- 2026-11-12, after the fall.
#
# The question that forced this was the operator's: why was it about to drop
# when there was no payload and the arm was only extended? The answer is that
# there IS a payload and it is the arm. At `lift 136.7 / elbow 143.7` the
# shoulder measured 19.28 N.m holding nothing but its own forearm, wrist and
# gripper at full lever arm. The rs-06 is rated 36. The motor was at 54% of
# rated and in no difficulty whatever. The ceiling was at 50%, so the SOFTWARE
# was the thing in difficulty.
#
# Raising the number does not fix the shape of the problem. Gravity torque at
# the shoulder is a function of pose, and over this arm's travel it runs from
# near zero (folded, links stacked over the axis) to its maximum (extended,
# horizontal). Any single number must sit either above that maximum or below
# it, and both choices are bad in a specific way:
#
#     ceiling above the max   never false-aborts, and is BLIND in the folded
#                             poses -- a joint can drive into the bench at
#                             17 N.m with gravity explaining 2 of it, and
#                             nothing fires, because 17 < 18.
#     ceiling below the max   catches that, and false-aborts every time the
#                             operator extends the arm. This is what dropped it.
#
# The fix is to stop asking "is this torque large" and start asking "is this
# torque MORE THAN THIS POSE EXPLAINS". Gravity load is predictable from the
# joint angles; an obstruction, a collision or a payload is not. The difference
# between measured and predicted -- the residual -- is the part that gravity
# does not account for, and that is the only part worth guarding.
#
# THE MODEL. For a planar serial chain the gravity torque at joint k is a sum
# over every link outboard of it:
#
#     tau_k = SUM_{j >= k} [ alpha_j*cos(THETA_j) + beta_j*sin(THETA_j) ] + c
#
# where THETA_j is the CUMULATIVE angle from the base to link j. The alphas and
# betas absorb the link masses, the centre-of-mass distances, g, and -- this is
# why the sin terms are there -- the arbitrary phase of each joint's zero. The
# arm has no persistent calibration and is zeroed against whatever pose it is
# parked in, so no angle here can be assumed to start from horizontal. A free
# constant `c` absorbs any steady sensor bias.
#
# That expression is LINEAR in its coefficients, which is the whole reason this
# is tractable without a dynamics library: ordinary least squares, solved from
# the normal equations, in a few dozen lines of arithmetic below.
#
# WHERE THE DATA COMES FROM, and why it is not another autonomous sweep. The
# first instinct was a commanded 2D grid over (lift, elbow). That means the arm
# driving itself through its most heavily loaded poses to learn what they cost,
# which is the one thing recent history argues against. Teleop already visits
# those poses, under a hand, with the operator deciding where it goes. So the
# samples are LOGGED FROM ORDINARY USE -- `_log_gravity_sample`, called from the
# teleop loop whenever the arm is near-stationary and tracking -- and the fit
# runs offline afterwards. Nothing moves to collect them, the coverage improves
# every session, and the model is always fitted to how this arm is actually
# used rather than to a grid someone guessed at.
# --------------------------------------------------------------------------

#: Joints whose angles accumulate into the vertical plane that gravity acts in,
#: ordered base-outward. `shoulder_pan` is deliberately absent: it rotates about
#: a VERTICAL axis, so gravity exerts no moment about it and a model would be
#: fitting noise. `wrist_yaw`, `wrist_roll` and `gripper` are absent for the
#: same reason or because the mass outboard of them is negligible.
PITCH_CHAIN: tuple[str, ...] = ("shoulder_lift", "elbow_flex", "wrist_flex")

#: Smallest residual worth calling abnormal, N.m, per motor model. A floor under
#: whatever the fit's own scatter suggests -- without it, a very clean fit on a
#: cold arm would set a threshold so tight that friction and stiction alone
#: trip it.
MIN_RESIDUAL_NM: dict[str, float] = {"rs-06": 3.0, "rs-00": 1.5}

#: How many times the fit's RMS residual counts as "more than this pose
#: explains". 4 sigma on a model with hundreds of samples is a long way out.
RESIDUAL_SIGMA = 4.0

#: Refuse to fit from fewer than this many samples per joint. The widest model
#: (`shoulder_lift`) has 7 free parameters; fitting 7 parameters to 20 points
#: produces a confident-looking model of nothing.
MIN_FIT_SAMPLES = 150

#: ...and refuse if the poses are all alike. A model fitted to one corner of the
#: workspace extrapolates catastrophically into the rest of it.
MIN_FIT_SPAN_DEG = 40.0

GRAVITY_MODEL_PATH = RANGES_PATH.parent / "gravity_model.json"
GRAVITY_SAMPLES_PATH = RANGES_PATH.parent / "gravity_samples.jsonl"


def _chain_angles(pose: dict[str, float]) -> list[float] | None:
    """Cumulative chain angles, radians. `None` if any link angle is missing.

    Cumulative because link 2's attitude in the world depends on where links 0
    and 1 put it, and it is world attitude that gravity cares about.
    """
    out: list[float] = []
    total = 0.0
    for name in PITCH_CHAIN:
        a = pose.get(name)
        if a is None or not math.isfinite(a):
            return None
        total += math.radians(a)
        out.append(total)
    return out


def _features(name: str, pose: dict[str, float]) -> list[float] | None:
    """Regressors for `name` at `pose`, or `None` if it has no model.

    A joint only sees the links OUTBOARD of it -- the mass between it and the
    base hangs off the base, not off this motor -- so the feature vector gets
    shorter the further out the joint is. `shoulder_lift` gets 7 terms,
    `elbow_flex` 5, `wrist_flex` 3.
    """
    if name not in PITCH_CHAIN:
        return None
    angles = _chain_angles(pose)
    if angles is None:
        return None
    k = PITCH_CHAIN.index(name)
    feats: list[float] = []
    for th in angles[k:]:
        feats.append(math.cos(th))
        feats.append(math.sin(th))
    feats.append(1.0)
    return feats


def _solve_ridge(rows: list[list[float]], ys: list[float],
                 lam: float = 1e-6) -> list[float] | None:
    """Least-squares fit of `rows @ x = ys`, by normal equations.

    `lam` is a tiny ridge term on the diagonal. It is not there to regularise
    in the statistical sense -- it is there so that a rank-deficient design
    matrix (the operator moved only one joint all session) produces a damped
    answer instead of a division by something near zero. The caller still has
    to check coverage; this only stops the arithmetic from exploding.

    Pure Python on purpose: this file's whole dependency surface is motorbridge
    and the standard library, and a 7x7 solve does not justify changing that.
    """
    n = len(rows[0])
    ata = [[0.0] * (n + 1) for _ in range(n)]
    for row, y in zip(rows, ys):
        for i in range(n):
            ri = row[i]
            for j in range(n):
                ata[i][j] += ri * row[j]
            ata[i][n] += ri * y
    for i in range(n):
        ata[i][i] += lam

    for col in range(n):                       # Gauss, partial pivoting.
        piv = max(range(col, n), key=lambda r: abs(ata[r][col]))
        if abs(ata[piv][col]) < 1e-12:
            return None
        ata[col], ata[piv] = ata[piv], ata[col]
        pv = ata[col][col]
        for j in range(col, n + 1):
            ata[col][j] /= pv
        for r in range(n):
            if r == col:
                continue
            f = ata[r][col]
            if f:
                for j in range(col, n + 1):
                    ata[r][j] -= f * ata[col][j]
    return [ata[i][n] for i in range(n)]


def _load_gravity_model() -> dict[str, dict]:
    if not GRAVITY_MODEL_PATH.exists():
        return {}
    try:
        return json.loads(GRAVITY_MODEL_PATH.read_text()).get("joints", {})
    except (OSError, ValueError):
        return {}


#: `_guard` runs at the stream rate. Re-reading and re-parsing the model file
#: 20 times a second to get the same answer would be the single most expensive
#: thing in the loop, so it is read once per process.
_GMODEL_CACHE: dict[str, dict] | None = None


def _gmodel() -> dict[str, dict]:
    global _GMODEL_CACHE
    if _GMODEL_CACHE is None:
        _GMODEL_CACHE = _load_gravity_model()
    return _GMODEL_CACHE


def _expected_torque(name: str, pose: dict[str, float],
                     gmodel: dict[str, dict]) -> float | None:
    """Torque this pose explains, N.m, or `None` if it cannot be predicted."""
    entry = gmodel.get(name)
    if not entry:
        return None
    feats = _features(name, pose)
    coef = entry.get("coef")
    if feats is None or not coef or len(coef) != len(feats):
        return None
    return sum(c * f for c, f in zip(coef, feats))


def _residual_limit(name: str, gmodel: dict[str, dict]) -> float | None:
    entry = gmodel.get(name)
    if not entry:
        return None
    rms = float(entry.get("resid_rms", 0.0))
    return max(MIN_RESIDUAL_NM[MOTOR_MODELS[name]], RESIDUAL_SIGMA * rms)


def _log_gravity_sample(fh, sampled: dict[str, dict],
                        targets: dict[str, float]) -> None:
    """Append one settled sample, if this frame qualifies. Never raises.

    QUALIFYING MATTERS MORE THAN QUANTITY. The model describes STATIC gravity
    load, so a frame is only usable when the arm is near-stationary -- a moving
    link adds inertial torque the model has no term for, and feeding those in
    biases every coefficient. Requiring the joint to be tracking does the same
    job for the other direction: a frame taken mid-stall would teach the model
    that an obstruction is normal for that pose, which is precisely the thing
    it exists to notice.

    Never raises because this runs inside the teleop control loop. A full disk
    is not a reason to drop an arm.
    """
    if fh is None:
        return
    pose = {n: st.get("pos_deg") for n, st in sampled.items()}
    if _chain_angles(pose) is None:
        return
    rec: dict[str, float] = {}
    for name in PITCH_CHAIN:
        st = sampled.get(name)
        if not st:
            return
        vel = st.get("vel_dps", float("nan"))
        if not math.isfinite(vel) or abs(vel) > STALL_VEL_DPS:
            return                              # moving: inertia, not gravity
        if name in targets:
            pos = st.get("pos_deg", float("nan"))
            if (math.isfinite(pos)
                    and abs(pos - targets[name]) > STALL_ERR_DEG):
                return                          # not tracking: possibly stalled
    for name in PITCH_CHAIN:
        st = sampled[name]
        rec[name] = round(float(st["pos_deg"]), 3)
        rec[name + "_tq"] = round(float(st.get("torque", 0.0)), 3)
    try:
        fh.write(json.dumps(rec) + "\n")
    except (OSError, ValueError, TypeError):
        pass


def clear_faults(args) -> int:
    """Clear latched faults on all seven. Enables no torque, commands no move.

    `--scan` says "clear these before energising" and used to offer no way to
    do it, which left the only documented recovery as a power cycle -- and a
    power cycle also silently discards the zero, since `set_zero_position()`
    is RAM-only until `store_parameters()`. Two unrelated things lost to one
    blunt instrument.

    Clearing a fault does not fix what caused it. Read the fault word first.
    """
    mb, ctrl, motors = _open(args)
    try:
        print("Reading faults before clearing. A fault on a motor that was "
              "never energised points at the shared 48 V rail, not that "
              "motor.\n")
        before: dict[str, tuple[int, int]] = {}
        for name in CAN_IDS:
            try:
                before[name] = motors[name].robstride_get_fault_report()
            except Exception:  # noqa: BLE001
                print(f"  {name:<15} unreadable")
        for name, (f, w) in before.items():
            if f or w:
                print(f"  {name:<15} fault 0x{f:X}  warn 0x{w:X}")
        if not any(f or w for f, w in before.values()):
            print("  nothing latched. Nothing to clear.")
            return 0

        for name in CAN_IDS:
            try:
                motors[name].clear_error()
                time.sleep(0.05)
            except Exception as exc:  # noqa: BLE001
                print(f"  {name:<15} clear failed: {exc}")

        time.sleep(0.2)
        still: list[str] = []
        for name in CAN_IDS:
            try:
                f, w = motors[name].robstride_get_fault_report()
                if f or w:
                    still.append(f"{name} (fault 0x{f:X} warn 0x{w:X})")
            except Exception:  # noqa: BLE001
                pass
        if still:
            print(f"\nSTILL LATCHED: {', '.join(still)}\n"
                  "A fault that will not clear is still present -- the motor "
                  "is over temperature, or the rail is still out of range.")
            return 1
        print("\nAll clear. Re-run --scan to confirm, and fix the cause "
              "before energising anything.")
        return 0
    finally:
        _shutdown(ctrl)


def zero(args) -> int:
    """Redefine every joint's zero as its present pose. Commands no motion."""
    print("This redefines the zero of all seven joints to the CURRENT pose.\n"
          "The arm must be parked in its sit-down pose. Every angle in the "
          "mapping, and every joint limit, is measured from here.")
    if input("type 'zero' to continue: ").strip() != "zero":
        return 1

    mb, ctrl, motors = _open(args)
    try:
        for name in CAN_IDS:
            motors[name].set_zero_position()
            time.sleep(0.05)
            print(f"  {name:<15} zeroed")

        # RECORDED TRAVEL LIVES IN THE OLD ZERO'S FRAME, so this just
        # invalidated it. Stale ranges are worse than none: every min and max
        # is now offset by however far the arm was from the old zero, and
        # `--move` would use them to decide a 36 N.m motor has room. Keep the
        # file for reference, but get it out from under `_load_ranges`.
        if RANGES_PATH.exists():
            stale = RANGES_PATH.with_suffix(
                f".stale-{time.strftime('%Y%m%d_%H%M%S')}.json")
            RANGES_PATH.rename(stale)
            print(f"\n!! zeroing moved the frame the recorded travel was "
                  f"measured in.\n   Retired it to {stale.name} -- re-run "
                  f"--ranges before --move.")

        # NOT RETIRED HERE, AND IT SHOULD BE -- 2026-11-13. GRAVITY_MODEL_PATH
        # and GRAVITY_SAMPLES_PATH are pose-keyed exactly like RANGES_PATH
        # above, but nothing moves them aside. A session run after this zero
        # appends fresh samples onto old ones measured in the retired frame,
        # and `--fit-gravity` cannot tell the difference -- it fits one model
        # out of two arms' worth of angles for the same real pose. Clear both
        # files by hand after re-zeroing until this is automated the same way.
        print("\nRe-run --scan: every joint should now read within a degree "
              "of 0.")
        return 0
    finally:
        _shutdown(ctrl)


def move_one(args) -> int:
    """First torque. One joint, bounded steps, velocity-limited by the motor."""
    name = args.joint
    # Not on a dry run: it enables nothing, and a confirmation prompt in front
    # of the one mode that is safe teaches people to type through the prompt.
    if name in SHOULDER_JOINTS and not args.dry_run:
        print(f"!! {name} is a shoulder joint: RS06, 36 N.m, and it carries "
              f"the arm's weight.\n"
              f"   Do a wrist joint first. Support the arm so nothing falls "
              f"if torque cuts.")
        if input("type 'shoulder' to continue: ").strip() != "shoulder":
            return 1

    mb, ctrl, motors = _open(args)
    motor = motors[name]
    engaged = False
    try:
        # Without this the motor never streams state, and every readback below
        # -- including the abort checks -- would be blind. See `_reporting`.
        _reporting([motor], True)
        time.sleep(0.2)
        state = _state_dict(_poll(motor, args.retries, ctrl))
        if "pos_deg" not in state:
            print(f"{name} (id {CAN_IDS[name]}) is not answering. Run --scan.",
                  file=sys.stderr)
            return 1

        home = state["pos_deg"]
        model = MOTOR_MODELS[name]
        rated = RATED_TORQUE_NM[model]
        # The warn ceiling, not the hard one: `--move` is a bounded few-degree
        # step on one joint, so anything above the gravity envelope here really
        # is suspicious. It tracks `--gravity-map` so a pose the arm has been
        # measured holding does not read as a stall.
        tq_ceiling = _ceilings(args)[name][1]
        # MEASURED TRAVEL IS A HARD FLOOR UNDER THIS COMMAND.
        # On 2026-11-12 this function happily planned -5 deg into a joint with
        # 1.4 deg of negative travel, and the motor spent 58.7 N.m discovering
        # the difference. The amplitude is the caller's guess; the ranges file
        # is a measurement. The measurement wins.
        rng = _load_ranges().get(name)
        if rng is None and not args.skip_range_check:
            print(f"\nREFUSING: no recorded travel for {name}.\n"
                  f"Record it first -- torque stays off, it cannot hurt "
                  f"anything:\n"
                  f"    python tools/mb_bringup.py --ranges\n"
                  f"Override with --skip-range-check only if you have checked "
                  f"the travel by hand.", file=sys.stderr)
            return 2
        if rng is not None:
            room_up = rng["max_deg"] - RANGE_MARGIN_DEG - home
            room_dn = home - (rng["min_deg"] + RANGE_MARGIN_DEG)
            # Only the direction we actually plan to travel can block us.
            room = {"up": room_up, "down": room_dn}.get(
                args.direction, min(room_up, room_dn))
            if room < args.amplitude:
                if room < 0.5:
                    # A joint parked against a stop is the NORMAL state of this
                    # arm's two gravity-loaded shoulders, not an error. If the
                    # other direction is open, say so instead of just refusing
                    # -- the advice used to be "move the arm somewhere with
                    # room", which for a resting joint means hand-holding a
                    # 36 N.m motor through its own test.
                    other = ("up" if room_up > room_dn else "down")
                    best = max(room_up, room_dn)
                    print(f"\nREFUSING: {name} is at {home:+.2f} deg with "
                          f"measured travel {rng['min_deg']:+.1f}..."
                          f"{rng['max_deg']:+.1f}.\n"
                          f"There is {room:+.2f} deg of room {args.direction} "
                          f"inside a {RANGE_MARGIN_DEG:.0f} deg margin -- it "
                          f"is against a stop.", file=sys.stderr)
                    if best >= args.amplitude and args.direction == "both":
                        print(f"\n{best:.1f} deg IS free going {other}. Test "
                              f"one-sided instead -- the joint lifts off the "
                              f"stop under its own power:\n"
                              f"    python tools/mb_bringup.py --move --joint "
                              f"{name} --amplitude {args.amplitude:.0f} "
                              f"--direction {other}", file=sys.stderr)
                    else:
                        print("Move the arm somewhere with room before "
                              "energising this joint.", file=sys.stderr)
                    return 2
                print(f"!! clamping amplitude {args.amplitude:.1f} -> "
                      f"{room:.1f} deg: measured travel "
                      f"{rng['min_deg']:+.1f}...{rng['max_deg']:+.1f}, "
                      f"home {home:+.2f}, {RANGE_MARGIN_DEG:.0f} deg margin.")
                args.amplitude = room

        plan = _plan(home, args.amplitude, args.cycles, args.direction)
        mode_name = {MODE_MIT: "MIT (no motor-side limits!)",
                     MODE_CSP: "CSP", MODE_POS_VEL: "POS_VEL profile"}[
                         _mode_of(args)]
        print(f"{name}: id {CAN_IDS[name]}, model {MOTOR_MODELS[name]}, "
              f"mode {mode_name}")
        print(f"home {home:+.2f} deg, plan: "
              + " -> ".join(f"{p:+.1f}" for p in plan))
        # kp/kd reach the wire only through `send_mit`. Printing them in a
        # profile-position run invites tuning a gain that is not in the loop --
        # there the motor runs its own, under the vlim/acc ceiling below.
        gains = (f"kp {args.kp}, kd {args.kd}, " if args.mit
                 else "gains: motor-internal, ")
        print(f"steps <= {args.max_step_deg} deg, vlim {args.vlim_dps} dps, "
              f"acc {args.acc_dpss} dps^2, {gains}"
              f"abort at {args.abort_deg} deg error, "
              f"{tq_ceiling:.1f} N.m, {args.max_temp:.0f} degC")

        if args.dry_run:
            print("\n--dry-run: nothing was enabled.")
            return 0

        motor.ensure_mode(_mode_of(args), args.mode_timeout_ms)

        # Command the present position BEFORE enabling, so the motor's first
        # act under torque is to hold still rather than chase a stale target.
        target = home
        _command(motor, args, target)
        motor.enable()
        engaged = True
        _command(motor, args, target, hold=0.4)
        print("\nenabled, holding home.\n")

        print(f"{'target':>9}{'measured':>10}{'err':>8}{'moved':>8}"
              f"{'ratio':>8}{'torque':>8}{'temp':>7}")
        print("-" * 58)

        worst = 0.0
        for waypoint in plan:
            while abs(waypoint - target) > 1e-6:
                before = _state_dict(_poll(motor, args.retries, ctrl)).get(
                    "pos_deg", target)
                step = max(-args.max_step_deg,
                           min(args.max_step_deg, waypoint - target))
                target += step
                # Stream the target for the whole settle window rather than
                # firing once and sleeping -- see `_command`.
                _command(motor, args, target, hold=args.settle)

                s = _state_dict(_poll(motor, args.retries, ctrl))
                pos = s.get("pos_deg", float("nan"))
                err = pos - target
                moved = pos - before
                worst = max(worst, abs(err))
                ratio = moved / step if abs(step) > 1e-6 else float("nan")

                print(f"{target:>9.2f}{pos:>10.2f}{err:>8.2f}{moved:>8.2f}"
                      f"{ratio:>8.2f}{s.get('torque', float('nan')):>8.2f}"
                      f"{s.get('temp', float('nan')):>7.1f}")

                if not math.isfinite(err):
                    print("\nABORT: lost feedback mid-move.")
                    return 1
                if abs(err) > args.abort_deg:
                    print(f"\nABORT: error {err:+.2f} deg exceeds "
                          f"--abort-deg {args.abort_deg}.")
                    return 1
                if abs(ratio) > 4.0 or (abs(moved) > 0.2 and abs(ratio) < 0.25):
                    print(f"\nABORT: moved {moved:+.2f} deg for a "
                          f"{step:+.2f} deg command (ratio {ratio:.2f}).\n"
                          f"Near 9 or 1/9 means the scale is still wrong -- "
                          f"wrong model string or wrong CAN id for this joint.")
                    return 1

                # Stall: torque climbing into a joint that is not moving. The
                # ratio guard above cannot see this -- it exempts small motion.
                torque = s.get("torque", 0.0)
                temp = s.get("temp", 0.0)
                if math.isfinite(torque) and abs(torque) > tq_ceiling:
                    print(f"\nABORT: torque {torque:+.2f} N.m exceeds "
                          f"{tq_ceiling:.1f} ({model}, rated {rated:.0f}).\n"
                          f"The joint moved {moved:+.2f} deg for a "
                          f"{step:+.2f} deg command while pulling that.\n"
                          f"It is driving into something: a mechanical stop, "
                          f"the bench, or another link.\n"
                          f"Do NOT raise the gain. Find the obstruction, and "
                          f"check which direction has travel before retrying.")
                    return 1
                if math.isfinite(temp) and temp > args.max_temp:
                    print(f"\nABORT: {temp:.0f} degC exceeds --max-temp "
                          f"{args.max_temp:.0f}. Let it cool before retrying.")
                    return 1

        print(f"\nworst tracking error: {worst:.2f} deg")
        if worst > 2.0:
            # NOT "raise the gain". That was the old advice here, and on a
            # stalled joint it is the one action that makes things worse --
            # more current into a motor already driving against a hard stop.
            # It is also wrong on its face in POS_VEL mode, where kp never
            # reaches the wire. Say what was actually seen instead.
            print(f"GATE NOT PASSED for {name}: worst error {worst:.2f} deg.\n"
                  f"Read the torque column before changing anything:\n"
                  f"  torque rising while the joint barely moves -> it is "
                  f"driving into an obstruction. Find it.\n"
                  f"  torque ~0 and position frozen -> the motor dropped out. "
                  f"Check --scan for a fault, and check the 48 V rail.\n"
                  f"  torque modest and motion smooth -> genuinely a tuning "
                  f"problem, and only then does --mit --kp mean anything.")
            return 1
        print(f"GATE PASSED for {name}: tracked +-{args.amplitude} deg, no "
              f"fault, no scale error.\nNext joint. Wrists before shoulders.")
        return 0
    finally:
        if engaged:
            try:
                motor.disable()
                print("\ntorque disabled.")
            except Exception as exc:  # noqa: BLE001
                print(f"\n!! could not disable {name}: {exc}\n"
                      f"!! CUT THE DC SWITCH.", file=sys.stderr)
        _reporting([motor], False)
        _shutdown(ctrl)


def _command_all(motors: dict, args, targets: dict[str, float],
                 hold: float = 0.0) -> None:
    """Stream a target to several motors at once, for `hold` seconds.

    One pass over every joint per period, not `_command` per joint in series:
    with six motors a serial version would hold joint 1's target six periods
    longer than joint 6's, which on an arm means the links fight each other.
    """
    period = 1.0 / max(1.0, args.stream_hz)
    deadline = time.monotonic() + max(0.0, hold)
    while True:
        for name, target in targets.items():
            _command(motors[name], args, target)
        if time.monotonic() >= deadline:
            return
        time.sleep(period)


def replay(args) -> int:
    """Drive the arm through a recorded trajectory. ALL joints, under power.

    This is the largest thing in this file by risk, and it is worth being
    explicit about why. Every other action here moves ONE joint a few degrees.
    This moves six at once through an arbitrary path, which is both the highest
    current the arm has drawn and the first time links can collide with each
    other. The LRS-600-48 is specified at exactly the arm's rating with no
    headroom, and its overload behaviour is hiccup mode -- a trip drops torque
    on all six simultaneously and the arm falls.

    So every gate that exists is applied, and two more are added:

      * joints with no measured travel are REFUSED, not skipped silently
      * every target is clamped into measured travel before it is ever sent
      * the timeline is stretched if any segment implies more than --vlim-dps
      * the arm is walked to frame 0 under bounded steps before playback, so
        the first frame is never a jump from wherever it happens to be
    """
    path = Path(args.replay)
    if not path.exists():
        path = TRAJ_DIR / args.replay
    if not path.exists():
        print(f"no such recording: {args.replay}", file=sys.stderr)
        avail = sorted(TRAJ_DIR.glob("traj_*.json")) if TRAJ_DIR.exists() else []
        if avail:
            print("available:", file=sys.stderr)
            for p in avail[-5:]:
                print(f"    {p.name}", file=sys.stderr)
        return 2
    try:
        doc = json.loads(path.read_text())
        frames = doc["frames"]
    except (OSError, ValueError, KeyError) as exc:
        print(f"cannot read {path}: {exc}", file=sys.stderr)
        return 2
    if len(frames) < 2:
        print(f"{path} holds {len(frames)} frame(s); nothing to replay.",
              file=sys.stderr)
        return 2

    # Which joints can be driven at all. A joint with no measured travel has
    # no clamp available, and an unclamped target on a 36 N.m motor is how
    # elbow_flex found a hard stop at 58.7 N.m.
    rngs = _load_ranges()
    recorded = [n for n in CAN_IDS if any(n in f["pos"] for f in frames)]

    # --joints is applied BEFORE the range check, not after. The other order
    # makes the refusal below a dead end: it tells the caller to drop the
    # unmeasured joint with --joints, and then refuses again on the same joint
    # because it was still in `blocked`.
    if args.joints:
        want = [j.strip() for j in args.joints.split(",") if j.strip()]
        bad = [j for j in want if j not in recorded]
        if bad:
            print(f"not in this recording: {', '.join(bad)}\n"
                  f"it holds: {', '.join(recorded)}", file=sys.stderr)
            return 2
        recorded = [n for n in recorded if n in want]

    playable = [n for n in recorded if n in rngs]
    blocked = [n for n in recorded if n not in rngs]
    if blocked and not args.skip_range_check:
        print(f"REFUSING: no measured travel for {', '.join(blocked)}.\n"
              f"Every target has to be clamped into a measured range before it "
              f"is sent.\nSweep them first -- torque stays off:\n"
              f"    python tools/mb_bringup.py --ranges --joint {blocked[0]}\n"
              f"Or drop them from this replay:\n"
              f"    ... --replay {path.name} --joints "
              f"{','.join(playable)}", file=sys.stderr)
        return 2
    if not playable:
        print("no playable joints.", file=sys.stderr)
        return 2

    # Clamp first, THEN measure the motion. Clamping after a velocity check
    # would mean checking speeds the arm is never commanded to make.
    clamped: dict[str, int] = {n: 0 for n in playable}
    track: list[tuple[float, dict[str, float]]] = []
    for f in frames:
        row = {}
        for n in playable:
            if n not in f["pos"]:
                continue
            r = rngs[n]
            lo = r["min_deg"] + RANGE_MARGIN_DEG
            hi = r["max_deg"] - RANGE_MARGIN_DEG
            if lo > hi:  # travel narrower than two margins
                lo = hi = (r["min_deg"] + r["max_deg"]) / 2.0
            v = min(hi, max(lo, f["pos"][n]))
            if abs(v - f["pos"][n]) > 1e-6:
                clamped[n] += 1
            row[n] = v
        if row:
            track.append((f["t"], row))
    if len(track) < 2:
        print("nothing left after clamping.", file=sys.stderr)
        return 2

    # Stretch the timeline so no segment demands more than --vlim-dps. The
    # motor would clamp this itself in POS_VEL, but then playback silently
    # lags the timeline and every later frame is a growing step change.
    worst_dps, worst_at = 0.0, ""
    for (t0, a), (t1, b) in zip(track, track[1:]):
        dt = max(1e-3, t1 - t0)
        for n in a.keys() & b.keys():
            dps = abs(b[n] - a[n]) / dt
            if dps > worst_dps:
                worst_dps, worst_at = dps, n
    # ORDER MATTERS. Applying the vlim floor and THEN dividing by --speed lets
    # --speed undo it: a 15 dps recording at --speed 5 came out as 0.20x, i.e.
    # 75 dps, against a 20 dps limit, while the flag's help promised it could
    # never do that. The floor has to be the last word.
    stretch = max(0.05, 1.0 / max(0.05, args.speed))
    vlim_floor = worst_dps / max(1.0, args.vlim_dps)
    stretch = max(stretch, vlim_floor, 1e-3)
    capped = vlim_floor > 1.0 / max(0.05, args.speed)

    print(f"{path.name}: {len(track)} frames, {track[-1][0]:.1f} s recorded")
    print(f"joints: {', '.join(playable)}")
    if blocked:
        print(f"EXCLUDED (no measured travel): {', '.join(blocked)}")
    any_clamped = {n: c for n, c in clamped.items() if c}
    if any_clamped:
        print("clamped into measured travel: " + ", ".join(
            f"{n} {c} frames" for n, c in any_clamped.items()))
    pace = ("as recorded" if abs(stretch - 1.0) < 0.01
            else f"{stretch:.2f}x slower" if stretch > 1.0
            else f"{1.0 / stretch:.2f}x faster")
    print(f"fastest segment {worst_dps:.1f} dps ({worst_at}), playing {pace} "
          f"-> {worst_dps / stretch:.1f} dps peak, {track[-1][0] * stretch:.1f} s"
          + (f"  [capped by --vlim-dps {args.vlim_dps:.0f}]" if capped else ""))

    mb, ctrl, motors = _open(args)
    motors = {n: m for n, m in motors.items() if n in playable}
    engaged = False
    warned: set[str] = set()
    try:
        _reporting(motors.values(), True)
        time.sleep(0.3)
        start = _sample_all(ctrl, motors)
        missing = [n for n in playable if "pos_deg" not in start.get(n, {})]
        if missing:
            print(f"not answering: {', '.join(missing)}. Run --scan.",
                  file=sys.stderr)
            return 1

        first = track[0][1]
        print(f"\n{'joint':<15}{'now':>9}{'frame 0':>9}{'approach':>10}")
        print("-" * 43)
        far = 0.0
        for n in playable:
            d = first[n] - start[n]["pos_deg"]
            far = max(far, abs(d))
            print(f"{n:<15}{start[n]['pos_deg']:>9.2f}{first[n]:>9.2f}"
                  f"{d:>+10.2f}")
        print(f"\nlargest approach move: {far:.2f} deg")

        if args.dry_run:
            print("\n--dry-run: nothing was enabled.")
            return 0

        print(f"\n!! ALL {len(playable)} JOINTS GO UNDER POWER AT ONCE.\n"
              f"   Support the arm. Keep a hand on the DC switch -- it is the "
              f"only abort that works if the stream stalls.\n"
              f"   Guards: {args.abort_deg:.0f} deg error, "
              f"{args.max_temp:.0f} degC, torque ceiling per model.")
        if sys.stdin.isatty():
            if input("type 'replay' to continue: ").strip() != "replay":
                print("aborted.")
                return 1

        for m in motors.values():
            m.ensure_mode(_mode_of(args), args.mode_timeout_ms)
        hold = {n: start[n]["pos_deg"] for n in playable}
        _command_all(motors, args, hold)
        for m in motors.values():
            m.enable()
        engaged = True
        _command_all(motors, args, hold, hold=0.4)
        print("\nenabled, holding.")

        # Approach frame 0 under the same bounded steps --move uses.
        print("approaching frame 0...")
        while True:
            step_left = False
            for n in playable:
                d = first[n] - hold[n]
                if abs(d) > 1e-3:
                    hold[n] += max(-args.max_step_deg,
                                   min(args.max_step_deg, d))
                    step_left = True
            _command_all(motors, args, hold, hold=args.settle)
            if _guard(ctrl, motors, args, hold, warned=warned):
                return _settle_and_release(ctrl, motors, args, engaged)
            if not step_left:
                break
        print("at frame 0.\n")

        print(f"{'t':>7}{'frame':>8}" + "".join(f"{n[:7]:>9}" for n in playable)
              + f"{'maxerr':>9}{'maxtq':>8}{'maxT':>6}")
        print("-" * (15 + 9 * len(playable) + 23))

        t_start = time.monotonic()
        worst_err = worst_tq = worst_temp = 0.0
        for i, (t, row) in enumerate(track):
            due = t_start + t * stretch
            while True:
                now = time.monotonic()
                if now >= due:
                    break
                _command_all(motors, args, row)
                time.sleep(min(0.01, due - now))
            _command_all(motors, args, row)
            s = _sample_all(ctrl, motors)
            err = max((abs(s[n]["pos_deg"] - row[n])
                       for n in row if "pos_deg" in s.get(n, {})), default=0.0)
            tq = max((abs(s[n].get("torque", 0.0)) for n in s), default=0.0)
            tp = max((s[n].get("temp", 0.0) for n in s), default=0.0)
            worst_err = max(worst_err, err)
            worst_tq = max(worst_tq, tq)
            worst_temp = max(worst_temp, tp)
            if i % 5 == 0 or i == len(track) - 1:
                sys.stdout.write(
                    f"\r\033[2K{t * stretch:>7.1f}{i:>8}" +
                    "".join(f"{row[n]:>9.1f}" for n in playable) +
                    f"{err:>9.2f}{tq:>8.2f}{tp:>6.0f}")
                sys.stdout.flush()
            if _guard(ctrl, motors, args, row, sampled=s, warned=warned):
                return _settle_and_release(ctrl, motors, args, engaged)

        print(f"\n\nreplayed {len(track)} frames in "
              f"{time.monotonic() - t_start:.1f} s")
        print(f"worst tracking error {worst_err:.2f} deg, peak torque "
              f"{worst_tq:.2f} N.m, peak {worst_temp:.0f} degC")
        if worst_err > args.abort_deg / 3:
            print(f"\n!! {worst_err:.2f} deg is high. Read the torque column "
                  f"before replaying faster.")
        return 0
    except KeyboardInterrupt:
        print("\n\ninterrupted -- disabling torque.")
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
        _reporting(motors.values(), False)
        _shutdown(ctrl)


def gravity_map(args) -> int:
    """Walk ONE joint to a chosen angle and record what every joint carried.

    This exists because the torque ceilings were a guess and the guess was
    wrong in the expensive direction. `rated * 0.5` is a number from a
    datasheet; what the arm actually needs to hold itself up is a property of
    its mass and geometry, and the only way to know it is to put the arm in the
    pose and read the torque off the motors that are holding it.

    The sweep is deliberately boring. It steps the chosen joint toward
    `--to-deg` in `--max-step-deg` increments, dwells `--settle` at each one so
    the reading is static rather than a transient, and samples all seven. Every
    joint's peak |torque| over the whole walk goes into the file -- not just
    the joint being moved, because the joint being moved is usually NOT the one
    that ends up loaded. Extending the elbow is what loads the shoulder.

    Guarded throughout, and an abort goes through `_settle_and_release` like
    everything else. The target is bounded by the measured travel the same way
    `--move` is: this walks much further than `--move` does, so planning into a
    hard stop would be correspondingly worse.
    """
    name = args.joint
    if args.to_deg is None:
        print("--gravity-map needs --to-deg: the angle to walk this joint to.\n"
              "Pick a pose that LOADS the arm -- an extended elbow, not a "
              "folded one -- and stay inside the measured travel.",
              file=sys.stderr)
        return 2

    rng = _load_ranges().get(name)
    if rng is None and not args.skip_range_check:
        print(f"\nREFUSING: no recorded travel for {name}.\n"
              f"    python tools/mb_bringup.py --ranges --joint {name}",
              file=sys.stderr)
        return 1
    if rng is not None:
        lo, hi = rng["min_deg"], rng["max_deg"]
        if not lo <= args.to_deg <= hi:
            print(f"\nREFUSING: --to-deg {args.to_deg:.1f} is outside {name}'s "
                  f"measured travel {lo:.1f} .. {hi:.1f}.\n"
                  f"The amplitude is your guess; the ranges file is a "
                  f"measurement. The measurement wins.", file=sys.stderr)
            return 1

    mb, ctrl, motors = _open(args)
    engaged = False
    warned: set[str] = set()
    try:
        _reporting(motors.values(), True)
        time.sleep(0.3)
        start = _sample_all(ctrl, motors)
        if name not in start:
            print(f"{name} is not answering. Run --scan.", file=sys.stderr)
            return 1
        here = start[name]["pos_deg"]
        steps = int(math.ceil(abs(args.to_deg - here) / max(0.01,
                                                            args.max_step_deg)))
        print(f"\n{name}: {here:.2f} -> {args.to_deg:.2f} deg "
              f"({steps} steps of {args.max_step_deg:.2f}), all other joints "
              f"held.")
        print("Peak torque on EVERY joint is recorded, not just this one.")
        if args.dry_run:
            print("\n--dry-run: nothing enabled.")
            return 0

        print("\n!! ALL 7 JOINTS GO UNDER POWER and one of them travels a "
              "long way.\n!! Keep a hand on the DC switch.")
        if input("type 'gravity' to continue: ").strip() != "gravity":
            print("aborted.")
            return 1

        for m in motors.values():
            m.ensure_mode(_mode_of(args), args.mode_timeout_ms)
        hold = {n: st["pos_deg"] for n, st in start.items() if n in motors}
        _command_all(motors, args, hold)
        for m in motors.values():
            m.enable()
        engaged = True
        _command_all(motors, args, hold, hold=0.4)
        print("\nenabled, holding.\n")

        order = [n for n in CAN_IDS if n in motors]
        print(f"{'target':>8}" + "".join(f"{ABBREV_TQ.get(n, n[:5]):>8}"
                                         for n in order))
        print("-" * (8 + 8 * len(order)))
        peak: dict[str, float] = {n: 0.0 for n in order}
        peak_at: dict[str, float] = {n: here for n in order}

        for _ in range(steps + 1):
            d = args.to_deg - hold[name]
            hold[name] += max(-args.max_step_deg,
                              min(args.max_step_deg, d))
            _command_all(motors, args, hold, hold=args.settle)
            s = _sample_all(ctrl, motors)
            for n in order:
                tq = abs(s.get(n, {}).get("torque", 0.0))
                if math.isfinite(tq) and tq > peak[n]:
                    peak[n] = tq
                    peak_at[n] = hold[name]
            print(f"{hold[name]:>8.1f}"
                  + "".join(f"{s.get(n, {}).get('torque', float('nan')):>8.2f}"
                            for n in order))
            if _guard(ctrl, motors, args, hold, sampled=s, warned=warned):
                # Everything measured up to the abort is still true, and the
                # pose that caused it is the most informative sample in the
                # run. Write before releasing, not after.
                _write_gravity(peak, peak_at, order, name, hold[name])
                print(f"\npartial map written to {GRAVITY_PATH}")
                return _settle_and_release(ctrl, motors, args, engaged)
            if abs(args.to_deg - hold[name]) < 1e-3:
                break

        _write_gravity(peak, peak_at, order, name, hold[name])
        print(f"\n{'joint':<15}{'peak N.m':>10}{'% rated':>9}"
              f"{'at ' + name:>18}")
        print("-" * 52)
        for n in order:
            rated = RATED_TORQUE_NM[MOTOR_MODELS[n]]
            print(f"{n:<15}{peak[n]:>10.2f}{peak[n] / rated * 100:>8.0f}%"
                  f"{peak_at[n]:>18.1f}")
        print(f"\nwritten to {GRAVITY_PATH}")
        print("The guard now reads this file: a joint is allowed "
              f"{GRAVITY_MARGIN:.2f}x its measured peak before\n"
              "high torque counts as suspicious, capped at "
              f"{HARD_TORQUE_FRAC:.0%} of rated.")
        print("\nThis maps ONE pose family. Repeat it for the poses you "
              "actually teleop through --\nthe file keeps the highest peak "
              "seen per joint across runs.")
        return 0
    except KeyboardInterrupt:
        print("\n\ninterrupted.")
        return _settle_and_release(ctrl, motors, args, engaged)
    finally:
        if engaged:
            for n, m in motors.items():
                try:
                    m.disable()
                except Exception as exc:  # noqa: BLE001
                    print(f"\n!! could not disable {n}: {exc}\n"
                          f"!! CUT THE DC SWITCH.", file=sys.stderr)
            print("torque disabled.")
        _reporting(motors.values(), False)
        _shutdown(ctrl)


#: Short headings for the gravity table; the full names do not fit seven wide.
ABBREV_TQ: dict[str, str] = {
    "shoulder_pan": "pan", "shoulder_lift": "lift", "elbow_flex": "elbow",
    "wrist_flex": "wflex", "wrist_yaw": "wyaw", "wrist_roll": "wroll",
    "gripper": "grip",
}


def _write_gravity(peak: dict[str, float], peak_at: dict[str, float],
                   scope: list[str], swept: str, reached: float) -> None:
    """Merge this run's peaks into the gravity file, keeping the HIGHEST.

    Merging rather than replacing, and keeping the maximum, because one sweep
    only visits one family of poses. A run that extends the elbow loads the
    shoulder; a run that pans does not. Overwriting would let the second run
    silently lower a ceiling the first run earned, and a ceiling that drops
    when you measure more is worse than no ceiling at all.

    This is the opposite convention to `_write_ranges`, which rewrites its
    scope outright. The difference is what a stale value costs: a leftover wide
    TRAVEL lets `--move` plan into a stop, so stale travel must be dropped. A
    leftover high TORQUE peak only makes the guard more permissive in a pose
    the arm has genuinely held before, and it is bounded by the hard ceiling.
    """
    out = _load_gravity()
    for n in scope:
        prev = out.get(n, {})
        if float(prev.get("peak_nm", 0.0)) >= peak[n]:
            continue
        out[n] = {"peak_nm": round(peak[n], 2),
                  "at_deg": round(peak_at[n], 2),
                  "swept": swept,
                  "recorded": time.strftime("%Y-%m-%dT%H:%M:%S")}
    out = {n: out[n] for n in CAN_IDS if n in out}
    GRAVITY_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = GRAVITY_PATH.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(
        {"recorded": time.strftime("%Y-%m-%dT%H:%M:%S"),
         "note": "peak |torque| each joint was measured holding during a slow "
                 "stepped sweep, under power. Highest value seen across runs. "
                 "'at_deg' is the swept joint's angle at that peak.",
         "last_sweep": {"joint": swept, "reached_deg": round(reached, 2)},
         "joints": out}, indent=2) + "\n")
    tmp.replace(GRAVITY_PATH)


def fit_gravity(args) -> int:
    """Fit the pose model from logged teleop samples. Touches no hardware.

    Reads `gravity_samples.jsonl`, fits each chain joint by least squares, and
    writes `gravity_model.json` -- but only for joints that earn it. The three
    gates below exist because a bad model here is worse than no model: the
    guard TRUSTS a fitted prediction, and a prediction fitted to one corner of
    the workspace will happily explain away 20 N.m of obstruction in a pose it
    has never seen.

      enough samples   `MIN_FIT_SAMPLES`, against 7 free parameters.
      enough spread    `MIN_FIT_SPAN_DEG` of travel on every chain angle the
                       joint depends on. An arm that only ever moved its elbow
                       teaches nothing about the shoulder's dependence on lift.
      beats a constant the fit must explain real variance. If predicting the
                       mean does as well, there is no pose dependence in this
                       data and the coefficients are noise.

    Run it after any teleop session. The sample log appends, so coverage
    accumulates and a model fitted today is refitted better next week.
    """
    if not GRAVITY_SAMPLES_PATH.exists():
        print(f"No samples at {GRAVITY_SAMPLES_PATH}.\n"
              f"They are written by the teleop loop. Run a normal session "
              f"first -- move the arm through\nthe poses you actually use, "
              f"especially extended ones, then come back and fit.")
        return 1

    rows: list[dict] = []
    bad = 0
    for line in GRAVITY_SAMPLES_PATH.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except ValueError:
            bad += 1
    print(f"{len(rows)} samples from {GRAVITY_SAMPLES_PATH.name}"
          + (f" ({bad} unparseable lines skipped)" if bad else ""))
    if not rows:
        return 1

    out: dict[str, dict] = {}
    print()
    for name in PITCH_CHAIN:
        key = name + "_tq"
        design: list[list[float]] = []
        ys: list[float] = []
        poses: list[dict[str, float]] = []
        for r in rows:
            if key not in r:
                continue
            pose = {j: r[j] for j in PITCH_CHAIN if j in r}
            feats = _features(name, pose)
            if feats is None:
                continue
            design.append(feats)
            ys.append(float(r[key]))
            poses.append(pose)

        if len(design) < MIN_FIT_SAMPLES:
            print(f"  {name:<14} SKIPPED -- {len(design)} samples, needs "
                  f"{MIN_FIT_SAMPLES}. Collect more.")
            continue

        k = PITCH_CHAIN.index(name)
        spans = []
        for j in PITCH_CHAIN[k:]:
            vals = [p[j] for p in poses if j in p]
            spans.append(max(vals) - min(vals) if vals else 0.0)
        if min(spans) < MIN_FIT_SPAN_DEG:
            thin = PITCH_CHAIN[k:][spans.index(min(spans))]
            print(f"  {name:<14} SKIPPED -- {thin} only moved "
                  f"{min(spans):.0f} deg, needs {MIN_FIT_SPAN_DEG:.0f}.\n"
                  f"  {'':<14} The data cannot separate its effect from the "
                  f"constant term.")
            continue

        coef = _solve_ridge(design, ys)
        if coef is None:
            print(f"  {name:<14} SKIPPED -- design matrix is singular.")
            continue

        resid = [y - sum(c * f for c, f in zip(coef, row))
                 for row, y in zip(design, ys)]
        rms = math.sqrt(sum(e * e for e in resid) / len(resid))
        mean = sum(ys) / len(ys)
        sd = math.sqrt(sum((y - mean) ** 2 for y in ys) / len(ys))
        if sd < 1e-9 or rms > 0.8 * sd:
            print(f"  {name:<14} SKIPPED -- residual {rms:.2f} N.m vs "
                  f"{sd:.2f} spread in the data.\n"
                  f"  {'':<14} The pose does not explain the torque here. "
                  f"Not writing a model that adds nothing.")
            continue

        lim = max(MIN_RESIDUAL_NM[MOTOR_MODELS[name]], RESIDUAL_SIGMA * rms)
        out[name] = {
            "coef": [round(c, 6) for c in coef],
            "terms": [f"{t}({j})" for j in PITCH_CHAIN[k:]
                      for t in ("cos", "sin")] + ["1"],
            "samples": len(design),
            "resid_rms": round(rms, 4),
            "resid_max": round(max(abs(e) for e in resid), 4),
            "torque_sd": round(sd, 4),
            "span_deg": {j: round(s, 1)
                         for j, s in zip(PITCH_CHAIN[k:], spans)},
        }
        print(f"  {name:<14} fitted from {len(design):>5} samples: "
              f"residual {rms:.2f} N.m rms, {max(abs(e) for e in resid):.2f} "
              f"max,\n  {'':<14} against {sd:.2f} N.m of spread. "
              f"Guard will flag past {lim:.1f} N.m unexplained.")

    if not out:
        print("\nNothing fitted. The guard keeps its fixed ceilings, which "
              "means it stays blind in folded\nposes and twitchy in extended "
              "ones. Collect more varied samples and run this again.")
        return 1

    tmp = GRAVITY_MODEL_PATH.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(
        {"fitted": time.strftime("%Y-%m-%dT%H:%M:%S"),
         "note": "Least-squares gravity model, tau_k = sum_{j>=k} "
                 "a_j*cos(THETA_j) + b_j*sin(THETA_j) + c, where THETA_j is "
                 "the cumulative chain angle in radians. Fitted from "
                 "near-stationary, tracking teleop samples. Regenerate with "
                 "--fit-gravity after any session.",
         "chain": list(PITCH_CHAIN),
         "joints": out}, indent=2) + "\n")
    tmp.replace(GRAVITY_MODEL_PATH)
    print(f"\nwrote {GRAVITY_MODEL_PATH}")
    print("The guard now asks whether the torque is more than the POSE "
          "explains, not whether it is\nlarge. Extended poses stop "
          "false-aborting; folded poses stop being blind.")
    return 0


def _guard(ctrl, motors, args, targets: dict[str, float],
           sampled: dict | None = None,
           warned: set[str] | None = None) -> int:
    """Torque / temperature / tracking check across every driven joint.

    Returns non-zero to abort. Shared by the approach and the playback so a
    joint cannot be protected during one and bare during the other.

    High torque on its own is NOT an abort. See the two-ceiling note at the top
    of this file: an arm holding itself up at full extension and an arm driving
    into a hard stop read the same on the torque channel, and only the tracking
    error and the velocity separate them. Getting that wrong in either
    direction is expensive -- ignoring a stall cooked a motor to 62 degC in the
    morning, and aborting on a gravity load dropped an extended arm in the
    evening of the same day.

    `warned` is a caller-owned set of joints that have already had their
    gravity-load notice printed. Without it a joint parked above its warn
    ceiling reprints every frame, at 20 Hz, over the live table.
    """
    s = sampled if sampled is not None else _sample_all(ctrl, motors)
    ceilings = _ceilings(args)
    gmodel = {} if getattr(args, "no_gravity_model", False) else _gmodel()
    pose = {n: st.get("pos_deg", float("nan")) for n, st in s.items()}
    for name, st in s.items():
        if name not in targets:
            continue
        model = MOTOR_MODELS[name]
        stall_nm, warn_nm, hard_nm = ceilings[name]
        rated = RATED_TORQUE_NM[model]
        tq = st.get("torque", 0.0)
        tp = st.get("temp", 0.0)
        pos = st.get("pos_deg", float("nan"))
        vel = st.get("vel_dps", float("nan"))
        err = abs(pos - targets[name]) if math.isfinite(pos) else 0.0

        # What does this POSE say the torque should be? When there is a fitted
        # model, that prediction replaces the fixed stall ceiling entirely --
        # see the long note above `PITCH_CHAIN`. The residual is the part
        # gravity does not account for, and it is sensitive in exactly the
        # poses the scalar was blind in: folded, where 17 N.m of obstruction
        # used to sit under an 18 N.m ceiling and say nothing.
        exp = _expected_torque(name, pose, gmodel)
        lim = _residual_limit(name, gmodel)
        modelled = (exp is not None and lim is not None
                    and math.isfinite(tq))
        resid = (tq - exp) if modelled else 0.0

        if math.isfinite(tq) and abs(tq) > hard_nm:
            _rec_event(f"abort:{name}:hard:{tq:+.2f}")
            print(f"\n\nABORT: {name} at {tq:+.2f} N.m is past the hard "
                  f"ceiling {hard_nm:.1f} ({model}, rated {rated:.0f}).\n"
                  f"That is {abs(tq) / rated * 100:.0f}% of rated continuous. "
                  f"No pose of an unloaded arm needs that.\n"
                  f"Do NOT raise --max-torque to get past it.")
            return 1

        # Stalled, or holding itself up? A stall is commanded somewhere it is
        # not getting to, and not moving. Both terms matter: a joint with a
        # large error that is still travelling is catching up, and a joint with
        # no error is not being asked for anything it is failing to do, however
        # hard it is working to stay there.
        # Two thresholds without a model, one with. The fixed pair existed only
        # because a scalar cannot tell a heavy pose from an obstruction, so it
        # needed a low number to abort on and a higher one merely to mention.
        # A residual already separates those cases, so it serves both.
        if modelled:
            over = over_warn = abs(resid) > lim
        else:
            over = math.isfinite(tq) and abs(tq) > stall_nm
            over_warn = math.isfinite(tq) and abs(tq) > warn_nm

        if (over and err > STALL_ERR_DEG
                and math.isfinite(vel) and abs(vel) < STALL_VEL_DPS):
            detail = (f"{tq:+.2f} N.m where this pose explains {exp:+.2f} "
                      f"-- {resid:+.2f} unaccounted for" if modelled
                      else f"{tq:+.2f} N.m")
            _rec_event(f"abort:{name}:stall:{tq:+.2f}:err{err:.2f}")
            print(f"\n\nABORT: {name} is STALLED -- {detail},\n"
                  f"{err:.2f} deg from target, moving {vel:+.1f} deg/s.\n"
                  f"Torque the pose does not explain, not getting there, "
                  f"not moving. It is driving into\nsomething: a stop, the "
                  f"bench, or another link. Do NOT raise --max-torque to "
                  f"get past it.")
            return 1

        if over_warn and warned is not None and name not in warned:
            warned.add(name)
            if modelled:
                print(f"\n\n!! {name} is carrying {tq:+.2f} N.m where this "
                      f"pose explains {exp:+.2f} --\n"
                      f"!! {resid:+.2f} N.m unaccounted for, past the "
                      f"{lim:.1f} limit, but tracking to {err:.2f} deg.\n"
                      f"!! Something is touching the arm, or there is a "
                      f"payload in the gripper. Not a stall.\n"
                      f"!! Hard ceiling is {hard_nm:.1f}.\n")
            else:
                print(f"\n\n!! {name} is carrying {tq:+.2f} N.m, over the "
                      f"{warn_nm:.1f} warn ceiling, but tracking to "
                      f"{err:.2f} deg.\n"
                      f"!! Reading that as the arm's own weight, not a stall. "
                      f"Hard ceiling is {hard_nm:.1f}.\n"
                      f"!! There is no fitted gravity model, so this is an "
                      f"inference from one number.\n"
                      f"!! Run --fit-gravity after this session to replace "
                      f"it with a pose-aware one.\n")

        if math.isfinite(tp) and tp > args.max_temp:
            _rec_event(f"abort:{name}:temp:{tp:.0f}")
            print(f"\n\nABORT: {name} at {tp:.0f} degC exceeds --max-temp "
                  f"{args.max_temp:.0f}. Let it cool.")
            return 1
        if math.isfinite(pos) and err > args.abort_deg:
            _rec_event(f"abort:{name}:track:{pos - targets[name]:+.2f}")
            print(f"\n\nABORT: {name} is {pos - targets[name]:+.2f} deg from "
                  f"target, past --abort-deg {args.abort_deg:.0f}.\n"
                  f"Either it is obstructed, or the 48 V rail dropped and it "
                  f"lost torque. Check --scan.")
            return 1
    return 0


def _settle_and_release(ctrl, motors, args, engaged: bool = True) -> int:
    """Stop pushing, hold the arm where it is, then release on a countdown.

    WHAT THIS REPLACES, and why it is worth the complexity: every abort path in
    this file used to return straight into a `finally` that called `disable()`
    on all six motors. On a folded arm that is harmless. On an arm extended at
    143 deg it converts a warning into a fall, which is exactly what happened
    on 2026-11-12 -- the guard was right that torque was high and its response
    destroyed more than the condition would have.

    Two things happen here, in order, and the first is the important one:

    1. RE-TARGET EVERY JOINT TO WHERE IT ACTUALLY IS. If the abort was a stall,
       the stall was caused by a commanded position the joint could not reach;
       setting the target to the measured position removes the error, and with
       it the torque, without removing the torque that holds the arm up. A
       stalled joint stops pushing immediately. A gravity-loaded joint keeps
       exactly the load it needs and no more.
    2. Stream that, announced, until something ends the hold -- then return and
       let the caller's `finally` release.

    WHAT ENDS THE HOLD, and why it is not a stopwatch by default. The first
    version counted down 5 seconds. On 2026-11-12 that window expired while the
    arm was fully extended and it fell anyway: five seconds is not enough to
    read a warning and get a hand under 10 kg. A clock is the wrong thing to
    key on, because the clock knows nothing about the arm.

    Heat is the right thing to key on, because heat is the only reason holding
    is not free. A motor asked to stand still under load is dissipating that
    load continuously -- the same run was at 40 degC holding 15.34 N.m, and the
    morning stall reached 62 degC. So `--hold-on-fault -1`, the default, holds
    INDEFINITELY and ends on whichever comes first:

        - any joint reaches `--hold-max-temp`, which is the real limit and is
          announced a few degrees out so it is not a surprise;
        - the operator presses Ctrl-C, having taken the weight;
        - nothing. It will hold all day if the arm is cool enough to.

    A positive `--hold-on-fault` restores the old fixed window, and 0 releases
    at once. Both remain honest options; neither is the default any more.

    The guards deliberately do NOT re-run during the hold. They just fired;
    re-running them would abort the abort handler on its first pass and release
    instantly, which is the behaviour this exists to prevent. Temperature is
    the one check that does run, because it is the one condition that gets
    worse the longer the hold succeeds.

    STILL NOT A CONTROLLED DESCENT. When the temperature limit does arrive, the
    arm goes limp wherever it is. Holding longer moves the fall rather than
    preventing it, and lowering an extended arm to the bench under power
    remains unbuilt. Returns 1 always: this is an abort path, and nothing it
    does makes the run a success.
    """
    if not engaged:
        return 1
    secs = getattr(args, "hold_on_fault", -1.0)
    if secs == 0:
        print("--hold-on-fault 0: releasing torque now.")
        return 1

    s = _sample_all(ctrl, motors)
    hold = {n: st["pos_deg"] for n, st in s.items()
            if n in motors and math.isfinite(st.get("pos_deg", float("nan")))}
    if not hold:
        # No positions means no safe target to stream. Holding the LAST
        # commanded target instead would be a guess about an arm we have just
        # lost sight of, and guessing is how a joint gets driven into a stop.
        _rec_event("release:blind")
        print("\n!! could not read any position -- releasing torque NOW.\n"
              "!! TAKE THE WEIGHT OF THE ARM.", file=sys.stderr)
        return 1

    hot = getattr(args, "hold_max_temp", STALL_TEMP_C)
    forever = secs < 0
    _rec_event("freeze:forever" if forever else f"freeze:{secs:.0f}s")
    if forever:
        print(f"\n!! FROZEN. Holding position until {hot:.0f} degC or Ctrl-C.")
        print("!! Targets reset to measured position -- nothing is being "
              "pushed.")
        print("!! Take the weight, then Ctrl-C to release. The arm will NOT "
              "drop on a timer.")
        print(f"!! It WILL drop if any joint reaches {hot:.0f} degC. Watch the "
              f"temperature.\n")
        end = float("inf")
    else:
        print(f"\n!! HOLDING WHERE IT IS FOR {secs:.0f} SECONDS, THEN TORQUE "
              f"DROPS.")
        print("!! Targets reset to measured position -- nothing is being "
              "pushed.")
        print("!! TAKE THE WEIGHT OF THE ARM NOW, or cut the DC switch.\n")
        end = time.monotonic() + secs

    period = 1.0 / max(1.0, args.stream_hz)
    t0 = time.monotonic()
    near = False
    try:
        while time.monotonic() < end:
            step = time.monotonic()
            _command_all(motors, args, hold)
            s = _sample_all(ctrl, motors)
            # Recorded INSIDE the hold, every pass. The last fall happened at
            # the end of a hold, so a log that stopped at the abort would
            # contain everything except the event.
            if REC is not None:
                REC.write(s, hold, {})
            tq = max((abs(s[n].get("torque", 0.0)) for n in s), default=0.0)
            tp = max((s[n].get("temp", 0.0) for n in s), default=0.0)
            hottest = max(s, key=lambda n: s[n].get("temp", 0.0), default="")

            if tp >= hot:
                _rec_event(f"release:temp:{hottest}:{tp:.0f}")
                print(f"\n\n!! {hottest} reached {tp:.0f} degC, at or past "
                      f"--hold-max-temp {hot:.0f}.\n"
                      f"!! Holding longer damages the motor. RELEASING "
                      f"TORQUE -- TAKE THE WEIGHT NOW.")
                return 1
            # Announce the limit before it arrives. A hold that ends without
            # warning is the 5-second countdown again, just with a different
            # trigger -- the point of keying on heat is that heat is visible
            # on the way up.
            if not near and tp >= hot - 5.0:
                near = True
                print(f"\n\n!! {hottest} at {tp:.0f} degC, within 5 of the "
                      f"{hot:.0f} limit. Torque drops when it gets there.\n")

            if sys.stdout.isatty():
                left = ("no limit" if forever
                        else f"{end - time.monotonic():>4.1f} s left")
                sys.stdout.write(
                    f"\r\033[2Kfrozen {time.monotonic() - t0:>5.1f} s, {left}, "
                    f"peak torque {tq:>6.2f} N.m, {tp:>3.0f}/{hot:.0f} degC")
                sys.stdout.flush()
            # Pace it. `_command_all` with no hold sends one pass and returns,
            # so without this the only thing limiting the rate is however long
            # `_sample_all` happens to take.
            time.sleep(max(0.0, period - (time.monotonic() - step)))
    except KeyboardInterrupt:
        # The operator has the arm. This is the intended way out of an
        # indefinite hold, so it is not an error and must not print a
        # traceback over the thing they are reading.
        _rec_event("release:operator")
        print("\n\nreleased by operator.")
        return 1
    _rec_event("release:expired")
    print("\nhold expired -- releasing torque.")
    return 1


#: Where `--record` writes and `--replay` reads. Not `calibration/`: a
#: trajectory is a thing the arm did once, not a property of the hardware.
TRAJ_DIR = Path(__file__).resolve().parent.parent / "recordings"

#: Frames per second for `--record`. `_sample_all` costs ~21 ms a pass, so 20 Hz
#: leaves room; hand motion has nothing above a few Hz in it anyway.
RECORD_HZ = 20.0


def record(args) -> int:
    """Capture the arm's pose over time BY HAND. Never enables torque.

    Same contract as `--ranges`: torque stays off, you move the arm, the
    encoders report where it went. The difference is that this keeps the whole
    time series rather than just the extremes.

        python tools/mb_bringup.py --record

    Saves continuously, for the reason written on `_write_ranges`: a recording
    is minutes of somebody moving an arm, and holding it in memory until the
    end means one bad exit throws all of it away.
    """
    mb, ctrl, motors = _open(args)
    try:
        _reporting(motors.values(), True)
        time.sleep(0.3)
        TRAJ_DIR.mkdir(parents=True, exist_ok=True)
        path = TRAJ_DIR / f"traj_{time.strftime('%Y%m%d_%H%M%S')}.json"

        print("TORQUE IS OFF AND STAYS OFF. Move the arm through the motion "
              "you want played back.\nCtrl-C when done.\n")
        print("Start from a pose you can return to -- replay begins by driving "
              "the arm to frame 0.\n")

        frames: list[dict] = []
        t0 = time.monotonic()
        period = 1.0 / RECORD_HZ
        next_frame = t0
        next_save = t0 + 2.0
        try:
            while True:
                now = time.monotonic()
                if now < next_frame:
                    time.sleep(min(0.005, next_frame - now))
                    continue
                next_frame += period
                sample = _sample_all(ctrl, motors)
                pos = {n: round(s["pos_deg"], 3)
                       for n, s in sample.items() if "pos_deg" in s}
                if pos:
                    frames.append({"t": round(time.monotonic() - t0, 4),
                                   "pos": pos})
                if time.monotonic() >= next_save:
                    _write_traj(path, frames)
                    next_save = time.monotonic() + 2.0
                if frames:
                    sys.stdout.write(
                        f"\r\033[2K{len(frames)} frames, "
                        f"{frames[-1]['t']:6.1f} s  " +
                        "  ".join(f"{n[:6]}{p:+7.1f}"
                                  for n, p in frames[-1]["pos"].items()))
                    sys.stdout.flush()
        except KeyboardInterrupt:
            pass

        _write_traj(path, frames)
        if len(frames) < 2:
            print("\n\nNothing recorded.", file=sys.stderr)
            return 1
        span = frames[-1]["t"]
        print(f"\n\n{len(frames)} frames over {span:.1f} s -> {path}")
        print(f"\n{'joint':<15}{'start':>9}{'end':>9}{'min':>9}{'max':>9}"
              f"{'travel':>9}")
        print("-" * 60)
        for name in CAN_IDS:
            vals = [f["pos"][name] for f in frames if name in f["pos"]]
            if not vals:
                continue
            print(f"{name:<15}{vals[0]:>9.2f}{vals[-1]:>9.2f}{min(vals):>9.2f}"
                  f"{max(vals):>9.2f}{max(vals) - min(vals):>9.2f}")
        print(f"\nReplay it with:\n"
              f"    python tools/mb_bringup.py --replay {path.name}")
        return 0
    finally:
        _reporting(motors.values(), False)
        _shutdown(ctrl)


def _write_traj(path: Path, frames: list[dict]) -> None:
    """Atomic, incremental. Same reasoning as `_write_ranges`."""
    if not frames:
        return
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(
        {"recorded": time.strftime("%Y-%m-%dT%H:%M:%S"),
         "hz": RECORD_HZ,
         "note": "hand-moved, torque off. Positions are degrees against the "
                 "zero in force when recorded -- a re-zero invalidates this.",
         "frames": frames}, separators=(",", ":")) + "\n")
    tmp.replace(path)


def _command(motor, args, target_deg: float, hold: float = 0.0) -> None:
    """Hold a position target by streaming it. Degrees in, radians on the wire.

    STREAMED, AND THAT IS MEASURED RATHER THAN STYLISTIC. A position mode here
    will not converge on a single frame, nor on two sent 30 ms apart. Holding
    one target and varying only the gap between two identical frames:

        gap 0.03 s  ->  1.50 deg short (never moved at all)
        gap 0.10 s  ->  1.35 deg short
        gap 0.20 s  ->  0.68 deg short
        gap 0.40 s  ->  landed, err +0.04 deg
        streamed at 20 Hz for 0.6 s -> landed, err +0.03 deg

    A single send leaves the joint one step short of target on every step,
    which reads exactly like a 1 deg steady-state droop. It is not: the true
    tracking error is 0.03 deg. The trap is that the obvious response to
    apparent droop is to raise the gain, and here that would mean chasing an
    artefact with a 36 N.m motor.

    Streaming also keeps the motor's CAN timeout fed, which matters most in
    MIT, where a stalled stream means dropped torque rather than a stale
    target.

    Three laws, and the default is the cautious one:

    `robstride_send_pos_vel_pp` is RobStride's *profile position* mode -- the
    motor interpolates to the target under its own velocity AND acceleration
    ceiling. That bounds the move inside the firmware, which is what you want
    for the arm's first motion: a Python loop at a 300 ms period cannot limit
    anything that happens between two iterations, and a sharp acceleration is
    exactly what trips the LRS-600 into hiccup mode.

    `..._csp` (cyclic synchronous position) takes only a velocity limit and
    expects to be streamed at a steady rate -- better for teleop, wrong for
    discrete bounded steps.

    `send_mit` has no ceiling of any kind. Opt-in only.
    """
    pos = math.radians(target_deg)
    vlim = math.radians(args.vlim_dps)
    acc = math.radians(args.acc_dpss)
    period = 1.0 / max(1.0, args.stream_hz)
    deadline = time.monotonic() + max(0.0, hold)
    while True:
        if args.mit:
            motor.send_mit(pos, 0.0, args.kp, args.kd, 0.0)
        elif args.csp:
            motor.robstride_send_pos_vel_csp(pos, vlim)
        else:
            motor.robstride_send_pos_vel_pp(pos, vlim, acc)
        if time.monotonic() >= deadline:
            return
        time.sleep(period)


def _reporting(motors, on: bool) -> None:
    """Turn RobStride active reporting on or off, per motor.

    MEASURED, not assumed, and it is the difference between a working readback
    and a motor that looks dead:

    `request_feedback()` + `poll_feedback_once()` returns NOTHING on these
    motors -- verified over 10 attempts against a motor that was provably alive
    (clean ping, fault report (0,0)). `robstride_set_active_report(True)` makes
    state arrive on the first poll.

    So a scan that only requests feedback reports a healthy arm as silent. That
    is a false abort on the most dangerous possible signal.

    !! IT IS ONE-SHOT. The note here used to say the motor "streams type-0x18
    frames at roughly 10 Hz once it is enabled". It does not. Measured
    2026-11-12, arm powered and answering 7/7: arm reporting once, then poll
    as hard as you like for six seconds -- 120 passes, every joint returns
    exactly ONE distinct (position, velocity) pair. Not stale by a few frames:
    frozen, velocity noise included. Re-arm on every pass and the same loop
    returns 62-77 distinct readings out of 78.

    Nothing caught this because nothing repeated a read. `scan` reads each
    motor once. `move_one` looks like it polls in a loop, but it is also
    streaming position commands, and those elicit their own state frames -- so
    it gets fresh data for a reason unrelated to this call.

    Any caller that reads the same motor twice must re-arm between reads. See
    `_sample_all`.
    """
    for motor in motors:
        try:
            motor.robstride_set_active_report(on)
        except Exception:  # noqa: BLE001 - absent motors raise; that is fine
            pass


def _poll(motor, retries: int, ctrl=None, drain: int = 16):
    """Drain the receive side until this motor's state lands.

    Assumes active reporting is already on -- see `_reporting`. The
    `request_feedback()` call is kept as a belt-and-braces nudge for a firmware
    revision that answers polls instead of streaming.
    """
    for _ in range(max(1, retries)):
        try:
            motor.request_feedback()
        except Exception:  # noqa: BLE001
            pass
        time.sleep(0.05)
        if ctrl is not None:
            # DRAIN THE BACKLOG, do not stop at the first frame.
            # `poll_feedback_once` moves exactly one frame into the state
            # cache, and the motor streams at ~10 Hz while a step takes 300 ms.
            # Reading after a single poll therefore returns a frame that is two
            # or three periods old. During motion that showed up as a constant
            # apparent tracking error of exactly one step size, flipping sign
            # with direction -- which looks like real droop and is not. Drain
            # until the socket is empty so `get_state` is the newest frame.
            for _ in range(drain):
                try:
                    ctrl.poll_feedback_once()
                except Exception:  # noqa: BLE001
                    break
        state = motor.get_state()
        if _state_dict(state):
            return state
    return None


def sweep(args) -> int:
    """Read-only: ping every id in a range and report who is really out there.

    The question this settles: are joints 2-7 on unexpected ids, or are they
    not reachable at all? Those need opposite fixes. Seeed ship the B601-RS as
    ids 1..7, but ids live in non-volatile motor memory and a replaced or
    factory-default motor sits wherever it was left -- exactly the trap the
    leader arm sprang, where salvaged servos arrived on donor ids.

    Nothing is energised and no id is written.
    """
    mb = _require_sdk()
    ctrl = mb.Controller(args.port)
    try:
        print(f"sweeping ids {args.sweep_min}..{args.sweep_max} on "
              f"{args.port}, host 0x{args.host_id:02X}. Read-only.\n")
        found: list[tuple[int, str]] = []
        for mid in range(args.sweep_min, args.sweep_max + 1):
            try:
                motor = ctrl.add_robstride_motor(mid, args.host_id, "rs-00")
            except Exception:  # noqa: BLE001
                continue
            try:
                res = motor.robstride_ping()
                found.append((mid, str(res)))
                known = next((j for j, i in CAN_IDS.items() if i == mid), None)
                label = f"  <- {known}" if known else "  <- NOT a B601 joint id"
                print(f"  id {mid:>3}  answered  {res}{label}")
            except Exception:  # noqa: BLE001
                pass
            if mid % 16 == 0:
                print(f"  ... {mid}", flush=True)

        print(f"\n{len(found)} motor(s) answered: "
              f"{', '.join(str(i) for i, _ in found) or 'none'}")
        expected = set(CAN_IDS.values())
        got = {i for i, _ in found}
        if got == expected:
            print("All seven present on the expected ids.")
            return 0
        if missing := sorted(expected - got):
            print(f"MISSING from the expected 1..7: {missing}")
        if extra := sorted(got - expected):
            print(f"On UNEXPECTED ids: {extra} -- these are probably the "
                  f"missing joints, left on a factory or donor id.")
        if not got - expected and missing:
            print("""
Nothing answered anywhere else either, so this is not an id problem -- those
motors are not electrically reachable. On a daisy-chained arm the usual cause
is a break or an unmated internal connector DOWNSTREAM of the last motor that
does answer: everything past it is cut off. Check the harness between the last
good joint and the next one.""")
        return 1
    finally:
        _shutdown(ctrl)


def _plan(home: float, amplitude: float, cycles: int,
          direction: str = "both") -> list[float]:
    """Out, back through home, the other way, back.

    Both directions is the better test and stays the default -- a one-sided
    move cannot show backlash or an asymmetric gain.

    But it is impossible for a joint resting against a stop, which is where
    `shoulder_lift` and `elbow_flex` sit in this arm's natural sitting pose:
    measured travel runs from ~0 upward, and that 0 is where gravity parks
    them. The alternative to one-sided is hand-lifting a 36 N.m joint and
    holding it steady while it is energised, which is worse. `--direction up`
    lifts away from the stop under the motor's own power and returns.
    """
    out: list[float] = []
    for _ in range(max(1, cycles)):
        if direction != "down":
            out += [home + amplitude, home]
        if direction != "up":
            out += [home - amplitude, home]
    return out


def _shutdown(ctrl) -> None:
    """`Controller` exposes close() and shutdown(); neither is a context
    manager in 0.3.9, so the caller's `finally` is the only cleanup there is."""
    try:
        ctrl.close()
    except Exception:  # noqa: BLE001
        try:
            ctrl.shutdown()
        except Exception:  # noqa: BLE001
            pass


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="can0", help="SocketCAN interface")
    p.add_argument("--host-id", type=lambda s: int(s, 0),
                   default=DEFAULT_HOST_ID,
                   help="master/feedback id (default 0xFD)")
    p.add_argument("--retries", type=int, default=5)

    action = p.add_mutually_exclusive_group()
    action.add_argument("--introspect", action="store_true",
                        help="dump the SDK's API surface; no CAN, no hardware")
    action.add_argument("--scan", action="store_true",
                        help="read-only bus scan (default)")
    action.add_argument("--sweep", action="store_true",
                        help="read-only: ping every id to find motors that are "
                             "not on their expected id")
    action.add_argument("--zero", action="store_true",
                        help="set zero to the present pose on all 7")
    action.add_argument("--move", action="store_true",
                        help="apply torque to ONE joint")
    action.add_argument("--clear-faults", action="store_true",
                        help="clear latched faults on all 7; no torque")
    action.add_argument("--ranges", action="store_true",
                        help="record travel by hand; torque stays OFF. "
                             "--joint records that one and keeps the rest")
    action.add_argument("--record", action="store_true",
                        help="capture a hand-moved trajectory; torque OFF")
    action.add_argument("--replay", metavar="FILE",
                        help="drive a recorded trajectory. ALL joints, "
                             "under power")
    action.add_argument("--gravity-map", action="store_true",
                        help="walk --joint to --to-deg and record what every "
                             "joint carried. ALL joints under power. This is "
                             "what sets the torque ceilings from measurement "
                             "instead of from the datasheet")
    action.add_argument("--fit-gravity", action="store_true",
                        help="fit the pose-aware gravity model from samples "
                             "logged during teleop. Touches NO hardware, "
                             "moves nothing. Run it after a session")

    p.add_argument("--joint", choices=sorted(CAN_IDS))
    p.add_argument("--joints", metavar="A,B",
                   help="--replay only: restrict to these joints")
    p.add_argument("--speed", type=float, default=1.0,
                   help="--replay only: <1 plays slower. Never plays faster "
                        "than --vlim-dps allows")
    p.add_argument("--amplitude", type=float, default=5.0,
                   help="degrees either side of the present position")
    p.add_argument("--direction", choices=("both", "up", "down"),
                   default="both",
                   help="which way to test. 'both' is the better check and "
                        "the default; use one-sided for a joint resting "
                        "against a stop")
    p.add_argument("--max-step-deg", type=float, default=1.0)
    p.add_argument("--vlim-dps", type=float, default=20.0,
                   help="POS_VEL velocity limit, deg/s")
    p.add_argument("--acc-dpss", type=float, default=90.0,
                   help="POS_VEL acceleration limit, deg/s^2")
    p.add_argument("--kp", type=float, default=8.0)
    p.add_argument("--kd", type=float, default=1.0, help="MIT only")
    p.add_argument("--settle", type=float, default=0.3,
                   help="seconds to stream each target before measuring")
    p.add_argument("--stream-hz", type=float, default=20.0,
                   help="rate at which the target is re-sent while held")
    p.add_argument("--to-deg", type=float, default=None,
                   help="--gravity-map only: angle to walk --joint to. Must "
                        "be inside its measured travel")
    p.add_argument("--abort-deg", type=float, default=15.0)
    p.add_argument("--hold-on-fault", type=float, default=-1.0,
                   help="seconds to hold position after an abort. Default -1 "
                        "holds INDEFINITELY, until --hold-max-temp or Ctrl-C. "
                        "0 releases immediately, which DROPS an extended arm")
    p.add_argument("--hold-max-temp", type=float, default=STALL_TEMP_C,
                   help="degC at which an indefinite hold gives up and "
                        "releases. Holding is not free -- a loaded joint "
                        "standing still dissipates that load continuously")
    p.add_argument("--max-torque", type=float, default=None,
                   help="N.m ceiling, overriding BOTH the gravity-aware warn "
                        "ceiling and the hard one. Default warn is half rated "
                        "(rs-06 18.0, rs-00 7.0) raised to cover anything "
                        "--gravity-map measured; default hard is 85%% of "
                        "rated (rs-06 30.6, rs-00 11.9)")
    p.add_argument("--max-temp", type=float, default=STALL_TEMP_C,
                   help="degC abort ceiling")
    p.add_argument("--no-gravity-model", action="store_true",
                   help="ignore a fitted gravity_model.json and guard on the "
                        "fixed ceilings instead. For comparing the two, or if "
                        "a bad fit is misbehaving -- note the fixed ceilings "
                        "are blind in folded poses, which is why the model "
                        "exists")
    p.add_argument("--skip-range-check", action="store_true",
                   help="energise a joint whose travel was never recorded")
    p.add_argument("--cycles", type=int, default=2)
    p.add_argument("--sweep-min", type=int, default=1)
    p.add_argument("--sweep-max", type=int, default=127)
    p.add_argument("--mode-timeout-ms", type=int, default=1000)
    p.add_argument("--csp", action="store_true",
                   help="cyclic synchronous position instead of the profile "
                        "mode: velocity limit only, no acceleration ceiling")
    p.add_argument("--mit", action="store_true",
                   help="use MIT instead of POS_VEL. The motor then enforces "
                        "NO velocity or position limit.")
    p.add_argument("--dry-run", action="store_true",
                   help="print the plan, enable nothing")
    args = p.parse_args()

    if args.mit and args.csp:
        p.error("--mit and --csp are different control laws; pick one")

    if args.introspect:
        return introspect()
    if args.sweep:
        return sweep(args)
    if args.zero:
        return zero(args)
    if args.clear_faults:
        return clear_faults(args)
    if args.ranges:
        return ranges(args)
    if args.record:
        return record(args)
    if args.replay:
        return replay(args)
    if args.fit_gravity:
        return fit_gravity(args)
    if args.gravity_map:
        if not args.joint:
            p.error("--gravity-map needs --joint")
        return gravity_map(args)
    if args.move:
        if not args.joint:
            p.error("--move needs --joint")
        return move_one(args)
    return scan(args)


if __name__ == "__main__":
    raise SystemExit(main())
