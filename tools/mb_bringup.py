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

#: Rated continuous torque, N.m, from Seeed's spec. Halved for the abort
#: ceiling below: a bounded few-degree bring-up move on an arm that is not
#: carrying a payload has no business approaching half of rated.
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
STALL_TORQUE_FRAC = 0.5
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
        tq_ceiling = (args.max_torque if args.max_torque is not None
                      else rated * STALL_TORQUE_FRAC)
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
            room = min(room_up, room_dn)
            if room < args.amplitude:
                if room < 0.5:
                    print(f"\nREFUSING: {name} is at {home:+.2f} deg with "
                          f"measured travel {rng['min_deg']:+.1f}..."
                          f"{rng['max_deg']:+.1f}.\n"
                          f"There is {room:+.2f} deg of room inside a "
                          f"{RANGE_MARGIN_DEG:.0f} deg margin -- it is "
                          f"already at a stop.\nMove the arm somewhere with "
                          f"room before energising this joint.",
                          file=sys.stderr)
                    return 2
                print(f"!! clamping amplitude {args.amplitude:.1f} -> "
                      f"{room:.1f} deg: measured travel "
                      f"{rng['min_deg']:+.1f}...{rng['max_deg']:+.1f}, "
                      f"home {home:+.2f}, {RANGE_MARGIN_DEG:.0f} deg margin.")
                args.amplitude = room

        plan = _plan(home, args.amplitude, args.cycles)
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


def _plan(home: float, amplitude: float, cycles: int) -> list[float]:
    """Out, back through home, the other way, back. Both directions matter --
    a one-sided test cannot show backlash or an asymmetric gain."""
    out: list[float] = []
    for _ in range(max(1, cycles)):
        out += [home + amplitude, home, home - amplitude, home]
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

    p.add_argument("--joint", choices=sorted(CAN_IDS))
    p.add_argument("--amplitude", type=float, default=5.0,
                   help="degrees either side of the present position")
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
    p.add_argument("--abort-deg", type=float, default=15.0)
    p.add_argument("--max-torque", type=float, default=None,
                   help="N.m abort ceiling; default is half rated for the "
                        "joint's model (rs-06 18.0, rs-00 7.0)")
    p.add_argument("--max-temp", type=float, default=STALL_TEMP_C,
                   help="degC abort ceiling")
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
    if args.move:
        if not args.joint:
            p.error("--move needs --joint")
        return move_one(args)
    return scan(args)


if __name__ == "__main__":
    raise SystemExit(main())
