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
import math
import sys
import time

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


def _open(args):
    """Controller + the seven motor handles. Enables nothing."""
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
            print("\nFAULTS REPORTED -- clear these before energising:")
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
                  "  - can0 is up but not actually wired to the arm, or the "
                  "120 ohm termination is missing\n"
                  "  - arm not powered (48 V)\n"
                  "  - motors are in RobStride's private protocol rather than "
                  "MIT comm mode\n"
                  "  - wrong host id: try --host-id 0xFF / 0xFE / 0x00 / 0xAA\n"
                  "Confirm frames exist at all with:  candump -tz can0")
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
        print("\nRe-run --scan: every joint should now read within a degree "
              "of 0.")
        return 0
    finally:
        _shutdown(ctrl)


def move_one(args) -> int:
    """First torque. One joint, bounded steps, velocity-limited by the motor."""
    name = args.joint
    if name in SHOULDER_JOINTS:
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
        plan = _plan(home, args.amplitude, args.cycles)
        mode_name = {MODE_MIT: "MIT (no motor-side limits!)",
                     MODE_CSP: "CSP", MODE_POS_VEL: "POS_VEL profile"}[
                         _mode_of(args)]
        print(f"{name}: id {CAN_IDS[name]}, model {MOTOR_MODELS[name]}, "
              f"mode {mode_name}")
        print(f"home {home:+.2f} deg, plan: "
              + " -> ".join(f"{p:+.1f}" for p in plan))
        print(f"steps <= {args.max_step_deg} deg, vlim {args.vlim_dps} dps, "
              f"kp {args.kp}, abort at {args.abort_deg} deg error")

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

        print(f"\nworst tracking error: {worst:.2f} deg")
        if worst > 2.0:
            print("Tracking is loose. Raise --kp a little, or the joint is "
                  "fighting gravity with too little gain.")
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
    state arrive on the first poll. The motor streams type-0x18 frames at
    roughly 10 Hz once it is enabled and is otherwise silent.

    So a scan that only requests feedback reports a healthy arm as silent. That
    is a false abort on the most dangerous possible signal.
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
    if args.move:
        if not args.joint:
            p.error("--move needs --joint")
        return move_one(args)
    return scan(args)


if __name__ == "__main__":
    raise SystemExit(main())
