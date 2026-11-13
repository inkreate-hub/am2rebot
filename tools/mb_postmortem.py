#!/usr/bin/env python3
"""Read a teleop flight-recorder CSV and say why the arm dropped.

    mb_postmortem.py recordings/teleop_20261113_181500.csv

Touches no hardware, no CAN bus, and no motor. Pure analysis of a log.

WHY THIS EXISTS AS A TOOL RATHER THAN AS EYEBALLS ON A CSV

The arm has fallen three times and all three diagnoses were reconstructed
afterwards from whatever happened to be on screen. That worked only because
the causes were in the software, and the software was printing. The remaining
causes are electrical and silent -- a PSU hiccup, a motor latching its own
protection, a rail dip -- and they announce themselves by *stopping*, which
leaves nothing to read.

What separates them is not any single number but the SHAPE of the collapse
across joints and time:

    all seven lose torque in the same frame    the rail went away. Nothing
                                               downstream chose this.
    one joint loses torque, status changes     that motor's own protection
                                               latched. The bus is fine.
    torque falls because we disabled it        a guard aborted, and the log
                                               says which guard and why.
    torque is present and position still       mechanical: the joint is
    runs away                                  slipping, or the load exceeds
                                               what it can hold.

So the job is to find the moment torque disappeared, decide which of those
shapes it was, and print the frames either side of it. That is mechanical
enough to automate and tedious enough to get wrong by hand at 20 Hz.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

#: Truncating the real names to column width gives `should should elbow_`, and
#: the two shoulders become indistinguishable in the one table you read to tell
#: them apart.
SHORT = {"shoulder_pan": "pan", "shoulder_lift": "lift", "elbow_flex": "elbow",
         "wrist_flex": "wflex", "wrist_yaw": "wyaw", "wrist_roll": "wroll",
         "gripper": "grip"}

#: Below this, a joint is treated as carrying nothing, N.m. Not zero: a
#: disabled RobStride still reports small nonzero torque from drivetrain drag.
DEAD_NM = 0.35

#: A joint is "loaded" above this, N.m. The collapse detector only looks at
#: joints that had something to lose -- an unloaded wrist reading 0.1 N.m has
#: not dropped anything, and counting it dilutes the signal.
LOADED_NM = 1.5

#: How many joints must lose torque within `SIMUL_FRAMES` for it to be read as
#: a rail event rather than a per-motor one.
SIMUL_JOINTS = 3
SIMUL_FRAMES = 3

#: Degrees of unexplained travel that counts as the arm actually falling, and
#: the speed that distinguishes falling from being moved.
FALL_DEG = 12.0
FALL_DPS = 25.0


def _f(row: dict, key: str) -> float | None:
    v = row.get(key, "")
    if v is None or v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


def load(path: Path) -> tuple[list[dict], list[str]]:
    with path.open(newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit(f"{path} has a header but no frames.")
    joints = sorted({c[:-4] for c in rows[0] if c.endswith("_pos")},
                    key=lambda j: list(rows[0]).index(j + "_pos"))
    return rows, joints


def describe_run(rows: list[dict], joints: list[str]) -> None:
    t_end = _f(rows[-1], "t") or 0.0
    print(f"{len(rows)} frames over {t_end:.1f} s "
          f"({len(rows) / max(t_end, 1e-3):.1f} Hz), {len(joints)} joints "
          f"logged\n")

    print(f"{'joint':<15}{'peak |tq|':>11}{'at t':>8}{'peak T':>8}"
          f"{'travel':>9}{'answered':>10}")
    print("-" * 61)
    for j in joints:
        tqs = [(abs(v), _f(r, "t") or 0.0)
               for r in rows if (v := _f(r, j + "_tq")) is not None]
        poss = [v for r in rows if (v := _f(r, j + "_pos")) is not None]
        tmps = [v for r in rows if (v := _f(r, j + "_tmp")) is not None]
        if not tqs:
            print(f"{j:<15}{'--- never answered ---':>46}")
            continue
        peak, at = max(tqs)
        print(f"{j:<15}{peak:>11.2f}{at:>8.1f}"
              f"{(max(tmps) if tmps else 0):>8.0f}"
              f"{(max(poss) - min(poss) if poss else 0):>9.1f}"
              f"{len(tqs) / len(rows) * 100:>9.0f}%")


def events(rows: list[dict]) -> list[tuple[int, float, str]]:
    out = []
    for i, r in enumerate(rows):
        ev = (r.get("event") or "").strip()
        if ev:
            out.append((i, _f(r, "t") or 0.0, ev))
    return out


def total_load(rows: list[dict], joints: list[str]) -> None:
    """Sum of |torque| over time -- a stand-in for current draw.

    WHY THIS IS WORTH PRINTING. The LRS-600-48 is 48 V / 12.5 A / 600 W,
    exactly Seeed's spec, so there is no headroom, and its overload behaviour
    is HICCUP mode: it drops the rail, waits, and retries. Seven motors
    holding an extended arm is the worst case the supply ever sees. If the sum
    peaks hard in the frames before every joint goes quiet, the supply is the
    suspect and no amount of guard tuning will help.

    Torque is not current, and the constant relating them is not measured on
    this arm. So this is a SHAPE, not an ampere figure -- read the spike, not
    the value.
    """
    series = []
    for r in rows:
        vals = [abs(v) for j in joints if (v := _f(r, j + "_tq")) is not None]
        if vals:
            series.append((_f(r, "t") or 0.0, sum(vals), len(vals)))
    if not series:
        return
    peak_t, peak_v, _ = max(series, key=lambda s: s[1])
    print(f"\npeak total |torque| {peak_v:.1f} N.m across all joints, "
          f"at t={peak_t:.1f} s")
    print("  A proxy for current draw, not an ampere figure. The LRS-600-48 "
          "has no headroom and\n  hiccups under overload, so a spike here "
          "immediately before a collapse is a supply event.")


def find_collapse(rows: list[dict], joints: list[str]) -> dict | None:
    """First frame where loaded joints go dead. Returns a verdict dict."""
    was_loaded: dict[str, int] = {}
    for i, r in enumerate(rows):
        died = []
        for j in joints:
            tq = _f(r, j + "_tq")
            if tq is None:
                continue
            if abs(tq) >= LOADED_NM:
                was_loaded[j] = i
            elif abs(tq) < DEAD_NM and j in was_loaded:
                if i - was_loaded[j] <= SIMUL_FRAMES:
                    died.append(j)
                del was_loaded[j]
        if died:
            # Widen: how many joints died within SIMUL_FRAMES of this one?
            window = rows[max(0, i - SIMUL_FRAMES):i + SIMUL_FRAMES + 1]
            all_died = set(died)
            for w in window:
                for j in joints:
                    tq = _f(w, j + "_tq")
                    if tq is not None and abs(tq) < DEAD_NM:
                        hist = [abs(v) for r2 in rows[:i]
                                if (v := _f(r2, j + "_tq")) is not None]
                        if hist and max(hist[-20:] or [0]) >= LOADED_NM:
                            all_died.add(j)
            return {"frame": i, "t": _f(rows[i], "t") or 0.0,
                    "joints": sorted(all_died)}
    return None


def find_fall(rows: list[dict], joints: list[str]) -> dict | None:
    """Position running away from target at speed -- the arm actually moving."""
    for i, r in enumerate(rows):
        for j in joints:
            pos, tgt, vel = (_f(r, j + "_pos"), _f(r, j + "_tgt"),
                             _f(r, j + "_vel"))
            if None in (pos, tgt, vel):
                continue
            if abs(pos - tgt) > FALL_DEG and abs(vel) > FALL_DPS:
                return {"frame": i, "t": _f(r, "t") or 0.0, "joint": j,
                        "err": pos - tgt, "vel": vel, "pos": pos}
    return None


def context(rows: list[dict], joints: list[str], frame: int,
            before: int = 6, after: int = 4) -> None:
    lo, hi = max(0, frame - before), min(len(rows), frame + after + 1)
    cols = [j for j in joints
            if any(_f(r, j + "_tq") is not None for r in rows[lo:hi])]
    print(f"\nframes {lo}..{hi - 1}, torque per joint (* = the frame in "
          f"question)\n")
    head = f"{'':1}{'t':>8}{'frame':>7}"
    for j in cols:
        head += f"{SHORT.get(j, j[:6]):>8}"
    print(head + "  event")
    print("-" * (len(head) + 10))
    for i in range(lo, hi):
        r = rows[i]
        mark = "*" if i == frame else " "
        line = f"{mark}{_f(r, 't') or 0:>8.2f}{i:>7}"
        for j in cols:
            tq = _f(r, j + "_tq")
            line += f"{'    --  ' if tq is None else f'{tq:>8.2f}'}"
        print(line + "  " + (r.get("event") or ""))


def verdict(rows: list[dict], joints: list[str]) -> None:
    evs = events(rows)
    aborts = [e for e in evs if e[2].startswith("abort")]
    releases = [e for e in evs if e[2].startswith("release")]
    leader = [e for e in evs if e[2].startswith("leader_lost")]
    collapse = find_collapse(rows, joints)
    fall = find_fall(rows, joints)

    print("\n" + "=" * 70)
    print("VERDICT")
    print("=" * 70)

    if evs:
        print("\nevents:")
        for _, t, ev in evs:
            print(f"  t={t:>7.2f}  {ev}")

    # Order matters: the software causes are checkable facts, so rule them out
    # before reaching for an electrical explanation that cannot be proven from
    # this log alone.
    if aborts:
        i, t, ev = aborts[0]
        print(f"\nA GUARD ABORTED at t={t:.2f}: {ev}")
        print("This was our decision, not the hardware's. The freeze that "
              "follows is the designed\nresponse; if the arm still fell, the "
              "question is what ended the freeze -- see the\nrelease event "
              "above.")
        context(rows, joints, i)
    elif leader:
        i, t, ev = leader[0]
        print(f"\nTHE LEADER STOPPED ANSWERING at t={t:.2f}: {ev}")
        print("A USB/serial fault on the leader side. The follower was fine. "
              "This is the known\nCH343 dropout -- cable, connector or EMI, "
              "not the arm.")
        context(rows, joints, i)
    elif collapse and len(collapse["joints"]) >= SIMUL_JOINTS:
        print(f"\nTORQUE COLLAPSED ON {len(collapse['joints'])} JOINTS AT "
              f"ONCE, t={collapse['t']:.2f}")
        print("  " + ", ".join(collapse["joints"]))
        print("\nNothing in software disables several motors in the same "
              "frame -- an abort would have\nlogged an event, and a freeze "
              "re-targets rather than releasing. A simultaneous loss\nacross "
              "joints points at the thing they SHARE: the 48 V rail.")
        print("\nThe LRS-600-48 is 600 W with no headroom and hiccup-mode "
              "overload protection.\nCheck the total-torque peak above, and "
              "watch the PSU LED on the next run.")
        context(rows, joints, collapse["frame"])
    elif collapse:
        print(f"\n{', '.join(collapse['joints'])} LOST TORQUE ALONE at "
              f"t={collapse['t']:.2f}")
        print("One joint, while the others kept working. That is the motor's "
              "own protection\nlatching -- overtemperature or overcurrent -- "
              "not the rail and not a guard.")
        print("Check its status word in the frames below, and its temperature "
              "in the table above.\n`mb_bringup.py --clear-faults` is the "
              "recovery; read the fault word before clearing it.")
        context(rows, joints, collapse["frame"])
    elif fall:
        print(f"\n{fall['joint']} RAN AWAY FROM ITS TARGET at "
              f"t={fall['t']:.2f}")
        print(f"{fall['err']:+.1f} deg off target, moving {fall['vel']:+.0f} "
              f"deg/s, while torque was still present.")
        print("Torque did not go away, so this is mechanical: the load "
              "exceeds what the joint can\nhold, or it is slipping.")
        context(rows, joints, fall["frame"])
    else:
        print("\nNO DROP FOUND IN THIS LOG.")
        print("No guard abort, no leader loss, no torque collapse, and no "
              "position runaway.")
        if releases:
            print(f"The run ended with: {releases[-1][2]}")
        print("If the arm fell during this run, the frames are not showing "
              "it -- check that the\nrecorder was running at the time, and "
              "that the sample rate was high enough to see it.")

    if releases:
        last = releases[-1]
        print(f"\nTorque was finally released by: {last[2]} at "
              f"t={last[1]:.2f}")
        if last[2].startswith("release:temp"):
            print("The freeze hit its temperature limit and let go. That is "
                  "the documented behaviour\nand it is still not a "
                  "controlled descent -- the arm goes limp where it is.")


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log", type=Path, help="teleop CSV from the flight recorder")
    p.add_argument("--frame", type=int, default=None,
                   help="dump context around this frame instead of the "
                        "automatic verdict")
    p.add_argument("--before", type=int, default=6)
    p.add_argument("--after", type=int, default=4)
    args = p.parse_args()

    if not args.log.exists():
        print(f"no such log: {args.log}", file=sys.stderr)
        return 1
    rows, joints = load(args.log)
    print(f"{args.log}\n")
    describe_run(rows, joints)
    total_load(rows, joints)
    if args.frame is not None:
        context(rows, joints, args.frame, args.before, args.after)
        return 0
    verdict(rows, joints)
    return 0


if __name__ == "__main__":
    sys.exit(main())
