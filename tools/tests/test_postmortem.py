"""Synthesise each way the arm can drop; check the postmortem names it.

No hardware. Each case writes a CSV in the shape the flight recorder writes,
runs mb_postmortem over it, and asserts on the verdict text.

The cases are the four distinguishable shapes, plus the two already-observed
real events replayed from their known numbers.
"""
import io
import subprocess
import tempfile
import sys
from contextlib import redirect_stdout
from pathlib import Path

HERE = Path(tempfile.mkdtemp(prefix="pm_"))
TOOLS = str(Path(__file__).resolve().parent.parent)
PYTHON = sys.executable
sys.path.insert(0, TOOLS)
import mb_bringup as mb  # noqa: E402
import mb_postmortem as pm  # noqa: E402

JOINTS = list(mb.CAN_IDS)
results = []


def check(label, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {label:<52} {detail}")
    results.append(ok)


def write_log(name, frames):
    """frames: list of (event, {joint: (lead,tgt,pos,vel,tq,tmp,st)})"""
    path = HERE / f"t_pm_{name}.csv"
    cols = ["t", "frame", "event"]
    for j in JOINTS:
        cols += [f"{j}_{f}" for f in mb.REC_FIELDS]
    with path.open("w") as fh:
        fh.write(",".join(cols) + "\n")
        for i, (ev, per) in enumerate(frames):
            row = [f"{i * 0.05:.3f}", str(i), ev]
            for j in JOINTS:
                v = per.get(j)
                row += ["" if x is None else f"{x}" for x in
                        (v if v else [None] * 7)]
            fh.write(",".join(row) + "\n")
    return path


def run(path):
    buf = io.StringIO()
    with redirect_stdout(buf):
        pm.main_argv = None
        rows, joints = pm.load(path)
        pm.describe_run(rows, joints)
        pm.total_load(rows, joints)
        pm.verdict(rows, joints)
    return buf.getvalue()


def nominal(tq_lift=12.0, pos=100.0, vel=0.0):
    """A healthy frame: everything loaded and tracking."""
    per = {}
    for j in JOINTS:
        t = tq_lift if j == "shoulder_lift" else (6.0 if j == "elbow_flex"
                                                  else 2.0)
        per[j] = (50.0, pos, pos, vel, t, 38.0, 0)
    return per


# --- 1. the rail went away ---------------------------------------------------
print("\n--- 1. simultaneous collapse = PSU / rail ----------------------")
frames = [("start", nominal()) for _ in range(30)]
dead = {j: (50.0, 100.0, 100.0, -60.0, 0.05, 38.0, 0) for j in JOINTS}
frames += [("", dead) for _ in range(10)]
out = run(write_log("rail", frames))
check("names a simultaneous collapse", "TORQUE COLLAPSED ON" in out)
check("points at the shared 48 V rail", "48 V rail" in out)
check("names the LRS-600-48 and hiccup mode",
      "LRS-600-48" in out and "hiccup" in out)
check("does NOT blame a guard", "A GUARD ABORTED" not in out)

# --- 2. one motor latched its own protection --------------------------------
print("\n--- 2. one joint alone = that motor's protection ---------------")
frames = [("start", nominal()) for _ in range(30)]
for _ in range(10):
    per = nominal()
    per["elbow_flex"] = (50.0, 100.0, 100.0, -70.0, 0.02, 62.0, 16)
    frames.append(("", per))
out = run(write_log("onemotor", frames))
check("names the single joint", "elbow_flex LOST TORQUE ALONE" in out)
check("attributes it to the motor, not the rail",
      "own protection" in out and "not the rail" in out)
check("suggests --clear-faults", "--clear-faults" in out)

# --- 3. a guard aborted ------------------------------------------------------
print("\n--- 3. a guard abort is OUR decision ---------------------------")
frames = [("start", nominal()) for _ in range(20)]
frames.append(("abort:shoulder_lift:stall:+24.10:err4.20", nominal(24.1)))
frames.append(("freeze:forever", nominal(19.0)))
frames += [("", nominal(19.0)) for _ in range(10)]
frames.append(("release:temp:shoulder_lift:50", nominal(19.0)))
out = run(write_log("abort", frames))
check("names the guard abort", "A GUARD ABORTED" in out)
check("quotes the abort reason", "stall" in out)
check("says it was our decision", "our decision" in out)
check("reports what ended the freeze", "release:temp" in out)
check("flags it is not a controlled descent",
      "not a\ncontrolled descent" in out or "controlled descent" in out)

# --- 4. leader dropout -------------------------------------------------------
print("\n--- 4. leader dropout is the CH343, not the arm ----------------")
frames = [("start", nominal()) for _ in range(20)]
frames.append(("leader_lost:ConnectionError", nominal()))
frames += [("", nominal()) for _ in range(5)]
out = run(write_log("leader", frames))
check("names the leader", "LEADER STOPPED ANSWERING" in out)
check("identifies the CH343 / cable / EMI",
      "CH343" in out and "EMI" in out)
check("exonerates the follower", "follower was fine" in out)

# --- 5. mechanical runaway: torque present, position gone -------------------
print("\n--- 5. torque present but position runs away -------------------")
frames = [("start", nominal()) for _ in range(20)]
for k in range(10):
    per = nominal()
    # Still 14 N.m, but 40 deg off target and moving fast.
    per["shoulder_lift"] = (50.0, 100.0, 100.0 - 40 - k * 5, -80.0, 14.0,
                            41.0, 0)
    frames.append(("", per))
out = run(write_log("runaway", frames))
check("names the runaway", "RAN AWAY FROM ITS TARGET" in out)
check("says torque was still present", "Torque did not go away" in out)
check("calls it mechanical", "mechanical" in out)

# --- 6. a clean run must not invent a cause ---------------------------------
print("\n--- 6. a clean run gets no false diagnosis ---------------------")
frames = [("start", nominal()) for _ in range(60)]
frames.append(("stop:operator", nominal()))
out = run(write_log("clean", frames))
check("reports no drop found", "NO DROP FOUND IN THIS LOG" in out)
check("invents no electrical cause", "48 V rail" not in out)
check("invents no guard abort", "A GUARD ABORTED" not in out)

# --- 7. a joint that never answered is not a collapse -----------------------
print("\n--- 7. a silent joint is reported, not diagnosed ---------------")
frames = []
for _ in range(40):
    per = nominal()
    del per["gripper"]           # never answered at all
    frames.append(("", per))
out = run(write_log("silent", frames))
check("flags the joint as never answering", "never answered" in out)
check("does not call it a drop", "NO DROP FOUND" in out)

# --- 8. runs as a CLI, exits 0 ----------------------------------------------
print("\n--- 8. the actual CLI works ------------------------------------")
r = subprocess.run(
    [PYTHON,
     f"{TOOLS}/mb_postmortem.py", str(HERE / "t_pm_rail.csv")],
    capture_output=True, text=True)
check("CLI exits 0", r.returncode == 0, r.stderr[-120:] if r.stderr else "")
check("CLI prints the verdict", "TORQUE COLLAPSED ON" in r.stdout)

r = subprocess.run(
    [PYTHON,
     f"{TOOLS}/mb_postmortem.py", str(HERE / "t_pm_abort.csv"), "--frame", "21"],
    capture_output=True, text=True)
check("--frame dumps context and skips the verdict",
      r.returncode == 0 and "VERDICT" not in r.stdout)

r = subprocess.run(
    [PYTHON,
     f"{TOOLS}/mb_postmortem.py", str(HERE / "nope.csv")],
    capture_output=True, text=True)
check("missing file exits 1 without a traceback",
      r.returncode == 1 and "Traceback" not in r.stderr)

for p in HERE.glob("t_pm_*.csv"):
    p.unlink()
print(f"\n{sum(results)}/{len(results)} passed")
sys.exit(0 if all(results) else 1)
