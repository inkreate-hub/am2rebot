"""Offline check of the pose-aware gravity guard. No CAN, no hardware.

Builds a SYNTHETIC two-link arm with known masses, generates samples the way
teleop would, fits the model, and then asks the guard the two questions that
matter:

  1. Does it still abort on a real obstruction in a LIGHT pose -- the case the
     fixed 18 N.m ceiling is blind to, because gravity explains ~0 N.m there
     and 17 N.m of obstruction is still under 18?
  2. Does it stay quiet in the EXTENDED pose that actually dropped the arm on
     2026-11-12 -- lift 136.7 / elbow 143.7, shoulder at 19.28 N.m?

Nothing here can write to calibration/: every path mb_bringup would write to
is redirected into a fresh temp directory before anything runs.
"""
import json
import math
import random
import sys
import tempfile
from argparse import Namespace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import mb_bringup as mb  # noqa: E402

HERE = Path(tempfile.mkdtemp(prefix="posemodel_"))
mb.GRAVITY_MODEL_PATH = HERE / "gravity_model.json"
mb.GRAVITY_SAMPLES_PATH = HERE / "gravity_samples.jsonl"
mb.GRAVITY_PATH = HERE / "gravity_torque.json"

results = []


def check(label, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {label:<54} {detail}")
    results.append(ok)


# --- a synthetic arm whose true torques we know ------------------------------
# Three links in the pitch plane. The phases are chosen so that the shoulder
# reads 19.28 N.m at lift 136.7 / elbow 143.7 -- the exact pose and the exact
# torque that dropped the real arm -- because the point of the test is that
# that reading is NORMAL. The amplitudes then put the light end of the range
# near zero, and it is that spread the scalar ceiling cannot straddle.
A_L, A_E, A_W = 12.0, 6.5, 0.8
PH_L, PH_E, PH_W = -2.386, 1.389, 0.4
BIAS = 0.3


def true_torque(name, lift, elbow, wflex):
    t1 = math.radians(lift)
    t2 = t1 + math.radians(elbow)
    t3 = t2 + math.radians(wflex)
    terms = {
        "shoulder_lift": [(A_L, PH_L, t1), (A_E, PH_E, t2), (A_W, PH_W, t3)],
        "elbow_flex": [(A_E, PH_E, t2), (A_W, PH_W, t3)],
        "wrist_flex": [(A_W, PH_W, t3)],
    }
    return sum(a * math.cos(th + ph) for a, ph, th in terms[name]) + BIAS


# --- generate samples the way a teleop session would -------------------------
random.seed(7)
with mb.GRAVITY_SAMPLES_PATH.open("w") as fh:
    for _ in range(600):
        lift = random.uniform(-10, 140)
        elbow = random.uniform(-5, 150)
        wflex = random.uniform(-60, 60)
        rec = {"shoulder_lift": lift, "elbow_flex": elbow, "wrist_flex": wflex}
        for j in mb.PITCH_CHAIN:
            # +/- 0.15 N.m of sensor noise and stiction.
            rec[j + "_tq"] = true_torque(j, lift, elbow, wflex) \
                + random.gauss(0, 0.15)
        fh.write(json.dumps(rec) + "\n")

print("\n--- fit -------------------------------------------------------")
rc = mb.fit_gravity(Namespace())
check("fit_gravity returned 0", rc == 0, f"rc={rc}")
model = json.loads(mb.GRAVITY_MODEL_PATH.read_text())["joints"]
check("all three chain joints fitted", len(model) == 3, f"{sorted(model)}")
mb._GMODEL_CACHE = model

print("\n--- the model reproduces the arm ------------------------------")
worst = 0.0
for lift, elbow, wflex in [(0, 0, 0), (136.7, 143.7, 0), (90, 20, -40),
                           (45, 120, 55), (120, -5, 10)]:
    pose = {"shoulder_lift": lift, "elbow_flex": elbow, "wrist_flex": wflex}
    for j in mb.PITCH_CHAIN:
        worst = max(worst, abs(mb._expected_torque(j, pose, model)
                               - true_torque(j, lift, elbow, wflex)))
check("predicts within 0.1 N.m across the workspace", worst < 0.1,
      f"worst {worst:.3f} N.m")

lim = mb._residual_limit("shoulder_lift", model)
check("residual limit landed on the 3.0 N.m floor", abs(lim - 3.0) < 1e-6,
      f"{lim:.2f} N.m")

# --- the guard ---------------------------------------------------------------
BASE = dict(max_torque=None, max_temp=50.0, abort_deg=15.0, stream_hz=20.0,
            no_gravity_model=False)


def ask(lift, elbow, wflex, tq_lift, err=0.0, vel=0.0, **over):
    """Guard verdict for a pose with `tq_lift` on the shoulder."""
    s = {}
    for j, ang in zip(mb.PITCH_CHAIN, (lift, elbow, wflex)):
        s[j] = {"pos_deg": ang, "vel_dps": vel, "temp": 35.0,
                "torque": tq_lift if j == "shoulder_lift"
                else true_torque(j, lift, elbow, wflex)}
    targets = {j: a for j, a in zip(mb.PITCH_CHAIN, (lift, elbow, wflex))}
    targets["shoulder_lift"] = lift + err
    return mb._guard(None, {}, Namespace(**{**BASE, **over}), targets,
                     sampled=s, warned=set())


print("\n--- 1. the blind spot: obstruction in a LIGHT pose -------------")
# Found by search rather than asserted, so the test cannot quietly pick a pose
# that flatters the model.
light = min(((a, b, 0.0) for a in range(-10, 141, 2)
             for b in range(-5, 151, 2)),
            key=lambda p: abs(true_torque("shoulder_lift", *p)))
grav = true_torque("shoulder_lift", *light)
check("a light pose exists in travel", abs(grav) < 3.0,
      f"lift {light[0]} elbow {light[1]} -> {grav:+.2f} N.m")
check("17 N.m obstruction there -> ABORT (model)",
      ask(*light, tq_lift=17.0, err=4.0, vel=0.0) == 1)
check("...and the fixed ceiling MISSES it, as argued",
      ask(*light, tq_lift=17.0, err=4.0, vel=0.0,
          no_gravity_model=True) == 0)

print("\n--- 2. the real event: extended, no obstruction ----------------")
ext = (136.7, 143.7, 0.0)
grav = true_torque("shoulder_lift", *ext)
check("the modelled arm reproduces the 19.28 N.m reading",
      abs(grav - 19.28) < 0.3, f"{grav:+.2f} N.m")
check("holding its own weight extended -> no abort",
      ask(*ext, tq_lift=grav, err=0.0, vel=0.0) == 0)
check("...even with the 2.43 deg lag seen on hardware",
      ask(*ext, tq_lift=grav, err=2.43, vel=0.5) == 0)

print("\n--- 3. an obstruction while extended still aborts --------------")
check("6 N.m over what the pose explains -> ABORT",
      ask(*ext, tq_lift=grav + 6.0, err=4.0, vel=0.0) == 1)
check("2 N.m over (inside the floor) -> no abort",
      ask(*ext, tq_lift=grav + 2.0, err=4.0, vel=0.0) == 0)

print("\n--- 4. moving joints are not stalls ----------------------------")
check("same torque+error but travelling -> no abort",
      ask(*light, tq_lift=17.0, err=4.0, vel=12.0) == 0)

print("\n--- 5. the hard ceiling is untouched ---------------------------")
check("31 N.m tracking perfectly -> ABORT anyway",
      ask(*ext, tq_lift=31.0, err=0.0, vel=5.0) == 1)

print("\n--- 6. refusals: bad data must not become a model --------------")
mb.GRAVITY_MODEL_PATH.unlink(missing_ok=True)
with mb.GRAVITY_SAMPLES_PATH.open("w") as fh:            # too few
    for i in range(40):
        fh.write(json.dumps({j: float(i) for j in mb.PITCH_CHAIN}
                            | {j + "_tq": 1.0 for j in mb.PITCH_CHAIN}) + "\n")
check("too few samples -> refuses", mb.fit_gravity(Namespace()) == 1
      and not mb.GRAVITY_MODEL_PATH.exists())

with mb.GRAVITY_SAMPLES_PATH.open("w") as fh:            # no spread
    for _ in range(400):
        lift = random.uniform(40, 45)
        rec = {"shoulder_lift": lift, "elbow_flex": 10.0, "wrist_flex": 0.0}
        for j in mb.PITCH_CHAIN:
            rec[j + "_tq"] = true_torque(j, lift, 10.0, 0.0)
        fh.write(json.dumps(rec) + "\n")
check("one corner of the workspace -> refuses",
      mb.fit_gravity(Namespace()) == 1
      and not mb.GRAVITY_MODEL_PATH.exists())

with mb.GRAVITY_SAMPLES_PATH.open("w") as fh:            # torque ~ pose-free
    for _ in range(400):
        rec = {"shoulder_lift": random.uniform(0, 140),
               "elbow_flex": random.uniform(0, 150),
               "wrist_flex": random.uniform(-60, 60)}
        for j in mb.PITCH_CHAIN:
            rec[j + "_tq"] = random.gauss(10, 3)
        fh.write(json.dumps(rec) + "\n")
check("pose explains nothing -> refuses", mb.fit_gravity(Namespace()) == 1
      and not mb.GRAVITY_MODEL_PATH.exists())

print("\n--- 7. sample logging only accepts settled frames --------------")
import io  # noqa: E402

settled = {j: {"pos_deg": 10.0, "vel_dps": 0.2, "torque": 5.0}
           for j in mb.PITCH_CHAIN}
tgt = {j: 10.0 for j in mb.PITCH_CHAIN}

buf = io.StringIO()
mb._log_gravity_sample(buf, settled, tgt)
check("settled + tracking frame is logged", buf.getvalue().count("\n") == 1)

buf = io.StringIO()
moving = {j: {"pos_deg": 10.0, "vel_dps": 25.0, "torque": 5.0}
          for j in mb.PITCH_CHAIN}
mb._log_gravity_sample(buf, moving, tgt)
check("moving frame is dropped (inertia, not gravity)", buf.getvalue() == "")

buf = io.StringIO()
mb._log_gravity_sample(buf, settled, tgt | {"shoulder_lift": 40.0})
check("not-tracking frame is dropped (maybe a stall)", buf.getvalue() == "")

buf = io.StringIO()
mb._log_gravity_sample(buf, {"shoulder_lift": settled["shoulder_lift"]}, {})
check("partial pose is dropped", buf.getvalue() == "")

mb._log_gravity_sample(None, settled, {})
check("no file handle is harmless", True)

print(f"\n{sum(results)}/{len(results)} passed")
sys.exit(0 if all(results) else 1)
