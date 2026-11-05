#!/usr/bin/env python
"""Exercise MuJoCoBackend: tracking, joint limits, step cap, speed cap, safety.

No hardware needed -- pure simulation. This is the acceptance evidence for
"loads in MuJoCo and responds to Python joint commands on a CPU-capable
machine" and "joint limits and configurable command-step/speed limits are
enforced".

    ./lerobot_alohamini/.venv/bin/python tools/test_backend.py
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ah.arm import SafetyLimits  # noqa: E402
from ah.backends import MuJoCoBackend  # noqa: E402
from ah.backends import RealB601Backend  # noqa: E402

FAILURES: list[str] = []


def check(label: str, ok: bool, detail: str = "") -> None:
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}" + (f" - {detail}" if detail else ""))
    if not ok:
        FAILURES.append(label)


print("=== 1. load and connect, CPU only ===")
arm = MuJoCoBackend(limits=SafetyLimits(max_step_deg=5.0, max_speed_dps=90.0))
arm.connect()
check("connected", arm.is_connected)
check("six arm joints", len(arm.joint_names) == 6, str(arm.joint_names))
check("gripper excluded by default", "gripper" not in arm.joint_names)

print("\n=== 2. model joint limits (degrees) ===")
lims = arm.model_joint_limits()
for name, (lo, hi) in lims.items():
    print(f"  {name:<15}{lo:8.1f} .. {hi:8.1f}")
check("shoulder_pan ~ +-160", abs(lims["shoulder_pan"][1] - 160.4) < 0.5)
check("wrist_flex ~ +-90", abs(lims["wrist_flex"][1] - 90.0) < 0.5)

def drive_to(a, name, value, loops=120, hz=30.0):
    """Command a target repeatedly at a realistic control rate until it lands.

    A tight `for` loop is NOT the right way to test this: at ~0.1 ms per
    iteration the speed cap (deg/SECOND) correctly allows almost no motion per
    command, so the arm appears stuck. At 30 Hz each command gets ~33 ms of
    budget, which is how teleop actually drives it.
    """
    period = 1.0 / hz
    for _ in range(loops):
        a.set_joint_positions({name: value})
        a.step()
        if abs(a._targets[name] - value) < 1e-3:
            break
        time.sleep(period)
    a.settle(1.5)
    return a.get_joint_positions()[name], a._targets[name]


print("\n=== 3. does the cascade actually track? ===")
pos, tgt = drive_to(arm, "shoulder_pan", 20.0)
check("target reached 20 deg", abs(tgt - 20.0) < 0.01, f"target={tgt:.3f}")
check("actual tracks target", abs(pos - tgt) < 1.0, f"actual={pos:.3f} err={pos - tgt:+.3f}")

print("\n=== 4. small joint changes are resolvable ===")
arm.settle(1.0)
base = arm.get_joint_positions()["wrist_yaw"]
for delta in (0.5, 1.0, 2.0):
    got, _ = drive_to(arm, "wrist_yaw", base + delta)
    err = got - base - delta
    check(f"{delta} deg command -> {got - base:+.3f} deg", abs(err) < 0.2, f"err={err:+.3f}")
    drive_to(arm, "wrist_yaw", base)

print("\n=== 5. joint limits are enforced ===")
before = arm.clamp_counts["joint_limit"]
applied = arm.set_joint_positions({"wrist_flex": 500.0})   # way past +90
check("absurd target clamped", applied["wrist_flex"] <= 90.0, f"applied={applied['wrist_flex']:.2f}")
check("clamp counted", arm.clamp_counts["joint_limit"] > before)

print("\n=== 6. step cap is enforced ===")
arm2 = MuJoCoBackend(limits=SafetyLimits(max_step_deg=2.0, max_speed_dps=10_000))
arm2.connect()
start = arm2._targets["shoulder_pan"]
applied = arm2.set_joint_positions({"shoulder_pan": start + 50.0})
moved = applied["shoulder_pan"] - start
check("50 deg jump capped to 2 deg", abs(moved - 2.0) < 1e-6, f"moved={moved:.4f}")
check("step cap counted", arm2.clamp_counts["step_cap"] == 1)

print("\n=== 7. speed cap is enforced ===")
arm3 = MuJoCoBackend(limits=SafetyLimits(max_step_deg=1000, max_speed_dps=10.0))
arm3.connect()
arm3.set_joint_positions({"shoulder_pan": 0.0})  # seeds dt
time.sleep(0.1)                                   # 0.1 s at 10 dps -> ~1 deg
applied = arm3.set_joint_positions({"shoulder_pan": 100.0})
check("burst limited by dps", applied["shoulder_pan"] < 3.0, f"applied={applied['shoulder_pan']:.3f}")
check("speed cap counted", arm3.clamp_counts["speed_cap"] == 1)

print("\n=== 8. NaN cannot reach the physics ===")
arm4 = MuJoCoBackend()
arm4.connect()
held = arm4._targets["elbow_flex"]
applied = arm4.set_joint_positions({"elbow_flex": float("nan")})
check("NaN rejected, target held", applied["elbow_flex"] == held)
check("nonfinite counted", arm4.clamp_counts["nonfinite"] == 1)
arm4.step()
check("sim state still finite", all(v == v for v in arm4.get_joint_positions().values()))

print("\n=== 9. hold / stop / partial commands ===")
drive_to(arm, "shoulder_pan", 12.0)
pan_target = arm._targets["shoulder_pan"]
# Command ONE joint. The others must hold their existing target, not snap to
# zero -- this is what makes per-joint direction testing possible.
arm.set_joint_positions({"wrist_yaw": arm.get_joint_positions()["wrist_yaw"] + 1.0})
check(
    "unspecified joints keep their target",
    abs(arm._targets["shoulder_pan"] - pan_target) < 1e-9,
    f"pan target {pan_target:.3f} -> {arm._targets['shoulder_pan']:.3f}",
)
held = arm.hold()
arm.settle(0.5)
check("hold freezes at measured pose", abs(held["shoulder_pan"] - arm.get_joint_positions()["shoulder_pan"]) < 1.0)
arm.stop()
check("stop leaves arm connected and held", arm.is_connected)

print("\n=== 10. unknown joint name rejected ===")
try:
    arm.set_joint_positions({"not_a_joint": 1.0})
    check("unknown name raises", False)
except KeyError:
    check("unknown name raises", True)

print("\n=== 11. real backend refuses honestly ===")
real = RealB601Backend(port="can0")
check("constructs", not real.is_connected)
try:
    real.connect()
    check("connect() raises NotImplementedError", False)
except NotImplementedError as exc:
    check("connect() raises NotImplementedError", True)
    print(f"        message starts: {str(exc).splitlines()[0][:60]}...")

print("\n=== 12. context manager ===")
with MuJoCoBackend() as cm:
    check("connected inside with-block", cm.is_connected)
check("disconnected after with-block", not cm.is_connected)

for a in (arm, arm2, arm3, arm4):
    a.disconnect()

print(f"\n{arm.safety_report()}")
print(f"\n{'ALL CHECKS PASSED' if not FAILURES else 'FAILURES: ' + ', '.join(FAILURES)}")
sys.exit(1 if FAILURES else 0)
