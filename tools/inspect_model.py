#!/usr/bin/env python
"""Dump the B601-RS MuJoCo model's joints, actuators and CPU step rate.

Verifies the model loads headless with no GPU and no ROS, and prints the
ground-truth joint limits that the leader mapping must be built against. The
MJCF limits are authoritative here -- lerobot's rebot_b601_follower config
carries a different set and disagrees with this model.

    ./lerobot_alohamini/.venv/bin/python tools/inspect_model.py
"""

import math
import sys
import time
from pathlib import Path

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # repo root

from ah import AH_TO_MJCF, MODEL_PATH  # noqa: E402

JOINT_TYPES = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    print(f"mujoco {mujoco.__version__}")
    print(f"model   {MODEL_PATH}")
    print(
        f"nq={model.nq} nu={model.nu} nbody={model.nbody} "
        f"timestep={model.opt.timestep}"
    )

    mjcf_to_ah = {v: k for k, v in AH_TO_MJCF.items()}

    print(
        f"\n{'#':>2}  {'mjcf joint':<13}{'AH name':<15}{'type':<7}"
        f"{'range':>21}{'degrees':>21}"
    )
    print("-" * 78)
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        jtype = JOINT_TYPES.get(int(model.jnt_type[i]), "?")
        lo, hi = model.jnt_range[i]
        deg = (
            f"{math.degrees(lo):8.1f} ..{math.degrees(hi):8.1f}"
            if jtype == "hinge"
            else f"{lo:.3f} .. {hi:.3f} m"
        )
        ah = mjcf_to_ah.get(name, "-")
        print(
            f"{i:>2}  {name:<13}{ah:<15}{jtype:<7}"
            f"{f'{lo:8.4f} ..{hi:8.4f}':>21}{deg:>21}"
        )

    print(f"\n{'actuator':<16}{'joint':<14}{'ctrlrange (N*m)':>18}")
    print("-" * 48)
    for i in range(model.nu):
        aname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        jname = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_JOINT, model.actuator_trnid[i, 0]
        )
        lo, hi = model.actuator_ctrlrange[i]
        print(f"{aname:<16}{jname:<14}{f'{lo:.0f} .. {hi:.0f}':>18}")

    # Headroom check: teleop needs the sim to run comfortably faster than
    # realtime on CPU, otherwise the loop cannot keep up with a 30 Hz leader.
    n = 5000
    t0 = time.perf_counter()
    for _ in range(n):
        mujoco.mj_step(model, data)
    elapsed = time.perf_counter() - t0
    sim_seconds = n * model.opt.timestep
    print(
        f"\n{n} steps in {elapsed:.3f}s -> {n / elapsed:.0f} steps/s, "
        f"{sim_seconds / elapsed:.1f}x realtime on CPU"
    )


if __name__ == "__main__":
    main()
