#!/usr/bin/env python
"""Survey self-collisions across the workspace.

The contact-exclude list in `MuJoCoBackend` was copied from Seeed's
`rs_grasp_scene.xml`. That scene has its own working envelope, so it is not
obvious the list covers every pair that touches in poses teleoperation can
reach. An unexcluded self-collision does not look like a collision from the
outside -- the constraint force cancels the actuator torque and the joint
simply stops responding, which is exactly how this model behaved before any
excludes were added.

Samples poses across each joint's full range and reports which body pairs
touch, how often, and how deep.

    ./lerobot_alohamini/.venv/bin/python tools/check_collisions.py
    ./lerobot_alohamini/.venv/bin/python tools/check_collisions.py --samples 4000
"""

import argparse
import os
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from ah import ALL_JOINTS  # noqa: E402
from ah.backends import MuJoCoBackend  # noqa: E402
from ah.backends.mujoco_backend import SELF_COLLISION_EXCLUDES  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=3000)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    arm = MuJoCoBackend(include_gripper=True)
    arm.connect()
    model, data = arm.model, arm.data
    rng = np.random.default_rng(args.seed)

    limits = {n: model.jnt_range[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, _mjcf(n))
    ] for n in ALL_JOINTS}

    pairs: Counter[tuple[str, str]] = Counter()
    worst: dict[tuple[str, str], float] = {}
    poses_with_contact = 0

    for _ in range(args.samples):
        for name in ALL_JOINTS:
            lo, hi = limits[name]
            data.qpos[arm._qpos_adr[name]] = rng.uniform(lo, hi)
        mujoco.mj_forward(model, data)
        if data.ncon:
            poses_with_contact += 1
        for i in range(data.ncon):
            contact = data.contact[i]
            b1 = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[contact.geom1]
            )
            b2 = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[contact.geom2]
            )
            key = tuple(sorted((b1, b2)))
            pairs[key] += 1
            worst[key] = min(worst.get(key, 0.0), float(contact.dist))

    excluded = {tuple(sorted(p)) for p in SELF_COLLISION_EXCLUDES}

    print(f"sampled {args.samples} random poses across every joint's full range")
    print(f"{poses_with_contact} ({100 * poses_with_contact / args.samples:.1f}%) "
          f"had at least one contact\n")

    if not pairs:
        print("no self-collisions anywhere in the sampled workspace")
        arm.disconnect()
        return 0

    print(f"{'body pair':<44}{'poses':>8}{'worst dist':>12}   excluded?")
    print("-" * 78)
    missing = []
    for key, count in pairs.most_common():
        is_excluded = key in excluded
        if not is_excluded:
            missing.append(key)
        print(
            f"{key[0] + ' <-> ' + key[1]:<44}{count:>8}{worst[key]:>12.5f}"
            f"   {'yes' if is_excluded else 'NO  <-- locks joints'}"
        )

    if missing:
        print("\nAdd to SELF_COLLISION_EXCLUDES in ah/backends/mujoco_backend.py:")
        for b1, b2 in missing:
            print(f'    ("{b1}", "{b2}"),')
    else:
        print("\nevery colliding pair is already excluded")

    arm.disconnect()
    return 0


def _mjcf(name: str) -> str:
    from ah import AH_TO_MJCF

    return AH_TO_MJCF[name]


if __name__ == "__main__":
    raise SystemExit(main())
