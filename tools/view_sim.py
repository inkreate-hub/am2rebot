#!/usr/bin/env python
"""Show the simulated B601-RS: interactive window, or rendered PNG frames.

Two modes, because they answer different questions:

  --window        Interactive MuJoCo viewer. Needs a display. Drag to orbit,
                  scroll to zoom. This is what you watch during teleop.
  --shot FILE     Render one frame offscreen to a PNG. Works headless over EGL,
                  so it also works in CI or over SSH, and it is how a result
                  gets attached to a report.

Useful combinations:

    # look at the home pose
    tools/view_sim.py --window

    # prove the home pose renders, headless
    tools/view_sim.py --shot reports/home.png

    # sweep every joint in turn so you can see which is which
    tools/view_sim.py --window --demo

    # contact sheet of the sweep, one PNG per joint
    tools/view_sim.py --demo --shot reports/sweep.png

Nothing here needs a GPU. On this machine the physics runs ~63x realtime on
CPU and rendering goes through EGL on the integrated Intel graphics.
"""

import argparse
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# Must be set before mujoco's GL is touched. egl = offscreen, no window.
if "--window" not in sys.argv:
    os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402

from ah import ARM_JOINTS  # noqa: E402
from ah.backends import MuJoCoBackend  # noqa: E402


def look_at_arm(cam: mujoco.MjvCamera, model: mujoco.MjModel) -> None:
    """Frame the whole arm.

    The model defines no camera, so derive the framing from MuJoCo's own model
    statistics rather than hard-coding a distance -- `stat.extent` is the
    model's characteristic size, so this stays correct if the mesh changes.
    """
    cam.lookat[:] = model.stat.center
    cam.distance = 1.4 * model.stat.extent
    cam.azimuth = 135.0
    cam.elevation = -20.0


def shoot(arm: MuJoCoBackend, path: Path, width: int, height: int) -> None:
    """Render the current state to a PNG."""
    from PIL import Image

    with mujoco.Renderer(arm.model, height=height, width=width) as renderer:
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(cam)
        look_at_arm(cam, arm.model)
        renderer.update_scene(arm.data, camera=cam)
        pixels = renderer.render()

    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(pixels).save(path)
    print(f"  wrote {path}  ({width}x{height})")


def demo_poses(arm: MuJoCoBackend) -> list[tuple[str, dict[str, float]]]:
    """A pose per joint, each a visible fraction of that joint's own travel.

    Scaled per joint rather than a fixed angle: 45 deg is a big move for
    wrist_flex and a small one for wrist_roll.
    """
    out: list[tuple[str, dict[str, float]]] = [("home", {})]
    home = arm.get_joint_positions()
    for name in ARM_JOINTS:
        lo, hi = arm.limits.joint_limits[name]
        # 35% toward whichever limit is further away, so the move is obvious
        # and never ends up pressed against an end stop.
        target = hi if (hi - home[name]) > (home[name] - lo) else lo
        value = home[name] + 0.35 * (target - home[name])
        out.append((name, {name: value}))
    return out


def drive_to(arm: MuJoCoBackend, pose: dict[str, float], seconds: float = 2.5,
             hz: float = 30.0) -> None:
    """Walk to a pose at the control rate, respecting the safety caps."""
    period = 1.0 / hz
    for _ in range(int(seconds * hz)):
        arm.set_joint_positions(pose)
        arm.step()
        time.sleep(period)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--window", action="store_true", help="interactive viewer")
    parser.add_argument("--shot", metavar="FILE", help="render a PNG instead/as well")
    parser.add_argument("--demo", action="store_true", help="sweep each joint in turn")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--seconds", type=float, default=30.0,
                        help="how long to hold the window open with no --demo")
    args = parser.parse_args()

    if not args.window and not args.shot:
        parser.error("pick --window and/or --shot FILE")

    arm = MuJoCoBackend(
        offscreen_size=(max(1280, args.width), max(960, args.height)),
        scenery=True,
    )
    arm.connect()
    arm.settle(1.0)   # let the controller take the home pose before showing it

    print(f"model      {arm.model_path}")
    print(f"joints     {', '.join(arm.joint_names)}")
    print(f"home pose  "
          + "  ".join(f"{k}={v:.1f}" for k, v in arm.get_joint_positions().items()))

    try:
        if args.window:
            import mujoco.viewer

            print("\nopening viewer - drag to orbit, scroll to zoom, Esc to quit")
            with mujoco.viewer.launch_passive(
                arm.model, arm.data, show_left_ui=False, show_right_ui=False
            ) as viewer:
                look_at_arm(viewer.cam, arm.model)
                if args.demo:
                    for label, pose in demo_poses(arm):
                        if not viewer.is_running():
                            break
                        print(f"  -> {label}")
                        period = 1.0 / 30.0
                        for _ in range(int(2.5 * 30)):
                            if not viewer.is_running():
                                break
                            arm.set_joint_positions(pose)
                            arm.step()
                            viewer.sync()
                            time.sleep(period)
                else:
                    deadline = time.perf_counter() + args.seconds
                    while viewer.is_running() and time.perf_counter() < deadline:
                        arm.step()
                        viewer.sync()
                        time.sleep(arm.substeps_per_command * arm.model.opt.timestep)

        if args.shot:
            base = Path(args.shot)
            if args.demo:
                for label, pose in demo_poses(arm):
                    drive_to(arm, pose)
                    arm.settle(0.5)
                    shoot(arm, base.with_name(f"{base.stem}-{label}{base.suffix}"),
                          args.width, args.height)
            else:
                shoot(arm, base, args.width, args.height)
    finally:
        arm.disconnect()
    print(f"\n{arm.safety_report()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
