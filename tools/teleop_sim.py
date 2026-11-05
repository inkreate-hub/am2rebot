#!/usr/bin/env python
"""Drive the simulated B601-RS live from the physical AM-ARM200 leader.

Same viewer as `view_sim.py --demo`, except the poses come from your hand
instead of a script.

    tools/teleop_sim.py --window

KEYS (in the viewer window)

    SPACE   pause / resume. Paused holds the follower where it is.
    E       re-engage: ramp back onto the leader's current pose.
    R       start / stop recording to a CSV.
    Q       quit.

SAFETY, in the order it applies

    1. staleness    If the leader read fails or goes older than --stale-after,
                    the follower HOLDS. It does not keep re-applying the last
                    value, because a frozen reading and a perfectly steady
                    hand look identical in the data.
    2. soft engage  On start and on every re-engage the target ramps from the
                    follower's CURRENT pose to the mapped leader pose over
                    --engage-seconds. Without this, starting with the leader
                    somewhere unrelated snaps the follower across its workspace.
    3. AHArm caps   Joint limits, per-command step cap and per-second speed cap,
                    enforced inside AHArm where no caller can bypass them.

DIRECTION IS NOT VERIFIED BY THIS SCRIPT. It applies whatever signs the mapping
carries. Use `--flip joint1,joint2` to invert any that move the wrong way, then
record them in the mapping. Neither the leader's nor the model's signs are
derivable on paper -- the leader's were measured and six of its seven axes read
inverted, and the MJCF's own joint axes are mixed.
"""

import argparse
import csv
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

if "--window" not in sys.argv:
    os.environ.setdefault("MUJOCO_GL", "egl")

from ah import ARM_JOINTS  # noqa: E402
from ah.arm import SafetyLimits  # noqa: E402
from ah.backends import MuJoCoBackend  # noqa: E402
from ah.leader import AMLeaderReader  # noqa: E402
from ah.mapping import JointMapper  # noqa: E402


class Session:
    """Teleop state machine: engaging -> running, plus paused and held."""

    def __init__(self, arm, reader, mapper, engage_seconds: float) -> None:
        self.arm = arm
        self.reader = reader
        self.mapper = mapper
        self.engage_seconds = engage_seconds

        self.paused = False
        self.quit = False
        self.recording: csv.writer | None = None
        self._record_file = None

        self._engage_t0: float | None = None
        self._engage_from: dict[str, float] | None = None
        self.engage()

        self.rows = 0
        self.held_stale = 0
        self.loop_ms: list[float] = []
        self.track_err: dict[str, list[float]] = {n: [] for n in ARM_JOINTS}

    # ------------------------------------------------------------------ engage

    def engage(self) -> None:
        """Begin ramping from wherever the follower is onto the leader pose."""
        self._engage_t0 = time.monotonic()
        self._engage_from = (
            self.arm.get_joint_positions() if self.arm.is_connected else None
        )

    @property
    def engaging(self) -> bool:
        return self._engage_t0 is not None

    def _blend(self, mapped: dict[str, float]) -> dict[str, float]:
        """Cross-fade from the pose at engage time to the live mapped pose."""
        if self._engage_t0 is None or self._engage_from is None:
            return mapped
        alpha = (time.monotonic() - self._engage_t0) / max(1e-6, self.engage_seconds)
        if alpha >= 1.0:
            self._engage_t0 = None
            self._engage_from = None
            return mapped
        return {
            name: (1.0 - alpha) * self._engage_from.get(name, value) + alpha * value
            for name, value in mapped.items()
        }

    # ----------------------------------------------------------------- control

    def tick(self) -> str:
        """One control iteration. Returns a short status string."""
        t0 = time.perf_counter()
        sample = self.reader.read()

        if self.paused:
            self.arm.hold()
            self.arm.step()
            status = "PAUSED"
        elif sample is None or self.reader.is_stale:
            # Hold rather than re-apply a stale value.
            self.arm.hold()
            self.arm.step()
            self.held_stale += 1
            status = f"STALE {self.reader.age_s * 1000:.0f} ms - holding"
        else:
            targets = self._blend(self.mapper(sample.position))
            self.arm.set_joint_positions(targets)
            self.arm.step()
            status = "ENGAGING" if self.engaging else "running"

        self.loop_ms.append((time.perf_counter() - t0) * 1e3)

        state = self.arm.get_state()
        for name in ARM_JOINTS:
            self.track_err[name].append(state.position[name] - state.target[name])

        if self.recording is not None and sample is not None:
            self.recording.writerow(
                [f"{state.t_mono:.6f}", f"{state.t_unix:.6f}", status.split()[0]]
                + [f"{sample.position.get(n, float('nan')):.4f}" for n in ARM_JOINTS]
                + [f"{state.target[n]:.4f}" for n in ARM_JOINTS]
                + [f"{state.position[n]:.4f}" for n in ARM_JOINTS]
                + [f"{self.loop_ms[-1]:.3f}"]
            )
            self.rows += 1

        return status

    # --------------------------------------------------------------- recording

    def toggle_recording(self, path: Path) -> str:
        if self.recording is not None:
            self._record_file.close()
            self.recording = None
            self._record_file = None
            return f"recording stopped ({self.rows} rows)"
        path.parent.mkdir(parents=True, exist_ok=True)
        self._record_file = open(path, "w", newline="")
        self.recording = csv.writer(self._record_file)
        self.recording.writerow(
            ["t_mono", "t_unix", "state"]
            + [f"leader_{n}" for n in ARM_JOINTS]
            + [f"target_{n}" for n in ARM_JOINTS]
            + [f"actual_{n}" for n in ARM_JOINTS]
            + ["loop_ms"]
        )
        self.rows = 0
        return f"recording to {path}"

    def close(self) -> None:
        if self._record_file is not None:
            self._record_file.close()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--window", action="store_true", help="show the viewer")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--leader-id", default="am_leader")
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--seconds", type=float, default=0.0, help="0 = until Q/Ctrl-C")
    parser.add_argument("--engage-seconds", type=float, default=1.5)
    parser.add_argument("--stale-after", type=float, default=0.25)
    parser.add_argument("--max-step-deg", type=float, default=5.0)
    parser.add_argument("--max-speed-dps", type=float, default=120.0)
    parser.add_argument("--flip", default="", help="comma-separated joints to invert")
    parser.add_argument("--record", metavar="FILE", help="start recording immediately")
    args = parser.parse_args()

    flips = {n.strip() for n in args.flip.split(",") if n.strip()}
    unknown = flips - set(ARM_JOINTS)
    if unknown:
        parser.error(f"--flip: unknown joints {sorted(unknown)}")

    arm = MuJoCoBackend(
        limits=SafetyLimits(
            max_step_deg=args.max_step_deg, max_speed_dps=args.max_speed_dps
        ),
        scenery=True,
    )
    arm.connect()
    arm.settle(0.5)

    mapper = JointMapper.from_ranges(
        arm.model_joint_limits(),
        directions={n: (-1 if n in flips else 1) for n in ARM_JOINTS},
    )
    print("\nMAPPING\n" + mapper.describe())

    reader = AMLeaderReader(
        port=args.port, leader_id=args.leader_id, stale_after_s=args.stale_after
    )
    try:
        reader.connect()
    except Exception as exc:  # noqa: BLE001
        print(f"\ncannot open the leader on {args.port}: {exc}", file=sys.stderr)
        print("check: ls /dev/ttyACM*   and   fuser -v /dev/ttyACM0", file=sys.stderr)
        arm.disconnect()
        return 1

    session = Session(arm, reader, mapper, args.engage_seconds)
    record_path = Path(args.record) if args.record else Path("reports/teleop.csv")
    if args.record:
        print(session.toggle_recording(record_path))

    def on_key(keycode: int) -> None:
        key = chr(keycode) if 32 <= keycode < 127 else ""
        if key == " ":
            session.paused = not session.paused
            print(f"\n[{'PAUSED' if session.paused else 'RESUMED'}]")
            if not session.paused:
                session.engage()        # resume must not snap either
        elif key in ("E", "e"):
            session.engage()
            print("\n[re-engaging]")
        elif key in ("R", "r"):
            print("\n[" + session.toggle_recording(record_path) + "]")
        elif key in ("Q", "q"):
            session.quit = True

    print(f"\nleader {args.port} -> simulated B601-RS at {args.hz:g} Hz")
    print("SPACE pause/resume   E re-engage   R record   Q quit\n")

    period = 1.0 / args.hz
    deadline = time.perf_counter() + args.seconds if args.seconds > 0 else float("inf")
    viewer = None
    try:
        if args.window:
            import mujoco.viewer

            from view_sim import look_at_arm

            viewer = mujoco.viewer.launch_passive(
                arm.model, arm.data, show_left_ui=False, show_right_ui=False,
                key_callback=on_key,
            )
            look_at_arm(viewer.cam, arm.model)

        n = 0
        while not session.quit and time.perf_counter() < deadline:
            if viewer is not None and not viewer.is_running():
                break
            status = session.tick()
            if viewer is not None:
                viewer.sync()
            n += 1
            if n % 10 == 0 and sys.stdout.isatty():
                lead = reader.last_good
                cells = (
                    "  ".join(
                        f"{k[:9]}:{v:7.1f}" for k, v in list(lead.position.items())[:3]
                    )
                    if lead
                    else "no leader data"
                )
                print(f"\r{status:<22} {cells}", end="", flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        if viewer is not None:
            viewer.close()
        session.close()
        reader.disconnect()
        arm.disconnect()

    print("\n\n=== leader input ===")
    print(reader.stats.summary())

    if session.loop_ms:
        import statistics

        print("\n=== loop timing ===")
        print(
            f"iterations {len(session.loop_ms)}, mean {statistics.fmean(session.loop_ms):.2f} ms, "
            f"p95 {sorted(session.loop_ms)[int(0.95 * (len(session.loop_ms) - 1))]:.2f}, "
            f"max {max(session.loop_ms):.2f}"
        )
        print(f"held on stale/failed input: {session.held_stale} iterations")

    print("\n=== follower tracking error (actual - target), degrees ===")
    import statistics

    for name, errors in session.track_err.items():
        if errors:
            print(
                f"  {name:<15} mean {statistics.fmean(errors):+7.3f}  "
                f"max |err| {max(abs(e) for e in errors):6.3f}"
            )
    print(f"\n{arm.safety_report()}")
    if session.rows:
        print(f"recorded {session.rows} rows to {record_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
