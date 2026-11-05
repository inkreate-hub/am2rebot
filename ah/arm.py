"""`AHArm`: the backend-agnostic follower interface.

Every caller -- teleoperation, replay, a future visual-servoing pipeline, a
LeRobot dataset recorder -- talks to this and nothing below it. Backends
translate AH commands into MuJoCo control or real CAN motor frames.

The contract, in B601 semantic joint names and **degrees**:

    arm = MuJoCoBackend()            # or RealB601Backend(...)
    arm.connect()
    q = arm.get_joint_positions()    # {"shoulder_pan": 12.4, ...}
    arm.set_joint_positions({"shoulder_pan": 20.0})
    arm.stop()
    arm.disconnect()

Why degrees and semantic names rather than radians and joint1..joint6: it is
what lerobot's `rebot_b601_follower` already speaks, so the real backend is a
drop-in. MuJoCo's radians, its joint1..joint7 naming and its *linear* gripper
slide are internal to `MuJoCoBackend`.

SAFETY IS ENFORCED HERE, NOT IN THE BACKENDS. `set_joint_positions` is
deliberately non-abstract: it clamps to joint limits, applies the per-command
step cap and the per-second speed cap, then hands the already-safe targets to
`_write_targets`. A backend cannot opt out, and a caller cannot bypass it.
"""

from __future__ import annotations

import abc
import time
from collections.abc import Mapping
from dataclasses import dataclass, field

from . import ARM_JOINTS


@dataclass(frozen=True)
class JointState:
    """One sampled follower state.

    Attributes:
        position: Measured joint positions in degrees, keyed by AH joint name.
            For the simulator this is the physics state; for the real arm it is
            motor feedback.
        target: The last commanded targets in degrees, after safety clamping.
            Comparing this against `position` is the follower tracking error --
            with torque actuators and a finite-gain controller the two are
            genuinely different, which is the point.
        t_mono: `time.monotonic()` when the sample was taken. Use for intervals
            and rates; it cannot jump backwards on an NTP step.
        t_unix: `time.time()` when the sample was taken. Use to correlate with
            a video file or another process.
    """

    position: dict[str, float]
    target: dict[str, float]
    t_mono: float
    t_unix: float


@dataclass
class SafetyLimits:
    """Caps applied to every command, regardless of backend or caller.

    Attributes:
        joint_limits: Per-joint `(min, max)` in degrees. Targets are clamped
            into this window. Defaults are filled from the backend's model on
            connect, so they match the real mechanism rather than a guess.
        max_step_deg: Largest change any single joint may make in one
            `set_joint_positions` call. Catches a bad mapping or a garbled
            leader read before it becomes a fast swing.
        max_speed_dps: Largest rate any joint may move, degrees per second,
            measured against wall-clock time between calls. Independent of
            `max_step_deg`: the step cap bounds one command, the speed cap
            bounds a burst of them.
        reject_nonfinite: Drop NaN/inf targets instead of passing them to the
            physics or the motors. A NaN reaching MuJoCo corrupts the whole
            simulation state irrecoverably.
    """

    joint_limits: dict[str, tuple[float, float]] = field(default_factory=dict)
    max_step_deg: float = 5.0
    max_speed_dps: float = 90.0
    reject_nonfinite: bool = True


class AHArm(abc.ABC):
    """Abstract follower arm.

    Subclasses implement four things: `_open`, `_close`, `_read_positions` and
    `_write_targets`. Everything public is provided here so that safety,
    bookkeeping and the command contract behave identically across backends.
    """

    #: Joints this interface commands. The gripper is a documented follow-on
    #: and is not part of the six-joint acceptance scope, so backends that
    #: support it extend this in their own `__init__`.
    joint_names: tuple[str, ...] = ARM_JOINTS

    def __init__(self, limits: SafetyLimits | None = None) -> None:
        self.limits = limits or SafetyLimits()
        self._connected = False
        self._targets: dict[str, float] = {}
        self._last_command_t: float | None = None
        #: Counts of safety interventions, for the run report. Honest telemetry
        #: beats a claim that limits "are enforced".
        self.clamp_counts: dict[str, int] = {
            "joint_limit": 0,
            "step_cap": 0,
            "speed_cap": 0,
            "nonfinite": 0,
        }

    # ---------------------------------------------------------------- lifecycle

    @property
    def is_connected(self) -> bool:
        """True between a successful `connect()` and `disconnect()`."""
        return self._connected

    def connect(self) -> None:
        """Bring the backend up and seed targets from the current pose.

        Seeding matters: if targets started at zero, the first
        `set_joint_positions` would be rate-limited away from a pose the arm is
        not actually in, and the step cap would make the arm crawl toward the
        leader from the wrong place. Starting from where the arm *is* means the
        first command is relative to reality.
        """
        if self._connected:
            return
        self._open()
        self._connected = True
        self._targets = dict(self._read_positions())
        self._last_command_t = None
        # Fill any unspecified joint limits from the backend's own model.
        model_limits = self.model_joint_limits()
        for name in self.joint_names:
            self.limits.joint_limits.setdefault(name, model_limits[name])

    def disconnect(self) -> None:
        """Stop the arm, then release the backend. Safe to call twice."""
        if not self._connected:
            return
        try:
            self.stop()
        finally:
            self._close()
            self._connected = False

    def __enter__(self) -> AHArm:
        self.connect()
        return self

    def __exit__(self, *exc: object) -> None:
        self.disconnect()

    # ----------------------------------------------------------------- commands

    def get_joint_positions(self) -> dict[str, float]:
        """Measured joint positions in degrees, keyed by AH joint name."""
        self._require_connected()
        return dict(self._read_positions())

    def get_state(self) -> JointState:
        """Measured positions, last commanded targets and both timestamps."""
        self._require_connected()
        return JointState(
            position=dict(self._read_positions()),
            target=dict(self._targets),
            t_mono=time.monotonic(),
            t_unix=time.time(),
        )

    def set_joint_positions(self, q: Mapping[str, float]) -> dict[str, float]:
        """Command joint targets in degrees. Returns what was actually applied.

        Partial commands are fine -- joints absent from `q` hold their previous
        target rather than snapping to zero, which is what makes single-joint
        testing possible.

        Every target passes through, in order: non-finite rejection, joint-limit
        clamping, the per-command step cap, then the per-second speed cap. The
        return value is the post-clamp target, so a caller can see exactly what
        the arm was asked to do and log target-versus-actual honestly.

        Args:
            q: Mapping of AH joint name to target in degrees. Unknown names
                raise `KeyError`.

        Returns:
            The applied targets in degrees, for every joint this arm commands.

        Raises:
            RuntimeError: If called before `connect()`.
            KeyError: If `q` contains a name this arm does not command.
        """
        self._require_connected()

        unknown = set(q) - set(self.joint_names)
        if unknown:
            raise KeyError(
                f"not joints of this arm: {sorted(unknown)}. "
                f"Known: {list(self.joint_names)}"
            )

        now = time.monotonic()
        # First command after connect has no meaningful dt, so the speed cap
        # cannot be evaluated; the step cap still applies.
        dt = None if self._last_command_t is None else max(1e-6, now - self._last_command_t)

        applied = dict(self._targets)
        for name, raw in q.items():
            value = float(raw)

            if self.limits.reject_nonfinite and not _is_finite(value):
                self.clamp_counts["nonfinite"] += 1
                continue  # hold the previous target

            previous = applied[name]

            lo, hi = self.limits.joint_limits[name]
            if value < lo or value > hi:
                self.clamp_counts["joint_limit"] += 1
                value = min(hi, max(lo, value))

            step = value - previous
            if abs(step) > self.limits.max_step_deg:
                self.clamp_counts["step_cap"] += 1
                value = previous + _sign(step) * self.limits.max_step_deg
                step = value - previous

            if dt is not None:
                max_by_speed = self.limits.max_speed_dps * dt
                if abs(step) > max_by_speed:
                    self.clamp_counts["speed_cap"] += 1
                    value = previous + _sign(step) * max_by_speed

            applied[name] = value

        self._targets = applied
        self._last_command_t = now
        self._write_targets(applied)
        return dict(applied)

    def hold(self) -> dict[str, float]:
        """Freeze targets at the arm's present measured pose.

        For pause: the controller keeps holding, so the arm resists gravity
        instead of going slack, but it stops chasing the leader. Returns the
        held targets.
        """
        self._require_connected()
        self._targets = dict(self._read_positions())
        self._last_command_t = None  # resume must not be speed-capped vs a stale dt
        self._write_targets(self._targets)
        return dict(self._targets)

    def stop(self) -> None:
        """Stop commanded motion immediately, leaving the arm held in place.

        Default implementation holds the current pose. A backend whose hardware
        has a distinct torque-off or brake state should override this and say in
        its docstring which it does -- "stop" meaning "go limp" versus "hold
        position" is a safety-relevant difference on a real arm.
        """
        if not self._connected:
            return
        self.hold()

    # ------------------------------------------------------------------ backend

    @abc.abstractmethod
    def _open(self) -> None:
        """Acquire the backend: load the model, or open the CAN bus."""

    @abc.abstractmethod
    def _close(self) -> None:
        """Release the backend. Must not raise if already released."""

    @abc.abstractmethod
    def _read_positions(self) -> Mapping[str, float]:
        """Measured positions in degrees, keyed by AH joint name."""

    @abc.abstractmethod
    def _write_targets(self, targets: Mapping[str, float]) -> None:
        """Apply already-clamped targets. Do not re-clamp or re-validate."""

    @abc.abstractmethod
    def model_joint_limits(self) -> dict[str, tuple[float, float]]:
        """Mechanism joint limits in degrees, from the model or the datasheet.

        Used to populate `SafetyLimits.joint_limits` on connect so the defaults
        match the real mechanism instead of a hand-copied guess.
        """

    # -------------------------------------------------------------------- utils

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError(
                f"{type(self).__name__} is not connected -- call connect() first"
            )

    def safety_report(self) -> str:
        """One-line summary of how often each cap intervened."""
        parts = [f"{k}={v}" for k, v in self.clamp_counts.items() if v]
        return "safety: " + (", ".join(parts) if parts else "no interventions")


def _is_finite(x: float) -> bool:
    return x == x and x not in (float("inf"), float("-inf"))


def _sign(x: float) -> float:
    return 1.0 if x >= 0 else -1.0
