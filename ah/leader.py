"""`AMLeaderReader`: calibrated joint positions from the physical AM-ARM200.

Wraps lerobot's `SOLeader` and adds the things a teleoperation loop needs and
the raw driver does not:

  - **Never raises mid-loop.** A failed read returns `None` rather than killing
    the session. A USB dropout is a thing to handle, not a crash -- the leader
    on this machine has re-enumerated mid-session before.
  - **Staleness.** `is_stale` goes true when the last good read is older than
    `stale_after_s`, so the follower can hold instead of chasing a frozen
    value. A leader that has stopped updating looks identical to a leader that
    is being held perfectly still; only the clock distinguishes them.
  - **Jitter statistics.** Read interval and sample-to-sample joint movement,
    so input jitter can be reported as a measured number.

Read-only: torque is never enabled, nothing is written to servo EEPROM. The arm
stays limp and is moved by hand.
"""

from __future__ import annotations

import logging
import statistics
import time
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)

#: Leader joint order, matching the am-leader-6dof profile (servo ids 1-7).
#: Identical names to the B601 follower, which is what makes joint-space
#: mapping possible without any kinematics.
LEADER_JOINTS: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
    "gripper",
)


@dataclass(frozen=True)
class LeaderSample:
    """One leader reading.

    Attributes:
        position: Joint angles in degrees (gripper normalised 0-100).
        t_mono: `time.monotonic()` at the read.
        t_unix: `time.time()` at the read.
        read_ms: How long the bus read took, milliseconds.
    """

    position: dict[str, float]
    t_mono: float
    t_unix: float
    read_ms: float


@dataclass
class ReaderStats:
    """Measured input quality. Reported rather than assumed."""

    reads_ok: int = 0
    reads_failed: int = 0
    intervals_ms: list[float] = field(default_factory=list)
    read_ms: list[float] = field(default_factory=list)
    #: Per-joint absolute change between consecutive samples, degrees.
    deltas: dict[str, list[float]] = field(default_factory=dict)

    def summary(self) -> str:
        if not self.intervals_ms:
            return "no samples"
        iv = self.intervals_ms
        rd = self.read_ms
        lines = [
            f"reads: {self.reads_ok} ok, {self.reads_failed} failed",
            f"interval: mean {statistics.fmean(iv):.1f} ms, "
            f"sd {statistics.pstdev(iv):.1f}, max {max(iv):.1f}  "
            f"-> {1000 / statistics.fmean(iv):.1f} Hz",
            f"bus read: mean {statistics.fmean(rd):.2f} ms, max {max(rd):.2f}",
        ]
        if self.deltas:
            lines.append("per-sample joint movement (input jitter), degrees:")
            for name, values in self.deltas.items():
                if values:
                    lines.append(
                        f"  {name:<15} mean {statistics.fmean(values):6.3f}  "
                        f"p95 {sorted(values)[int(0.95 * (len(values) - 1))]:6.3f}  "
                        f"max {max(values):6.3f}"
                    )
        return "\n".join(lines)


class AMLeaderReader:
    """Calibrated joint positions from the physical AM-ARM200 leader.

    Args:
        port: Serial device, e.g. `/dev/ttyACM0`. Check with `ls /dev/ttyACM*`;
            the number depends on plug order.
        leader_id: Calibration id. Must match a stored calibration, otherwise
            the zero is meaningless.
        arm_profile: lerobot arm profile. `am-leader-6dof` is the AM-ARM200.
        stale_after_s: How old the last good read may be before `is_stale`.
            0.25 s is ~8 missed frames at 30 Hz: long enough to ride out a
            hiccup, short enough that a dead leader is caught quickly.
        track_jitter: Collect per-sample statistics. Small cost, and it is the
            evidence for "log input jitter".

    Example:
        >>> reader = AMLeaderReader(port="/dev/ttyACM0", leader_id="am_leader")
        >>> reader.connect()                           # doctest: +SKIP
        >>> sample = reader.read()                     # doctest: +SKIP
        >>> sample.position["shoulder_pan"]            # doctest: +SKIP
        12.4
        >>> reader.disconnect()                        # doctest: +SKIP
    """

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        leader_id: str = "am_leader",
        arm_profile: str = "am-leader-6dof",
        stale_after_s: float = 0.25,
        track_jitter: bool = True,
    ) -> None:
        self.port = port
        self.leader_id = leader_id
        self.arm_profile = arm_profile
        self.stale_after_s = stale_after_s
        self.track_jitter = track_jitter

        self._leader = None
        self._connected = False
        self._last_good: LeaderSample | None = None
        self._consecutive_failures = 0
        self.stats = ReaderStats()

    # ---------------------------------------------------------------- lifecycle

    def connect(self) -> None:
        """Open the serial bus and load the stored calibration.

        Does NOT re-run the calibration sweep: a fresh sweep produces different
        (though equally valid) offsets, which would invalidate every mapping
        tuned against the current one.

        Raises:
            ConnectionError: If the port cannot be opened or servos do not
                answer. Worth catching at startup -- unlike mid-loop read
                failures, this one should stop the session.
        """
        from lerobot.teleoperators.so_leader import SOLeader
        from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig

        self._leader = SOLeader(
            SOLeaderTeleopConfig(
                port=self.port, id=self.leader_id, arm_profile=self.arm_profile
            )
        )
        self._leader.connect(calibrate=False)
        if not self._leader.is_calibrated:
            logger.warning(
                "leader reports uncalibrated -- joint zeros are meaningless and "
                "the mapping will be offset. See RUNBOOK.md section 2."
            )
        self._connected = True

    def disconnect(self) -> None:
        """Release the bus. Safe to call twice, and safe after a failure."""
        if self._leader is not None and self._connected:
            try:
                self._leader.disconnect()
            except Exception as exc:  # noqa: BLE001 - teardown must not mask
                logger.warning("leader disconnect failed: %s", exc)
        self._connected = False

    def __enter__(self) -> AMLeaderReader:
        self.connect()
        return self

    def __exit__(self, *exc: object) -> None:
        self.disconnect()

    # ------------------------------------------------------------------- state

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def last_good(self) -> LeaderSample | None:
        """Most recent successful sample, or None before the first one."""
        return self._last_good

    @property
    def age_s(self) -> float:
        """Seconds since the last good read. `inf` if there has not been one."""
        if self._last_good is None:
            return float("inf")
        return time.monotonic() - self._last_good.t_mono

    @property
    def is_stale(self) -> bool:
        """True when input is too old to act on.

        The caller should hold the follower rather than keep applying the last
        value: a frozen reading and a perfectly steady hand are indistinguish-
        able from the data alone.
        """
        return self.age_s > self.stale_after_s

    @property
    def consecutive_failures(self) -> int:
        """Failed reads since the last good one. Non-zero means trouble."""
        return self._consecutive_failures

    # -------------------------------------------------------------------- read

    def read(self) -> LeaderSample | None:
        """Read all seven joints.

        Returns:
            A `LeaderSample`, or `None` if the read failed. Failure is normal
            enough to handle inline -- USB re-enumeration, a contended port, a
            servo flagging a voltage fault. The caller should treat `None` as
            "no new information", not as an error to abort on.
        """
        if not self._connected or self._leader is None:
            return None

        t0 = time.perf_counter()
        try:
            action = self._leader.get_action()
        except Exception as exc:  # noqa: BLE001 - any bus error is survivable
            self._consecutive_failures += 1
            self.stats.reads_failed += 1
            if self._consecutive_failures in (1, 10, 100):
                logger.warning(
                    "leader read failed (%d in a row): %s",
                    self._consecutive_failures,
                    exc,
                )
            return None

        read_ms = (time.perf_counter() - t0) * 1e3
        position = {
            name: float(action[f"{name}.pos"])
            for name in LEADER_JOINTS
            if f"{name}.pos" in action
        }
        sample = LeaderSample(
            position=position,
            t_mono=time.monotonic(),
            t_unix=time.time(),
            read_ms=read_ms,
        )

        if self.track_jitter:
            self.stats.reads_ok += 1
            self.stats.read_ms.append(read_ms)
            if self._last_good is not None:
                self.stats.intervals_ms.append(
                    (sample.t_mono - self._last_good.t_mono) * 1e3
                )
                for name, value in position.items():
                    previous = self._last_good.position.get(name)
                    if previous is not None:
                        self.stats.deltas.setdefault(name, []).append(
                            abs(value - previous)
                        )

        self._last_good = sample
        self._consecutive_failures = 0
        return sample
