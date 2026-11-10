"""`JointMapper`: AM-ARM200 leader joint angles -> B601 follower joint targets.

Joint-space mapping only. No FK, no IK, no Cartesian retargeting -- the same
pattern Seeed use for StarArm102 -> B601, which is leader/follower joint-name
alignment plus per-joint direction, range and offset.

WHAT HAS TO BE RECONCILED

1. **Different travel.** The leader and the follower do not share joint ranges.
   Measured leader travel (find_leader_ranges.py, id=am_leader) against the
   MuJoCo model's limits:

       joint          leader span   follower span
       shoulder_pan        237.8          320.8
       shoulder_lift       206.2          191.4
       elbow_flex          185.1          191.4
       wrist_flex          185.0          180.0
       wrist_yaw           158.5          180.0
       wrist_roll          342.3          359.8

2. **Different zero conventions.** The leader zeroes at mid-travel, set by its
   calibration sweep. The follower's shoulder_lift and elbow_flex run
   -11.5 .. 179.9 deg, i.e. almost entirely positive. So those two need an
   offset of roughly +84 deg, not a stretch.

3. **Different signs.** Neither side is derivable from a datasheet. The leader's
   were measured on hardware (six of its seven axes read inverted relative to
   intuition). The model's joint axes are mixed too -- `joint1` is `0 0 -1`
   while `joint2` is `0 0 1`. So direction is configurable and must be
   confirmed live; see `tools/verify_mapping_live.py`.

THE FITTING RULE

`fit_map()` derives each mapping instead of hand-typing numbers:

  - if the leader's span fits inside the follower's, use **1:1 degrees** and
    centre-align. Preserves the natural feel -- the follower mirrors the
    operator's hand rather than amplifying it.
  - otherwise **scale to fit**, so the operator can still reach both ends.

Centre-aligning rather than min-aligning means a mapping error shows up as a
symmetric offset rather than driving the follower into an end stop.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field

from . import ARM_JOINTS

#: Leader travel measured on this arm, degrees. From `find_leader_ranges.py`,
#: calibration id `am_leader`, 2026-11-05. The gripper is normalised 0-100.
LEADER_RANGES: dict[str, tuple[float, float]] = {
    "shoulder_pan": (-118.1, 119.7),
    "shoulder_lift": (-102.6, 103.6),
    "elbow_flex": (-92.3, 92.9),
    "wrist_flex": (-92.4, 92.5),
    "wrist_yaw": (-78.5, 80.0),
    "wrist_roll": (-170.1, 172.3),
    "gripper": (0.1, 99.0),
}


#: Verified leader -> follower directions, observed on hardware 2026-11-05.
#:
#: -1 means the follower joint must run opposite to the leader's reported
#: angle. Established by driving the sim from the live leader and watching:
#: elbow_flex, wrist_flex and wrist_yaw moved the wrong way, the other three
#: were correct.
#:
#: NOT a calibration property, and re-calibrating will not change it. A
#: calibration sweep sets each joint's zero and range; the sign comes from how
#: the servo horn was physically seated on the leader and which way the MJCF's
#: joint axis points (`rs_arm.xml` is itself mixed -- joint1 is `0 0 -1` while
#: joint2 is `0 0 1`). Re-running calibration would produce the same signs with
#: different offsets, discarding the measured ranges below for nothing.
DEFAULT_DIRECTIONS: dict[str, int] = {
    "shoulder_pan": 1,
    "shoulder_lift": 1,
    "elbow_flex": -1,
    "wrist_flex": -1,
    "wrist_yaw": -1,
    "wrist_roll": 1,
    "gripper": 1,
}


@dataclass
class JointMap:
    """One joint's leader -> follower conversion.

        follower_deg = clamp(direction * scale * leader_deg + offset)

    Attributes:
        direction: +1 or -1. Whether increasing leader angle increases or
            decreases the follower joint. NOT derivable from either model --
            confirm on hardware.
        scale: Gain. 1.0 means true 1:1 degrees.
        offset: Added after scaling, degrees. Reconciles the two zero
            conventions.
        leader_range: Measured leader travel, for provenance and for `describe`.
        follower_range: The follower's own limit, from its model.
        note: Why this mapping is what it is.
    """

    direction: int = 1
    scale: float = 1.0
    offset: float = 0.0
    leader_range: tuple[float, float] = (0.0, 0.0)
    follower_range: tuple[float, float] = (0.0, 0.0)
    note: str = ""

    def apply(self, leader_deg: float) -> float:
        """Convert one leader angle to a follower target, clamped to its limit."""
        value = self.direction * self.scale * leader_deg + self.offset
        lo, hi = self.follower_range
        return min(hi, max(lo, value))

    def image(self) -> tuple[float, float]:
        """Where the leader's full travel lands on the follower, pre-clamping."""
        a = self.direction * self.scale * self.leader_range[0] + self.offset
        b = self.direction * self.scale * self.leader_range[1] + self.offset
        return (min(a, b), max(a, b))

    def clipped_fraction(self) -> float:
        """Fraction of the mapped span that falls outside the follower's limit."""
        lo_img, hi_img = self.image()
        lo, hi = self.follower_range
        span = hi_img - lo_img
        if span <= 0:
            return 0.0
        lost = max(0.0, lo - lo_img) + max(0.0, hi_img - hi)
        return lost / span


def fit_map(
    leader_range: tuple[float, float],
    follower_range: tuple[float, float],
    direction: int = 1,
) -> JointMap:
    """Derive a centre-aligned mapping from one range onto another.

    Uses 1:1 degrees when the leader's travel fits inside the follower's,
    otherwise scales to fit. Either way both midpoints are aligned, so the
    follower sits mid-travel when the leader does.

    Args:
        leader_range: Measured leader `(min, max)` in degrees.
        follower_range: Follower `(min, max)` in degrees, from its model.
        direction: +1 or -1.

    Returns:
        A `JointMap` whose `scale` and `offset` place the leader's travel
        centred inside the follower's.
    """
    l_lo, l_hi = leader_range
    f_lo, f_hi = follower_range
    l_span, f_span = l_hi - l_lo, f_hi - f_lo
    l_mid, f_mid = (l_lo + l_hi) / 2.0, (f_lo + f_hi) / 2.0

    if l_span <= f_span:
        scale = 1.0
        note = "1:1 degrees, centred (leader travel fits)"
    else:
        scale = f_span / l_span
        note = f"scaled {scale:.3f} to fit (leader travel exceeds follower)"

    # Align midpoints: direction * scale * l_mid + offset == f_mid
    offset = f_mid - direction * scale * l_mid
    return JointMap(
        direction=direction,
        scale=scale,
        offset=offset,
        leader_range=leader_range,
        follower_range=follower_range,
        note=note,
    )


@dataclass
class JointMapper:
    """Configurable six-joint leader -> follower mapping.

    Attributes:
        maps: Per-joint `JointMap`, keyed by AH/B601 joint name. Leader and
            follower share these names, which is what makes joint-space
            mapping viable without FK.

    Example:
        >>> from ah.mapping import JointMapper
        >>> m = JointMapper.from_ranges({"shoulder_pan": (-160.4, 160.4)})
        >>> round(m({"shoulder_pan": 0.0})["shoulder_pan"], 3)
        -0.8
    """

    maps: dict[str, JointMap] = field(default_factory=dict)

    @classmethod
    def from_ranges(
        cls,
        follower_ranges: Mapping[str, tuple[float, float]],
        directions: Mapping[str, int] | None = None,
        joints: tuple[str, ...] = ARM_JOINTS,
    ) -> JointMapper:
        """Build a mapper by fitting measured leader travel onto the follower.

        Args:
            follower_ranges: Follower limits in degrees, normally
                `arm.model_joint_limits()`. Taking them from the live backend
                rather than a constant means the mapping cannot drift out of
                step with the model.
            directions: Per-joint +1/-1. Defaults to `DEFAULT_DIRECTIONS`,
                which were observed on hardware rather than assumed.
            joints: Which joints to map. Defaults to the six arm joints.
        """
        directions = directions if directions is not None else DEFAULT_DIRECTIONS
        maps = {
            name: fit_map(
                LEADER_RANGES[name],
                tuple(follower_ranges[name]),
                int(directions.get(name, DEFAULT_DIRECTIONS.get(name, 1))),
            )
            for name in joints
            if name in follower_ranges
        }
        return cls(maps=maps)

    def flip(self, name: str) -> int:
        """Invert one joint's direction and re-fit it. Returns the new sign.

        Re-fitting matters: `offset` is derived from `direction` in `fit_map`,
        so flipping the sign alone would leave the joint centred on the wrong
        value. This recomputes both, which keeps mid-travel mapped to
        mid-travel.

        Args:
            name: AH joint name.

        Returns:
            The new direction, +1 or -1.

        Raises:
            KeyError: If the joint is not in this mapper.
        """
        current = self.maps[name]
        self.maps[name] = fit_map(
            current.leader_range, current.follower_range, -current.direction
        )
        return self.maps[name].direction

    def directions(self) -> dict[str, int]:
        """Current per-joint signs, for printing or saving."""
        return {name: jm.direction for name, jm in self.maps.items()}

    def flip_argument(self) -> str:
        """The `--flip` value that reproduces the current directions.

        Expressed as a difference from `DEFAULT_DIRECTIONS`, so an empty result
        means "the verified set" rather than "all positive".
        """
        changed = [
            n
            for n, jm in self.maps.items()
            if jm.direction != DEFAULT_DIRECTIONS.get(n, 1)
        ]
        return ",".join(changed) if changed else "(none - defaults are verified)"

    def __call__(self, leader: Mapping[str, float]) -> dict[str, float]:
        """Map leader angles to follower targets, in degrees.

        Keys absent from `self.maps` are ignored, so a leader reading that
        includes the gripper can be passed to a six-joint mapper unchanged.

        Args:
            leader: Leader joint angles in degrees, keyed by joint name.

        Returns:
            Follower targets in degrees, clamped to each joint's limit.
        """
        return {
            name: jm.apply(float(leader[name]))
            for name, jm in self.maps.items()
            if name in leader
        }

    def describe(self) -> str:
        """The mapping table, as documentation. Printed by the mapping tools."""
        rows = [
            f"{'joint':<15}{'dir':>4}{'scale':>7}{'offset':>9}"
            f"{'leader range':>18}{'-> follower':>18}{'limit':>18}{'clip':>7}",
            "-" * 106,
        ]
        for name, jm in self.maps.items():
            img_lo, img_hi = jm.image()
            rows.append(
                f"{name:<15}{jm.direction:>+4}{jm.scale:>7.3f}{jm.offset:>+9.2f}"
                f"{f'{jm.leader_range[0]:.1f} .. {jm.leader_range[1]:.1f}':>18}"
                f"{f'{img_lo:.1f} .. {img_hi:.1f}':>18}"
                f"{f'{jm.follower_range[0]:.1f} .. {jm.follower_range[1]:.1f}':>18}"
                f"{jm.clipped_fraction() * 100:>6.1f}%"
            )
        rows.append("")
        for name, jm in self.maps.items():
            rows.append(f"  {name:<15} {jm.note}")
        return "\n".join(rows)
