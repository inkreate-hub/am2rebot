#!/usr/bin/env python
"""Map AM-ARM200 leader joint angles onto reBot B601 follower joint angles.

⚠️ SUPERSEDED FOR THE PHYSICAL ARM — USE `tools/mb_teleop.py`. ⚠️

`B601_LIMITS` below is lerobot's `RebotB601FollowerConfig.joint_limits`, which
describes the **DM** arm. Ivan's is the **RS** arm, and once it was zeroed and
measured by hand on 2026-11-12 the two shoulders came out the other way round:

    joint           this table      measured on the real RS arm
    shoulder_lift   -200 ..   1     -0.35 .. +208.79
    elbow_flex      -200 ..   1     -0.18 .. +241.79

Same travel, opposite sign. Run the leader through this table and its mid-pose
lands near -100 deg on both shoulders, so every frame of a teleop session
commands two 36 N.m joints into the stop they are already parked against — the
stop that read 58.7 N.m on the morning of 2026-11-12.

`tools/mb_teleop.py` reads `calibration/b601_ranges.json` instead, the same
measured file `mb_bringup.py --move` and `--replay` already refuse to run
without. It also drives through motorbridge's `add_robstride_motor`, where
`teleop_am_to_b601.py` goes via lerobot's `RebotB601Follower` and its hard-coded
`add_damiao_motor`. This file is kept for the sim path and for its provenance.

Both arms use the same joint names and both speak degrees, so LeRobot will
happily connect them — but the numbers do not line up, for two separate reasons:

1. Different zero conventions. The AM leader zeroes at mid-travel (that is what
   its calibration sweep establishes). The B601 zeroes at its sit-down pose and
   unfolds in one direction only, which is why its shoulder_lift and elbow_flex
   limits are (-200, 1) -- entirely negative. This is an OFFSET.

2. Different travel. wrist_roll is a full-turn motor on the leader (+-170) but
   the B601 allows +-90; the leader gripper is a 0..100 normalised scale while
   the B601 gripper is -270..0 degrees. These are SCALES.

Measured leader ranges this was built against (find_leader_ranges.py,
2026-11-05, id=am_leader):

    shoulder_pan   -118.1 ..  119.7      wrist_yaw     -78.5 ..  80.0
    shoulder_lift  -102.6 ..  103.6      wrist_roll   -170.1 .. 172.3
    elbow_flex      -92.3 ..   92.9      gripper         0.1 ..  99.0
    wrist_flex      -92.4 ..   92.5

Deliberately 1:1 wherever it fits. A full-span stretch would use more of the
follower's envelope but makes teleop feel amplified and unpredictable; matching
degrees to degrees means the follower mirrors your hand.

NOTHING HERE IS DIRECTION-VERIFIED. Signs are the one thing that cannot be
derived from range data -- a joint that reads +90 on the leader may need -90 on
the follower depending on how each arm is assembled. Verify joint by joint on
the real hardware, at low speed, with --robot.max_relative_target set. See
RUNBOOK.md.
"""

from dataclasses import dataclass, field

# B601 follower joint limits, degrees.
# Source: RebotB601FollowerConfig.joint_limits. The follower clips to these
# itself; we clip too so the preview tells the truth about what it will do.
#
# ⚠️ DM-variant limits. Sign-flipped on both shoulders against the measured RS
# arm — see the module docstring. Do not drive the physical arm through these.
B601_LIMITS: dict[str, tuple[float, float]] = {
    "shoulder_pan": (-150.0, 150.0),
    "shoulder_lift": (-200.0, 1.0),
    "elbow_flex": (-200.0, 1.0),
    "wrist_flex": (-80.0, 90.0),
    "wrist_yaw": (-90.0, 90.0),
    "wrist_roll": (-90.0, 90.0),
    "gripper": (-270.0, 0.0),
}


@dataclass
class JointMap:
    """follower_deg = clip(direction * scale * leader_deg + offset, limits)"""

    direction: int = 1
    scale: float = 1.0
    offset: float = 0.0


@dataclass
class AmToB601Mapping:
    joints: dict[str, JointMap] = field(
        default_factory=lambda: {
            # 1:1. Leader reaches +-118 of the follower's +-150; the rest is
            # headroom we simply do not use, which is fine.
            "shoulder_pan": JointMap(direction=1, scale=1.0, offset=0.0),
            # 1:1 shifted into the follower's negative-only window. Leader
            # +-102.6 becomes -202.6..3.6, so roughly 2 degrees clip off each
            # end -- the far extremes of the leader's travel, not useful poses.
            "shoulder_lift": JointMap(direction=1, scale=1.0, offset=-100.0),
            # 1:1 shifted. Leader +-92.3 becomes -192.3..-7.1, entirely inside
            # (-200, 1) with nothing clipped at all.
            "elbow_flex": JointMap(direction=1, scale=1.0, offset=-100.0),
            # Leader +-92.4 against (-80, 90). Scaled slightly rather than
            # clipped so the ends of the leader's travel stay reachable; 0.86
            # maps -92.4..92.5 onto -79.5..79.6.
            "wrist_flex": JointMap(direction=1, scale=0.86, offset=0.0),
            # 1:1, fits as-is.
            "wrist_yaw": JointMap(direction=1, scale=1.0, offset=0.0),
            # Full-turn motor on the leader (+-170) into the follower's +-90.
            # 0.5 keeps the whole leader sweep usable at half rate.
            "wrist_roll": JointMap(direction=1, scale=0.5, offset=0.0),
            # Leader gripper is normalised 0..100, follower is -270..0 degrees.
            # ASSUMES leader 0 == closed. If your gripper opens when you close
            # the leader's, flip to direction=1, offset=-270.
            "gripper": JointMap(direction=-1, scale=2.727, offset=0.0),
        }
    )

    def __call__(self, leader_action: dict[str, float]) -> dict[str, float]:
        """Translate a leader action dict ({"shoulder_pan.pos": ...}) in place of keys."""
        out: dict[str, float] = {}
        for key, value in leader_action.items():
            name = key.removesuffix(".pos")
            jm = self.joints.get(name)
            if jm is None:  # unknown joint: pass through untouched
                out[key] = value
                continue
            mapped = jm.direction * jm.scale * value + jm.offset
            low, high = B601_LIMITS[name]
            out[key] = max(low, min(high, mapped))
        return out

    def would_clip(self, name: str, leader_min: float, leader_max: float) -> tuple[float, float]:
        """Where a leader span lands on the follower, before clipping."""
        jm = self.joints[name]
        a = jm.direction * jm.scale * leader_min + jm.offset
        b = jm.direction * jm.scale * leader_max + jm.offset
        return (min(a, b), max(a, b))


# Leader ranges as measured, used only by the preview below.
MEASURED = {
    "shoulder_pan": (-118.1, 119.7),
    "shoulder_lift": (-102.6, 103.6),
    "elbow_flex": (-92.3, 92.9),
    "wrist_flex": (-92.4, 92.5),
    "wrist_yaw": (-78.5, 80.0),
    "wrist_roll": (-170.1, 172.3),
    "gripper": (0.1, 99.0),
}


def preview() -> None:
    m = AmToB601Mapping()
    print(f"{'joint':<15}{'leader span':>18}{'-> follower':>20}{'limit':>16}  lost")
    print("-" * 82)
    for name, (lmin, lmax) in MEASURED.items():
        fmin, fmax = m.would_clip(name, lmin, lmax)
        low, high = B601_LIMITS[name]
        clipped = max(0.0, low - fmin) + max(0.0, fmax - high)
        span = fmax - fmin
        pct = (clipped / span * 100) if span else 0.0
        print(
            f"{name:<15}{f'{lmin:.1f} .. {lmax:.1f}':>18}"
            f"{f'{fmin:.1f} .. {fmax:.1f}':>20}"
            f"{f'({low:.0f}, {high:.0f})':>16}"
            f"  {pct:4.1f}%"
        )
    print("-" * 82)
    print("'lost' is how much of the mapped span falls outside the follower's limits")
    print("and will be clipped. Directions are UNVERIFIED -- check on hardware.")


if __name__ == "__main__":
    preview()
