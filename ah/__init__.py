"""AH arm-teleoperation stack: physical AM-ARM200 leader -> reBot B601 follower.

Layered so the follower backend is swappable without touching anything above it:

    AMLeaderReader   calibrated Feetech joint positions from the physical leader
    JointMapper      six-joint order, signs, zero offsets, range conversion,
                     safety limits
    AHArm            high-level follower interface: connect, get_joint_positions,
                     set_joint_positions, stop, disconnect
    MuJoCoBackend    the working implementation (this task)
    RealB601Backend  contract for driving the physical arm over CAN later

A real CAN SDK cannot send motor frames to a MuJoCo model, so the abstraction
sits *above* the transport: callers issue AH joint commands and each backend
translates them into MuJoCo control or real motor frames.

UNITS AND NAMES, everywhere above the backend boundary:

  - B601 semantic joint names, matching lerobot's rebot_b601_follower:
    shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw, wrist_roll
  - degrees, not radians
  - the gripper is carried as a seventh name but is a documented follow-on;
    acceptance for this task covers the six arm joints

MuJoCo's own model uses joint1..joint7 in radians, and the gripper is a *slide*
joint in metres. That translation is MuJoCoBackend's problem alone -- it does
not leak upwards.

No ROS 2, no Isaac Sim, no GPU, no FK/IK.
"""

from pathlib import Path

__version__ = "0.1.0"

REPO_ROOT = Path(__file__).resolve().parent.parent

# Fetched by setup.sh rather than vendored: Seeed-Projects/reBot_RS_Sim has no
# LICENSE file, and am2rebot is a public repo.
MODEL_PATH = (
    REPO_ROOT / "vendor" / "reBot_RS_Sim" / "rebotarm_mujoco_rs" / "models" / "rs_arm.xml"
)

# The six driven arm joints, in canonical order. Acceptance scope for this task.
ARM_JOINTS: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
)

# Documented follow-on. Present in the model and in the mapping, excluded from
# the six-joint acceptance criteria.
GRIPPER_JOINT = "gripper"

ALL_JOINTS: tuple[str, ...] = (*ARM_JOINTS, GRIPPER_JOINT)

# AH semantic name -> MuJoCo joint name in rs_arm.xml.
# joint7 is the gripper slide; joint_left/joint_right follow it via equality
# constraints in the MJCF and are not commanded directly.
AH_TO_MJCF: dict[str, str] = {
    "shoulder_pan": "joint1",
    "shoulder_lift": "joint2",
    "elbow_flex": "joint3",
    "wrist_flex": "joint4",
    "wrist_yaw": "joint5",
    "wrist_roll": "joint6",
    "gripper": "joint7",
}

__all__ = [
    "AH_TO_MJCF",
    "ALL_JOINTS",
    "ARM_JOINTS",
    "GRIPPER_JOINT",
    "MODEL_PATH",
    "REPO_ROOT",
    "__version__",
]
