"""`RealB601Backend`: the adapter contract for the physical B601, not yet live.

Deliberately a documented stub. Physical CAN actuation is a later milestone, and
a backend that *looks* implemented but has never driven a motor is worse than
one that refuses to connect and tells you what it needs.

What this file is for: pinning down the contract now, while the simulated
backend is being written, so the real one drops in without reshaping anything
above it.

HOW IT WILL WORK

The AH layer passes B601 semantic joint names in degrees. The real arm wants
CAN frames. Three candidate transports, all of which fit behind this same
interface:

1. **lerobot's `rebot_b601_follower`** -- highest level. Already speaks
   `{"shoulder_pan.pos": deg, ...}`, already clips to joint limits, already
   converts to radians internally (`math.radians` in its `send_action`). For
   the RS variant use `--robot.can_adapter=socketcan --robot.port=can0`; the
   DM variant uses the Damiao serial bridge at 921600 baud. Thinnest adapter:
   rename keys, call `send_action`, read back `get_observation`.
2. **Seeed's `reBotArm_control_py`** -- their own Python control and kinematics
   stack. Use if we want their kinematics rather than lerobot's plumbing.
3. **MotorBridge / python-can directly** -- lowest level, MIT or POS_VEL mode
   per joint. Most control, most work, and we would be reimplementing limit
   handling that lerobot already does.

Recommendation when the time comes: start at (1). The simulated backend already
emulates POS_VEL, so the control mode matches and only the transport changes.

⚠️ LEROBOT'S FOLLOWER TARGETS THE **DM** VARIANT, NOT RS

`RebotB601Follower`'s own docstring says "reBot B601-DM follower arm (6-DOF +
gripper, Damiao CAN motors)", and its `MOTOR_MODELS` table is hard-coded to
Damiao part numbers:

    shoulder_pan/lift/elbow -> "4340P"
    wrist_*/gripper         -> "4310"

Ivan has the **RS** (Robstride) variant. Those model strings are passed
straight to motorbridge, so they are wrong for this arm.

The capability exists one layer down: motorbridge 0.3.9 exports
`Mode.ROBSTRIDE_POS_VEL_CSP` alongside the Damiao `POS_VEL` / `MIT` /
`FORCE_POS` modes. So the options are:

  a. Subclass `RebotB601Follower` with an RS motor-model table and the
     Robstride mode. Smallest change, keeps lerobot's limit handling.
  b. Drive motorbridge directly from this backend in
     `Mode.ROBSTRIDE_POS_VEL_CSP`, which is what the simulated backend already
     emulates.
  c. Use Seeed's `reBotArm_control_py`, which is written for their hardware.

Do not assume (a) works until the motor models are confirmed against the
actual RS part numbers -- a wrong model means wrong scaling on every command.

⚠️ THE SIMULATED MAPPING DOES NOT TRANSFER

`ah/mapping.py` is fitted against the MuJoCo model's limits. The real arm's
are different, and not by a little:

    joint           rs_arm.xml (sim)      lerobot (real)
    shoulder_pan    -160.4 ..  160.4      -150 .. 150
    shoulder_lift    -11.5 ..  179.9      -200 ..   1     <-- sign flipped
    elbow_flex       -11.5 ..  179.9      -200 ..   1     <-- sign flipped
    wrist_flex       -90.0 ..   90.0       -80 ..  90
    wrist_yaw        -90.0 ..   90.0       -90 ..  90
    wrist_roll      -179.9 ..  179.9       -90 ..  90     <-- half the travel
    gripper            0.0 ..  100.0      -270 ..   0     <-- different unit

shoulder_lift and elbow_flex are almost entirely POSITIVE in the model and
almost entirely NEGATIVE on the real arm: different zero conventions, roughly
180 deg apart. The gripper is a normalised 0-100 slide in the model and
-270..0 DEGREES on the real arm.

Because `JointMapper.from_ranges` takes the follower's limits from the live
backend, the offsets re-derive themselves once `model_joint_limits()` returns
the real numbers. **The directions do not.** They were verified against the
MJCF's joint axes, which are not the motors' axes, so every sign has to be
re-verified on the physical arm exactly as it was on the simulated one.

WHAT MUST HOLD FOR THE SWAP TO BE SAFE

- `model_joint_limits()` must return the *real* mechanism's limits. These are
  **not** the same numbers as the MuJoCo model: `rs_arm.xml` gives
  shoulder_lift and elbow_flex as -11.5 .. 179.9 deg, while lerobot's
  `RebotB601FollowerConfig.joint_limits` gives -200 .. 1 deg. Different zero
  conventions. Getting this wrong sends the arm to the far end of its travel on
  the first command.
- `stop()` must be documented as either "hold position" or "torque off". The
  base class holds position. On a 48 V arm with a 2.5 kg payload that
  distinction matters, so decide it explicitly rather than inheriting.
- `RebotB601FollowerConfig.max_relative_target` defaults to `None`, i.e. **no
  cap on per-step jumps**. The AH layer's own `max_step_deg` and
  `max_speed_dps` already bound this, but set the follower's cap too -- defence
  in depth, since a bug above could otherwise reach the motors unbounded.
- The B601 does not persist calibration. It zeroes against whatever pose it is
  physically holding at connect, every session, so `connect()` must require the
  arm to be in its sit-down pose or refuse.
"""

from __future__ import annotations

from collections.abc import Mapping

from ..arm import AHArm, SafetyLimits


class RealB601Backend(AHArm):
    """Physical reBot B601 follower over CAN. **Not implemented yet.**

    Constructing it is fine -- useful for type checks, docs and tests -- but
    `connect()` raises, on purpose. See the module docstring for the intended
    transport and the constraints that must hold before this goes live.

    Args:
        port: `can0` for SocketCAN on the RS variant, or the Damiao serial
            bridge device (e.g. `/dev/ttyACM0`) for the DM variant.
        can_adapter: `"socketcan"` or `"damiao"`.
        limits: Safety caps. Supply real mechanism limits explicitly; do not
            inherit the simulator's.

    Example:
        >>> arm = RealB601Backend(port="can0")
        >>> arm.is_connected
        False
    """

    def __init__(
        self,
        port: str = "can0",
        can_adapter: str = "socketcan",
        limits: SafetyLimits | None = None,
    ) -> None:
        super().__init__(limits)
        self.port = port
        self.can_adapter = can_adapter

    def _open(self) -> None:
        raise NotImplementedError(
            "RealB601Backend is a contract stub -- physical CAN actuation is a "
            "later milestone.\n"
            "To implement: wrap lerobot's rebot_b601_follower (it already takes "
            "degrees under these joint names), supply the REAL mechanism joint "
            "limits rather than the MuJoCo model's, set its "
            "max_relative_target, and require the sit-down pose at connect "
            "because the arm re-zeroes every session.\n"
            "See ah/backends/real_b601.py for the full contract."
        )

    def _close(self) -> None:
        return

    def _read_positions(self) -> Mapping[str, float]:
        raise NotImplementedError

    def _write_targets(self, targets: Mapping[str, float]) -> None:
        raise NotImplementedError

    def model_joint_limits(self) -> dict[str, tuple[float, float]]:
        raise NotImplementedError(
            "Must return the real mechanism's limits in degrees -- NOT the "
            "MuJoCo model's. rs_arm.xml and lerobot's "
            "RebotB601FollowerConfig.joint_limits use different zero "
            "conventions and disagree by ~180 deg on shoulder_lift/elbow_flex."
        )
