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
