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

The capability exists one layer down. motorbridge 0.3.9 has full Robstride
support:

    Controller.from_socketcanfd(channel="can0")
    Controller.add_robstride_motor(motor_id, feedback_id, model)
    Motor.robstride_send_pos_vel_csp(...)      # Mode.ROBSTRIDE_POS_VEL_CSP = 5
    Motor.robstride_ping / robstride_get_fault_report / set_zero_position

RS HARDWARE FACTS, from Seeed's B601-RS wiki:

    joint        CAN id   model
    J1 shoulder_pan   1    RS06
    J2 shoulder_lift  2    RS06
    J3 elbow_flex     3    RS06
    J4 wrist_flex     4    RS00
    J5 wrist_yaw      5    RS00
    J6 wrist_roll     6    RS00
    J7 gripper        7    RS00

    supply   48 V DC / 12.5 A, per Seeed's own reBot Arm 102 + B601-RS training
             guide. Ours is a Mean Well **LRS-600-48** (48 V, 12.5 A, 600 W),
             which meets that exactly -- some resellers suggest 25 A "for full
             performance", so 12.5 A is the floor, not headroom.

             ⚠️ LRS-600 input range is **switch-selectable** 90-132 / 180-264
             VAC. German mains is 230 V. If the switch arrives on 115 V, mains
             destroys the unit. Check it before the plug goes in.

             ⚠️ Overload protection is hiccup mode -- constant-current limit,
             auto-recovering. If a fast multi-joint move trips it, the rail dips
             and **all seven motors lose torque simultaneously and the arm
             falls**. There is no graceful degradation. This is a reason to keep
             `max_speed_dps` conservative on the real backend, not just a
             reason to buy a bigger PSU.

             Quoted motor currents are PHASE current, not bus current: RS06 is
             14.3 Apk rated / 57 Apk peak, RS00 4.7 / 15.5 Apk. Do not sum those
             and conclude the arm needs 40+ A from the supply -- in FOC the DC
             draw is a fraction of phase current. It converges upward only when
             holding high torque against a stall.
    CAN      1 Mbit/s, **classic CAN, not CAN FD**
    setup    sudo ip link set can0 down
             sudo ip link set can0 up type can bitrate 1000000

Two mismatches to resolve before trusting any of this:

  - motorbridge only exposes `from_socketcanfd`, but the arm is classic CAN at
    1 Mbit/s. A CAN FD controller does carry classic frames, so this probably
    works -- but "probably" is not good enough before energising a 48 V arm.
    Verify with `robstride_ping` on one motor and nothing else connected.
  - lerobot's `motor_can_ids` pairs each joint with a feedback id (0x11..0x17).
    That is a Damiao convention; Seeed document the RS motors as plain ids
    1..7. Confirm the RS feedback-id scheme before adding motors.

LEROBOT ALREADY HAS A ROBSTRIDE BUS -- and it settles the FD question

`lerobot/motors/robstride/` ships `RobstrideMotorsBus`, built on python-can
rather than motorbridge, and its constructor takes **`use_can_fd: bool = True`**
and `bitrate: int = 1_000_000`. So classic CAN at 1 Mbit/s is a supported
configuration on this path, where motorbridge only exposes `from_socketcanfd`.
That removes the one blocker that could not be resolved by reading alone:

    RobstrideMotorsBus(port="can0", motors={...},
                       can_interface="socketcan",
                       use_can_fd=False, bitrate=1_000_000)

Two caveats, both from the source:

  - `tables.py` names the models **`O0` .. `O6`** (plus `ELO5`), not `RS00` /
    `RS06`. Numerically 1:1 with Robstride's part numbers, so RS00 -> "O0" and
    RS06 -> "O6" -- but that correspondence is INFERRED from the numbering, not
    documented, and the limit table makes it worth checking: `O6` carries
    `PMAX 112.5 rad` against `12.57` (= 4*pi) for every other model, which
    suggests a different reduction. Wrong model means wrong scaling on every
    command, so confirm against one motor before adding seven.
  - The file's own first line is
    `TODO(Virgile): only the MIT protocole is implemented for now`. `POS_VEL`
    exists in its `ControlMode` enum but is not wired up. The simulated backend
    emulates POS_VEL, so on this path the control mode does NOT match -- MIT is
    a torque law with kp/kd, which is closer to the cascade in
    `MuJoCoBackend.step()` than to a position interface. `RebotB601Follower`
    already defaults to `control_mode="mit"` with per-joint gains, which is a
    useful starting point for the gains.

So the options are:

  a. Seeed's own `rebot` package (`ReBotRSMITController`), which is written for
     this exact arm and reads its motor table from
     `config/rebotarm_rs.yaml`. Shortest path to a moving arm. **Not installed
     on this machine.**
  b. `RobstrideMotorsBus` with `use_can_fd=False`, driven from this backend.
     Needs `python-can` (also not installed) and MIT gains, but it is in-tree,
     Apache-2.0, and already handles handshake, enable/disable, fault clear and
     `set_zero_position`.
  c. Drive motorbridge directly in `Mode.ROBSTRIDE_POS_VEL_CSP`, matching what
     the simulated backend emulates -- but motorbridge is FD-only, which is the
     unverified part.
  d. Subclass `RebotB601Follower` with an RS motor table. Keeps lerobot's limit
     handling, but that class is DM-shaped throughout: it calls
     `add_damiao_motor` unconditionally, for both the `damiao` and the
     `socketcan` adapter.

Recommendation: (a) if Seeed's package can be obtained, purely to confirm the
arm moves and that the ids and models are right. Otherwise (b), behind this
interface so the AH stack keeps owning the safety.

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

    #: Robstride motor model per joint, from Seeed's B601-RS wiki. The three
    #: shoulder joints carry the larger RS06; wrists and gripper are RS00.
    #: These strings go straight to motorbridge's add_robstride_motor, and a
    #: wrong model means wrong scaling on every command.
    #:
    #: ⚠️ lerobot's `RobstrideMotorsBus` uses different strings for the same
    #: motors -- `O0` .. `O6`, see `LEROBOT_MODEL_NAMES` below. Which table
    #: applies depends on which transport this backend ends up using.
    RS_MOTOR_MODELS: dict[str, str] = {
        "shoulder_pan": "RS06",
        "shoulder_lift": "RS06",
        "elbow_flex": "RS06",
        "wrist_flex": "RS00",
        "wrist_yaw": "RS00",
        "wrist_roll": "RS00",
        "gripper": "RS00",
    }

    #: CAN id per joint, 1..7 in joint order.
    RS_CAN_IDS: dict[str, int] = {
        "shoulder_pan": 1,
        "shoulder_lift": 2,
        "elbow_flex": 3,
        "wrist_flex": 4,
        "wrist_yaw": 5,
        "wrist_roll": 6,
        "gripper": 7,
    }

    #: Same motors as `RS_MOTOR_MODELS`, named the way lerobot's
    #: `motors/robstride/tables.py` names them. The mapping is INFERRED from the
    #: part numbering (RS00 -> O0, RS06 -> O6) and is not documented anywhere;
    #: verify against one motor before trusting it, especially O6, whose PMAX is
    #: 112.5 rad where every other model is 12.57.
    LEROBOT_MODEL_NAMES: dict[str, str] = {"RS00": "O0", "RS06": "O6"}

    #: Classic CAN at 1 Mbit/s. NOT CAN FD, despite motorbridge only exposing
    #: from_socketcanfd. lerobot's RobstrideMotorsBus takes `use_can_fd=False`
    #: at this bitrate, so that path does support classic frames.
    CAN_BITRATE = 1_000_000

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
            "later milestone.\n\n"
            "This arm is the RS (Robstride) variant, so lerobot's "
            "rebot_b601_follower is NOT a drop-in: it is written for the DM "
            "and its motor table is hard-coded to Damiao part numbers.\n\n"
            "Before anything is energised:\n"
            f"  sudo ip link set {self.port} down\n"
            f"  sudo ip link set {self.port} up type can bitrate "
            f"{self.CAN_BITRATE}\n"
            "  48 V supply, arm parked in its sit-down pose (it re-zeroes "
            "every session)\n\n"
            "Then: ping one motor with motorbridge's robstride_ping before "
            "adding all seven, supply the REAL joint limits rather than the "
            "MuJoCo model's, set max_relative_target, and re-verify every "
            "joint direction on hardware -- the simulated signs were measured "
            "against the MJCF's axes, not the motors'.\n\n"
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
