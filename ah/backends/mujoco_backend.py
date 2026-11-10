"""`MuJoCoBackend`: the simulated reBot B601-RS follower.

Loads Seeed's `rs_arm.xml` and drives it with the cascade controller that model
is built for. Headless, CPU-only -- measured 63x realtime on an Intel Iris Xe
machine with no GPU.

WHY A CONTROLLER IS NEEDED AT ALL

`rs_arm.xml` actuators are `<motor>`, i.e. **torque** sources with `ctrlrange`
in N*m (+-36 on the shoulder joints, +-14 on the wrist). The model's own comment
says the position and velocity loops live in Python, matching the real motors'
POS_VEL mode. So this backend implements that cascade:

    position error -> desired velocity (kp, capped)
    velocity error -> torque           (kv, clamped to ctrlrange)

That is deliberately *not* a perfect position servo, and that is the point:
`target` and `position` genuinely differ, so follower tracking error is a real
measurement rather than a tautology. A `<position>` actuator would have made
the tracking plots meaningless.

UNITS AT THE BOUNDARY

Above this class everything is B601 semantic names in degrees. Inside:

  - arm joints are radians (`compiler angle="radian"` in the MJCF)
  - the gripper is `joint7`, a **slide** joint in metres, 0 .. 0.05

The gripper is a documented follow-on, excluded from the six-joint acceptance
scope. It is wired up but off by default -- pass `include_gripper=True`.
"""

from __future__ import annotations

import math
from collections.abc import Mapping

import mujoco
import numpy as np

from .. import AH_TO_MJCF, ARM_JOINTS, GRIPPER_JOINT, MODEL_PATH
from ..arm import AHArm, SafetyLimits

#: Gripper slide travel, metres, from `joint7` in the MJCF. The AH-level
#: gripper command is 0..100 (matching the leader's normalised scale) and is
#: converted to metres here.
GRIPPER_TRAVEL_M = 0.05

#: Reflected rotor inertia added to each arm DoF, kg*m^2.
#:
#: `rs_arm.xml` specifies no `armature`, which makes the low-inertia wrist
#: joints numerically stiff: with kp=12/kv=6 the velocity loop goes into a
#: bang-bang limit cycle -- measured ctrl pinned at the +-14 N*m rail and qvel
#: alternating +-2.53 rad/s while the joint sat still, leaving 0.8 deg of
#: standing error. The heavier shoulder joints were unaffected, which is why it
#: looked like a per-joint mystery.
#:
#: This is a modelling omission, not a tuning problem: a geared actuator really
#: does carry rotor inertia reflected through the gearbox squared, and it is
#: what keeps a real harmonic-drive joint stable at these gains. Measured sweep:
#:
#:   armature  kp/kv   steady err   chatter
#:   0.00      12/6    -0.796 deg   yes
#:   0.01      12/6     0.000 deg   no
#:   0.05      12/6     0.000 deg   no
#:
#: 0.01 is the smallest value that removes it, so it perturbs the dynamics least.
ARMATURE = 0.01

#: Self-collision pairs that must be excluded, or the arm cannot move at all.
#:
#: `rs_arm.xml` is designed to be `<include>`d into a scene that supplies these;
#: Seeed's own `rs_grasp_scene.xml` carries exactly this list. Loaded standalone
#: the arm self-collides in every pose -- measured at the zero pose:
#: base_link/link1 interpenetrating by 10.4 mm and link2/link5 touching,
#: producing 5 contacts whose constraint force (-35.97 N*m) cancels the full
#: 36 N*m actuator torque. The joints appear mechanically seized: 12 N*m of
#: commanded torque yielded a constant 0.006 rad/s.
#:
#: We inject them with MjSpec rather than shipping our own scene file, which
#: keeps MuJoCo's `meshdir` resolution pointing at the vendored mesh directory.
#:
#: Source: Seeed-Projects/reBot_RS_Sim,
#: rebotarm_mujoco_rs/models/rs_grasp_scene.xml
SELF_COLLISION_EXCLUDES: tuple[tuple[str, str], ...] = (
    ("world", "link1"),
    ("base_link", "link1"),
    ("link1", "link2"),
    ("link2", "link3"),
    ("link2", "link5"),
    ("link3", "link4"),
    ("link3", "link5"),
    ("link4", "link5"),
    ("link5", "link6"),
    ("gripper_left", "gripper_right"),
)


class MuJoCoBackend(AHArm):
    """Simulated B601-RS follower.

    Args:
        model_path: MJCF to load. Defaults to the fetched Seeed `rs_arm.xml`.
        limits: Safety caps. Joint limits are filled from the model on connect,
            so normally you only set `max_step_deg` / `max_speed_dps`.
        include_gripper: Command `joint7` as well as the six arm joints. Off by
            default -- the gripper is a follow-on, not part of acceptance.
        kp: Position-loop gain, 1/s. Position error in rad becomes desired
            velocity in rad/s.
        kv: Velocity-loop gain, N*m per rad/s.
        max_joint_vel: Desired-velocity clamp, rad/s. Bounds how fast the
            controller will *try* to move, independently of the AH-level
            `max_speed_dps`, which bounds how fast targets may change.
        substeps_per_command: Physics steps advanced per `step()` call. With the
            model's 0.002 s timestep, 15 substeps is 30 ms, i.e. a 33 Hz control
            loop.
        armature: Reflected rotor inertia per arm DoF, kg*m^2. See `ARMATURE` --
            the MJCF omits it and the wrist joints chatter without it.
        kp_gripper: Gripper PD stiffness, N/m. The gripper uses a plain PD on
            position rather than the arm's velocity cascade -- see `step()`.
        kd_gripper: Gripper PD damping, N per m/s.
        offscreen_size: `(width, height)` of the offscreen framebuffer, for
            `mujoco.Renderer`. The MJCF declares none, so the 640x480 default
            otherwise caps rendered frames.
        scenery: Add non-colliding lights and a floor so renders are legible.
            Off by default -- headless physics does not need it.

    Example:
        >>> from ah.backends import MuJoCoBackend
        >>> arm = MuJoCoBackend()
        >>> arm.connect()
        >>> arm.set_joint_positions({"shoulder_pan": 10.0})
        >>> for _ in range(50):
        ...     arm.step()
        >>> round(arm.get_joint_positions()["shoulder_pan"], 1)
        10.0
        >>> arm.disconnect()
    """

    def __init__(
        self,
        model_path: str | None = None,
        limits: SafetyLimits | None = None,
        include_gripper: bool = False,
        kp: float = 12.0,
        kv: float = 6.0,
        max_joint_vel: float = 2.0,
        substeps_per_command: int = 15,
        armature: float = ARMATURE,
        kp_gripper: float = 6000.0,
        kd_gripper: float = 60.0,
        offscreen_size: tuple[int, int] = (1280, 960),
        scenery: bool = False,
    ) -> None:
        super().__init__(limits)
        self.model_path = str(model_path or MODEL_PATH)
        self.include_gripper = include_gripper
        self.joint_names = (*ARM_JOINTS, GRIPPER_JOINT) if include_gripper else ARM_JOINTS
        self.kp = kp
        self.kv = kv
        self.max_joint_vel = max_joint_vel
        self.substeps_per_command = substeps_per_command
        self.armature = armature
        self.kp_gripper = kp_gripper
        self.kd_gripper = kd_gripper
        self.offscreen_size = offscreen_size
        self.scenery = scenery

        self.model: mujoco.MjModel | None = None
        self.data: mujoco.MjData | None = None
        self._qpos_adr: dict[str, int] = {}
        self._dof_adr: dict[str, int] = {}
        self._act_id: dict[str, int] = {}
        self._sim_time = 0.0

    # ---------------------------------------------------------------- lifecycle

    def _open(self) -> None:
        # Load through MjSpec so the self-collision excludes can be added
        # before compiling. See SELF_COLLISION_EXCLUDES -- without these the
        # arm is seized in every pose.
        spec = mujoco.MjSpec.from_file(self.model_path)
        for body1, body2 in SELF_COLLISION_EXCLUDES:
            exclude = spec.add_exclude()
            exclude.bodyname1 = body1
            exclude.bodyname2 = body2

        # Offscreen framebuffer for mujoco.Renderer. rs_arm.xml declares no
        # <visual><global offwidth>, so the default 640x480 caps any render and
        # raises rather than downscaling. Must be set on the spec: mutating
        # model.vis.global_ after compile leaves the EGL context inconsistent
        # and the renderer dies with an unprintable EGLError.
        spec.visual.global_.offwidth = self.offscreen_size[0]
        spec.visual.global_.offheight = self.offscreen_size[1]

        if self.scenery:
            self._add_scenery(spec)

        self.model = spec.compile()

        # implicitfast integrates joint damping implicitly, which is the right
        # choice for a damped, actuator-driven chain. Not what fixed the
        # chatter -- armature was -- but it costs nothing and is more robust.
        self.model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST

        for name in ARM_JOINTS:
            jid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, AH_TO_MJCF[name]
            )
            self.model.dof_armature[self.model.jnt_dofadr[jid]] = self.armature

        self.data = mujoco.MjData(self.model)

        for name in self.joint_names:
            mjcf = AH_TO_MJCF[name]
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, mjcf)
            if jid < 0:
                raise RuntimeError(f"joint '{mjcf}' not in {self.model_path}")
            self._qpos_adr[name] = int(self.model.jnt_qposadr[jid])
            self._dof_adr[name] = int(self.model.jnt_dofadr[jid])
            aid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{mjcf}_motor"
            )
            if aid < 0:
                raise RuntimeError(f"actuator '{mjcf}_motor' not in {self.model_path}")
            self._act_id[name] = int(aid)

        # Start from the model's keyframe if it has one, else the zero pose,
        # then settle so the arm is not mid-fall on the first read.
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self._sim_time = 0.0

    @staticmethod
    def _add_scenery(spec: mujoco.MjSpec) -> None:
        """Add lights and a ground plane -- purely so renders are legible.

        `rs_arm.xml` is an arm, not a scene: it carries no light and no floor,
        so rendered straight it is a dim object floating in black. Seeed's own
        scene files supply these.

        The floor is given `contype=0 conaffinity=0`, i.e. visual only. A
        colliding floor would change the physics, and the arm's base already
        sits at the origin where a plane would intersect it.
        """
        spec.worldbody.add_light(
            pos=[0.6, -0.6, 1.4], dir=[-0.4, 0.4, -1.0],
            type=mujoco.mjtLightType.mjLIGHT_SPOT, cutoff=60.0, exponent=8.0,
            diffuse=[0.8, 0.8, 0.8], specular=[0.25, 0.25, 0.25], castshadow=1,
        )
        spec.worldbody.add_light(
            pos=[-0.8, 0.5, 1.0], dir=[0.5, -0.3, -1.0],
            type=mujoco.mjtLightType.mjLIGHT_SPOT, cutoff=70.0, exponent=4.0,
            diffuse=[0.4, 0.4, 0.45], castshadow=0,
        )
        spec.worldbody.add_geom(
            name="ah_visual_floor",
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            size=[1.2, 1.2, 0.02],
            pos=[0.0, 0.0, 0.0],
            rgba=[0.26, 0.28, 0.31, 1.0],
            contype=0,
            conaffinity=0,
        )

    def _close(self) -> None:
        self.model = None
        self.data = None

    # ----------------------------------------------------------------- plumbing

    def _read_positions(self) -> Mapping[str, float]:
        assert self.data is not None
        out: dict[str, float] = {}
        for name in self.joint_names:
            raw = float(self.data.qpos[self._qpos_adr[name]])
            out[name] = self._from_native(name, raw)
        return out

    def _write_targets(self, targets: Mapping[str, float]) -> None:
        """Store targets. Torque is computed in `step()`, where dt is known."""
        self._target_native = {
            name: self._to_native(name, value) for name, value in targets.items()
        }

    def model_joint_limits(self) -> dict[str, tuple[float, float]]:
        """Joint limits in AH units, read from the MJCF `range` attributes.

        Authoritative for this task. Note lerobot's `rebot_b601_follower`
        carries a *different* set of limits; the model wins here because it is
        what actually constrains the simulated mechanism.
        """
        assert self.model is not None
        out: dict[str, tuple[float, float]] = {}
        for name in self.joint_names:
            jid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, AH_TO_MJCF[name]
            )
            lo, hi = (float(v) for v in self.model.jnt_range[jid])
            out[name] = (self._from_native(name, lo), self._from_native(name, hi))
        return out

    # ------------------------------------------------------------------ physics

    def step(self, substeps: int | None = None) -> None:
        """Advance the simulation, running the POS_VEL cascade each substep.

        Call once per control-loop iteration. The controller runs at physics
        rate rather than control rate, which is what keeps a 33 Hz command
        stream stable against a 500 Hz physics step.

        Args:
            substeps: Physics steps to advance. Defaults to
                `substeps_per_command`.
        """
        self._require_connected()
        assert self.model is not None and self.data is not None
        targets = getattr(self, "_target_native", None)
        if targets is None:
            targets = {
                name: self._to_native(name, v) for name, v in self._targets.items()
            }

        for _ in range(substeps if substeps is not None else self.substeps_per_command):
            for name in self.joint_names:
                qadr, dadr = self._qpos_adr[name], self._dof_adr[name]
                pos_err = targets[name] - float(self.data.qpos[qadr])

                aid = self._act_id[name]
                lo, hi = self.model.actuator_ctrlrange[aid]

                if name == GRIPPER_JOINT:
                    # The gripper gets a plain PD on position, not the velocity
                    # cascade, and no bias feedforward. Three reasons:
                    #
                    #  - It is a SLIDE joint in metres over 0.05 m of travel,
                    #    not a hinge in radians, so the arm's gains are
                    #    meaningless here.
                    #  - joint_left and joint_right follow it through very
                    #    stiff equality constraints (solref 0.001). qfrc_bias
                    #    on that DoF is dominated by constraint forces rather
                    #    than gravity, so feeding it forward drove the actuator
                    #    to its -500 N rail and pushed the jaw past its own
                    #    lower limit.
                    #  - The MJCF says so: its comment specifies joint7 is
                    #    driven by a PD controller in Python.
                    force = (
                        self.kp_gripper * pos_err
                        - self.kd_gripper * float(self.data.qvel[dadr])
                    )
                    self.data.ctrl[aid] = float(np.clip(force, lo, hi))
                    continue

                # Position loop -> desired velocity, clamped so a large error
                # does not demand an impossible rate.
                vel_des = float(
                    np.clip(self.kp * pos_err, -self.max_joint_vel, self.max_joint_vel)
                )
                # Velocity loop -> torque, clamped to the actuator's ctrlrange,
                # which carries the real motor's output limit.
                vel_err = vel_des - float(self.data.qvel[dadr])

                # Gravity/bias feedforward. Without it this cascade has no
                # integral term, so holding against gravity requires a standing
                # velocity error, which means a standing POSITION error -- the
                # arm settles short of its target by an amount that grows with
                # load. Measured before adding this: a 1 deg command arrived as
                # 0.59 deg, a 2 deg command as 1.57 deg.
                #
                # qfrc_bias is gravity + Coriolis + centrifugal for this DoF,
                # i.e. exactly the torque needed to stand still. The real
                # motors' POS_VEL firmware does the equivalent internally.
                bias = float(self.data.qfrc_bias[dadr])
                self.data.ctrl[aid] = float(np.clip(self.kv * vel_err + bias, lo, hi))
            mujoco.mj_step(self.model, self.data)
            self._sim_time += float(self.model.opt.timestep)

    @property
    def sim_time(self) -> float:
        """Simulated seconds elapsed since connect."""
        return self._sim_time

    def settle(self, seconds: float = 1.0) -> None:
        """Hold the current targets for a while so the arm reaches them.

        Useful after `connect()` or a large move, before taking a measurement.
        """
        assert self.model is not None
        self.step(int(seconds / float(self.model.opt.timestep)))

    # ------------------------------------------------------------- unit bridges

    def _to_native(self, name: str, value: float) -> float:
        """AH units (degrees, or 0-100 gripper) -> MuJoCo units (rad, or m)."""
        if name == GRIPPER_JOINT:
            return max(0.0, min(1.0, value / 100.0)) * GRIPPER_TRAVEL_M
        return math.radians(value)

    def _from_native(self, name: str, value: float) -> float:
        """MuJoCo units -> AH units."""
        if name == GRIPPER_JOINT:
            return value / GRIPPER_TRAVEL_M * 100.0
        return math.degrees(value)
