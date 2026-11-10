# Bringing up the physical reBot B601-RS

From an unpowered arm to leader-driven teleoperation, in seven stages with a
hard gate on each. **Do not skip a gate.** Every stage is either read-only or
bounded to one joint, and the first stage that applies torque is stage 5 — by
which point three separate things that can silently destroy a run have already
been measured rather than assumed.

The simulated follower is already working end to end
(`tools/teleop_sim.py`), so this is a transport swap, not new control work. What
does **not** transfer is covered in `ah/backends/real_b601.py`: the joint
limits, the joint directions, the gripper's units, and the control mode.

---

## Why this order

Three facts drive it.

**1. The motor model string is a 9× scale factor, and it is a guess.**
lerobot's `RobstrideMotorsBus` encodes every position command as a 16-bit
fraction of `±PMAX`, taken from a per-model table:

| model | PMAX | encode range | resolution |
|---|---|---|---|
| `O0` (and O1-O5, ELO5) | 12.57 rad | ±720.2° | 0.0220°/tick |
| `O6` | 112.5 rad | ±6445.8° | 0.1967°/tick |

Seeed's wiki says J1-J3 are **RS06** and J4-J7 are **RS00**. lerobot names its
models `O0`..`O6`. RS00→`O0` and RS06→`O6` is *inferred from the numbering and
documented nowhere*. Get it wrong and every command and every readback on the
three shoulder joints is off by **112.5/12.57 = 8.95×**.

The failure is asymmetric:

- host PMAX too **large** → commands compress toward zero → the joint
  under-moves ~9×. Annoying, safe.
- host PMAX too **small** → commands expand → a 10° command becomes ~90°.
  On a 48 V arm with 36 N·m at the shoulder, that is the one that breaks
  something.

So the shoulders start as `O6` (the safe direction of the error) and stage 3
*measures* the factor with the motors de-energised.

**2. The arm has no persistent calibration.** It zeroes against whatever pose
it is physically holding, every session (`set_zero_position`, CAN cmd `0xFE`).
Park it in the sit-down pose before connecting, every time.

**3. MIT mode is a torque law, not a position interface.** lerobot's Robstride
bus only implements MIT (`tables.py` line 15: *"only the MIT protocole is
implemented for now"*). The motor enforces **no position limits whatsoever** —
it applies `kp·(q_target − q) + kd·(0 − q̇)` and will happily drive through a
hard stop. Every limit is ours. `AHArm.set_joint_positions` already enforces
them in the non-abstract method, so no backend can bypass it, but the numbers
fed into it must be the real mechanism's.

---

## Stage 0 — PSU on the bench, arm not connected

Mean Well **LRS-600-48**: 48 V / 12.5 A / 600 W. Matches Seeed's spec for the
B601-RS exactly, which also means there is no headroom.

- [ ] **Input selector switch on 230 V.** LRS-600 is switch-selectable
      90-132 / 180-264 VAC. On German mains, a unit shipped in the 115 V
      position is destroyed the moment it is plugged in. Check first.
- [ ] Earth the ⏚ terminal. Also gives CAN a ground reference.
- [ ] Power up **with nothing connected**, meter `+V`/`−V`.
- [ ] Trim V-ADJ to **48.0 V**. Do not trim up for headroom: the motors regen
      on deceleration, the LRS cannot sink current, so the rail rises above
      whatever you set.
- [ ] Terminal cover on, ferrules on the wires, inline ≈15 A fuse, DC switch on
      the arm side.

> Seeed: *"disconnect the power before connecting or disconnecting JST, XT30 or
> other cables."* Never hot-plug the XT30.

**Gate 0:** 48.0 ± 0.2 V no-load, terminals covered, nothing else connected.

**Know the failure mode before you go further:** LRS overload protection is
*hiccup mode* — constant-current limit with automatic recovery. If a fast
multi-joint move trips it, the rail dips and **all seven motors lose torque
simultaneously and the arm falls.** It then recovers, after the arm has already
dropped. There is no graceful degradation. This is why the speed caps in stages
5-7 are not optional paperwork.

---

## Stage 1 — CAN link, raw frames only

Nothing on this machine exists yet: no CAN interface, no `can` kernel modules,
no `can-utils`, no `python-can`. Install the host side first.

```bash
sudo apt install can-utils
/home/inko/.local/uvenv/bin/uv pip install \
    --python ./lerobot_alohamini/.venv/bin/python python-can
```

Plug in the USB-CAN adapter and bring the link up **classic, 1 Mbit/s**:

```bash
tools/can_up.sh can0           # or by hand:
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0     # expect: state UP, bitrate 1000000
```

Now power the arm and watch raw traffic, with **no software driving it**:

```bash
candump -tz can0
```

Then, in another terminal, the read-only probe — it sends one `CLEAR_FAULT`
(`0xFB`) poll per id and reports who answers:

```bash
./lerobot_alohamini/.venv/bin/python tools/can_check.py --port can0
```

**Gate 1:** all seven ids answer. Record what `data[0]` of each reply contains —
this settles the feedback-id question. lerobot keys its decode cache on
`data[0]` via `_id_to_name`, built from `recv_id if recv_id is not None else
id`. Seeed document the RS motors as plain ids 1..7; lerobot's *Damiao* config
pairs each with a feedback id `0x11..0x17`. If `data[0]` is the device id, leave
`recv_id=None`. If it is `0x10+id`, set `recv_id` accordingly or every frame is
dropped as "Unmapped CAN frame".

**Abort if:** fewer than seven answer. Do not proceed with a partial bus — a
missing shoulder joint means the arm is held by six motors and dropped by one.

---

## Stage 2 — Handshake and read, still no torque

```bash
./lerobot_alohamini/.venv/bin/python tools/rs_probe.py --port can0
```

Opens `RobstrideMotorsBus(use_can_fd=False, bitrate=1_000_000)`, handshakes all
seven, and prints position / velocity / torque / MOSFET temperature. Torque is
never enabled.

**Gate 2:** 7/7 handshake clean, no faults, positions stable and repeatable over
30 s. Temperatures sane (ambient-ish).

**Abort if:** the handshake reports a fault — `_handshake` raises on any motor
that answers with the fault bit set. Clear it and find out why before going on.

---

## Stage 3 — Measure the PMAX scale factor by hand

**The most important stage, and it is read-only.** Torque stays off; you move
the joint with your hand.

```bash
./lerobot_alohamini/.venv/bin/python tools/rs_probe.py --port can0 \
    --backdrive shoulder_pan
```

It samples the readback while you rotate the joint, and prints the travel as it
would decode under *every* candidate PMAX. Move the joint through a physically
known angle — a quarter turn, or stop-to-stop with a protractor — and read off
which column matches reality.

```
moved: 1206 ticks
   model   PMAX     implied travel
   O0      12.57      26.5 deg
   O6     112.5      237.3 deg      <-- matches a ~240 deg sweep
```

**Gate 3:** for each of the seven joints, one candidate matches the physical
motion within ~10%, and it is the same candidate for all three shoulder joints
and the same for all four wrist/gripper joints.

**Abort if:** nothing matches, or the shoulders disagree with each other. Then
the model table is wrong in a way we do not understand, and nothing should be
energised. Fall back to Seeed's own `rebot` package
(`ReBotRSMITController`, motor table in `config/rebotarm_rs.yaml`) to find out
what they use.

Write the answer into `RS_MOTOR_MODELS` / `LEROBOT_MODEL_NAMES` in
`ah/backends/real_b601.py` with a note saying it was measured, not inferred.

---

## Stage 4 — Zero the arm and measure real joint limits

Still no torque. Park the arm in its **sit-down pose** first.

```bash
./lerobot_alohamini/.venv/bin/python tools/rs_probe.py --port can0 --zero
./lerobot_alohamini/.venv/bin/python tools/rs_probe.py --port can0 --ranges
```

`--zero` sends `set_zero_position` to all seven. `--ranges` wraps
`record_ranges_of_motion`: you hand-move every joint to both stops while it
records the extremes.

This is how the real limits get into the code, and they are **not** the MuJoCo
model's:

| joint | `rs_arm.xml` (sim) | lerobot config (real) |
|---|---|---|
| shoulder_pan | −160.4 .. 160.4 | −150 .. 150 |
| shoulder_lift | −11.5 .. 179.9 | −200 .. 1 |
| elbow_flex | −11.5 .. 179.9 | −200 .. 1 |
| wrist_flex | −90.0 .. 90.0 | −80 .. 90 |
| wrist_yaw | −90.0 .. 90.0 | −90 .. 90 |
| wrist_roll | −179.9 .. 179.9 | −90 .. 90 |
| gripper | 0 .. 100 (normalised) | −270 .. 0 (**degrees**) |

`shoulder_lift` and `elbow_flex` are almost entirely positive in the model and
almost entirely negative on the arm: different zero conventions, ~180° apart.
Getting this wrong sends the arm to the far end of its travel on the first
command.

**Gate 4:** measured ranges agree with lerobot's config to within a few degrees
*after* zeroing from the sit-down pose. They go into
`RealB601Backend.model_joint_limits()` as measured values.

`JointMapper.from_ranges` reads the follower's limits from the live backend
rather than a constant, so **the offsets re-derive themselves** once this
returns real numbers. The directions do not — see stage 6.

---

## Stage 5 — First torque, one joint, bounded

```bash
./lerobot_alohamini/.venv/bin/python tools/rs_move_one.py --port can0 \
    --joint wrist_yaw --amplitude 5 --kp 8 --kd 1 --max-step-deg 1
```

Start with a **wrist** joint, not a shoulder: RS00, 14 N·m, and if the scale
factor is still somehow wrong the consequence is small. Hold the DC switch.

What the tool does: enables one motor, commands small bounded steps around the
present position, logs `target / measured / error / torque / temp` per step,
and disables torque on exit or on any fault.

Gains: `kp` is encoded over 0..500, `kd` over **0..5**. lerobot's
`RebotB601FollowerConfig` suggests `mit_kd = 12.0` for the shoulder joints —
that is the **Damiao** path, and the Robstride encoder would silently clamp it
to 5. Do not copy DM gains across. Start at the low end and climb.

**Gate 5:** a ±5° command tracks with a stable error, no oscillation, no fault,
temperature flat. Then repeat per joint, wrists before shoulders, with the arm
supported so nothing falls if torque cuts.

**Abort if:** the joint oscillates (kp too high / kd clamped), or the measured
motion is ~9× or ~1/9× the command — go back to stage 3.

---

## Stage 6 — Verify every direction on hardware

The signs in `DEFAULT_DIRECTIONS` were measured against the **MJCF's joint
axes**, which are not the motors' axes. They do not transfer. Every one gets
re-measured, exactly as it was in sim:

```bash
./lerobot_alohamini/.venv/bin/python tools/teleop_sim.py --backend real \
    --port can0 --joints wrist_yaw --max-step-deg 1 --max-speed-dps 15
```

One joint at a time, leader live, follower bounded to a crawl. Note whether the
follower follows or opposes, then record it. `--flip` toggles against the
verified-in-sim defaults, so the real set wants its own table:
`REAL_DIRECTIONS` in `ah/mapping.py`, separate from `DEFAULT_DIRECTIONS`.

**Gate 6:** all six arm joints move the same way as the leader, each confirmed
in isolation.

---

## Stage 7 — Teleop

Same loop, same `Session`, same mapping, different backend:

```bash
./lerobot_alohamini/.venv/bin/python tools/teleop_sim.py --backend real \
    --port can0 --hz 30 --max-step-deg 2 --max-speed-dps 60
```

Build up: `--joints` one at a time → wrists → shoulders → all six → add
`--gripper` last (its units differ, −270..0° rather than 0-100).

Everything the sim teleop already provides carries over unchanged, because
`Session` only ever talks to `AHArm`:

- soft engage cross-fade, so re-engaging does not jerk
- SPACE pause / E re-engage / R record / Q quit
- stale-leader detection — the leader's USB adapter has re-enumerated three
  times, and on the real arm a stale read must **hold**, not continue
- CSV recording of `leader / target / actual` per joint plus loop timing

Caps to carry from the start, and why: `max_step_deg` and `max_speed_dps` are
the only thing standing between a mapping bug and a 48 V arm at full torque,
*and* they are what keeps the LRS-600 out of hiccup mode. Raise them slowly.

**Gate 7:** the arm tracks the leader through a slow full-range motion on all
six joints, with the recorded tracking error plotted and no PSU trip.

---

## Still open

- **`stop()` semantics.** The base class holds position. On a 48 V arm with a
  2.5 kg payload, "hold" and "torque off" are very different outcomes and the
  real backend must choose explicitly rather than inherit.
- **`max_relative_target`** — set it on the follower too. Defence in depth: the
  AH layer already caps per-step motion, but a bug above it should not be able
  to reach the motors unbounded.
- **Control mode mismatch.** `MuJoCoBackend` emulates POS_VEL; the Robstride
  bus gives us MIT. The sim's cascade (position error → desired velocity →
  torque, with `qfrc_bias` feedforward) is closer to MIT than to a position
  interface, but the gains do not transfer and gravity feedforward has no
  equivalent on the real arm — the motors' own MIT loop is all there is.
- **E-stop.** There is currently no hardware kill beyond the DC switch. Decide
  whether that is acceptable before raising the speed caps.
