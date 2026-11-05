# AM-ARM200 + reBot B601 — LeRobot runbook

Target: AM-ARM200 as **leader**, Seeed reBot B601 as **follower**.

Both arms are supported by the *same* repo — `liyiteng/lerobot_alohamini`. You do
not need two forks. See the gap section at the bottom before trusting the
cross-vendor pair.

## Where this stands (2026-11-05)

The leader arm is **finished**. Assembled, all 7 servo ids flashed and verified
by motion, calibrated, and the calibration proven to survive a power cycle.

| Thing | State |
|---|---|
| 7× Feetech STS3215 | ids 1–7, model 777, 1 Mbaud uniform, 60/60 reply rate each |
| Joint→id mapping | verified by motion |
| Calibration (`--teleop.id=am_leader`) | ✅ **done — do not run it again** |
| Follower | not chosen yet — this is the only blocker left |

**Do not re-run `lerobot-calibrate` on the leader.** See §2.

Next thing to do: pick the follower, then first teleop run (§3).

## Layout

| Thing | Path |
|---|---|
| uv | `~/.local/uvenv/bin/uv` |
| Repo | `~/Claude/RobotArm/lerobot_alohamini` (v0.6.1, HEAD `ec65318f`) |
| venv | `~/Claude/RobotArm/lerobot_alohamini/.venv` (CPython 3.12.16) |
| Install script | `~/Claude/RobotArm/install.sh` |
| Leader calibration | `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/am_leader.json` |

Activate: `source ~/Claude/RobotArm/lerobot_alohamini/.venv/bin/activate`

This directory **is** the git repo `inkreate-hub/am2rebot` (public). The nested
`lerobot_alohamini/` clone is gitignored, not vendored — `setup.sh` re-clones it
at the pinned commit and re-applies `patches/`. Rebuild from scratch with
`./setup.sh`.

### Helper scripts

All take `--port /dev/ttyACM0`. Run them with the venv python.

| Script | What | Writes? |
|---|---|---|
| `check_leader_standalone.py` | Scan all baudrates, list ids + model + position. `--watch` to wiggle joints live. pyserial only, runs without lerobot. | no — PING/READ only |
| `calib_check.py` | Diffs the calibration JSON against servo EEPROM, per joint. The fast "is it still good?" check. | no |
| `verify_mapping.py` | Prompts you to move each joint, logs which id actually moved. ⚠️ half-fixed, see below. | no |
| `set_one_id.py` | Flashes ONE named joint's id. **One servo on the bus at a time.** | **yes — EEPROM** |
| `calibrate_leader.py` | The calibration sweep. Only needed if a servo is replaced or re-IDed. | **yes — EEPROM + JSON** |

⚠️ `verify_mapping.py` **false-positives — don't trust its MISMATCH output.** Its
detection loop still takes the *first* id over a flat 60-tick threshold. Moving
one joint jostles its neighbours, so a coupled joint can win the race. A real
swap is symmetric (A reports B *and* B reports A); a lone asymmetric mismatch is
coupling. The fix is half-written: `noise_floor()` and a circular `delta()`
exist in the file but are not wired in — switch the loop to argmax-with-dominance
against per-joint floors. Mapping on this arm is already verified, so this only
matters if you rebuild.

## ⚠️ Two joints bind: `shoulder_pan` and `wrist_yaw`

Established 2026-11-05 with `nudge.py`, which commands a small move while
logging load, voltage, current and status. The arm drives fine — but not on
every joint.

**`wrist_flex` (id 4) drives correctly**, tracking 2° steps to within 0.6°, at
**2–6% load**:

| target | actual | load |
|---|---|---|
| 81.69 | 81.45 | -24 |
| 85.69 | 85.49 | -24 |
| 89.69 | 89.10 | -56 |

**`shoulder_pan` (id 1) does not move at all** — and it is the *easiest* joint
on the arm, vertical axis with no gravity load and ~115° of headroom each way:

| commanded | actual | load | volts |
|---|---|---|---|
| -10.48 | -12.48 | -100 | 11.6 |
| -6.48 | -12.48 | -284 | 11.6 |
| -2.48 | -12.48 | **-464** | 11.6 |

46% commanded torque, zero motion, supply rock-steady at 11.6 V with no sag.
`wrist_yaw` behaves the same way and trips overload protection — a position
error it can never close.

So: not a supply problem, not a dead drive path, not a misconfigured servo
(`Overload_Torque`, `Protection_Time` and `Torque_Limit` are identical between
a working and a stuck joint). **Those two joints have genuine mechanical
resistance.** Worth checking for a snagged cable, overtightened fastener or a
printed part rubbing.

I initially concluded from `shoulder_pan` alone that the whole arm had no drive
authority. That was wrong — `wrist_flex` disproved it. Don't generalise from
one joint on this arm.

Nothing is damaged: every joint reports `Status=0` and 26–32 °C.

**Consequences:**

- The deliverable is unaffected either way. A leader is backdriven by hand and
  only its encoders are read, which is why `read_joints.py` sustains 98.8 Hz.
- Criterion 1 ("freely movable, no mechanical interference or excessive
  friction") needs those two joints checked by hand before sign-off.

## Measured step response and backlash (`wrist_flex`, driven)

`measure_steps.py --joint wrist_flex`, 5 reps per step:

| step | achieved | error | stddev |
|---|---|---|---|
| 1° | 0.369° | -0.631 | 0.294 |
| 2° | 1.160° | -0.840 | 0.039 |
| 5° | 4.097° | -0.903 | 0.048 |
| **backlash** | | | **0.791° (9.0 ticks)** |

Every step lands ~0.85° short, and backlash is 0.79° — the slack eats the move.
Confirmed independently: the `nudge.py` run gives 0.79/0.79/0.80° from the
same-target up-versus-down readings.

**Separate the two questions**, because the criteria only ask the first:

- **Can the sensor resolve 1°?** Yes, decisively. 1° = 11.4 ticks against a
  measured 0-tick noise floor. 2° and 5° are trivial.
- **Can the actuator deliver 1°?** No — backlash is 0.79°, comparable to the
  step itself. Irrelevant for a leader arm, where the human provides motion.

Once past the slack, repeatability is **0.04°** — about half an encoder tick.

Quote **0.79° backlash** as the real repeatability figure, not the 0.000° noise
floor.

## Machine constraints found

| Item | Value | Consequence |
|---|---|---|
| System Python | 3.14.4 | Too new; repo needs `>=3.12`. uv provisions 3.12.16. |
| GPU | Intel Iris Xe only | CPU torch. **Training must happen off this box.** Teleop + recording are fine. |
| ffmpeg | 8.0.1 (system) | OK — torchcodec ≥0.8.1 supports ffmpeg 8, so conda's pinned 7.1.1 is not needed. |
| `dialout` group | already a member ✅ | `/dev/ttyACM*` / `/dev/ttyUSB*` accessible without extra setup. |

## ⚠️ Local patch to the repo

`src/lerobot/motors/feetech/feetech.py` — `_assert_same_firmware()` downgraded
from `raise RuntimeError` to `logger.warning`.

The AM-ARM200 leader shipped with **mixed STS3215 firmware**: ids 1,5,6,7 on
3.9, ids 2,3,4 on 3.10. Upstream refuses to connect a mixed bus at all.

Safe here because the only firmware-gated entry in the Feetech control tables is
`Hts` (`tables.py:97`, valid for firmware >= 2.54) and both versions clear it —
every register this driver touches is identical across the bus. Verified: all 7
motors handshake and return plausible tick values.

Saved as `patches/0001-relax-feetech-firmware-guard.patch` and re-applied by
`setup.sh`, so re-cloning upstream does not lose it.

Revert with:
```bash
git -C ~/Claude/RobotArm/lerobot_alohamini checkout -- src/lerobot/motors/feetech/feetech.py
```

Proper fix if a joint ever reads offset/inverted/jumpy: flash all 7 to a common
version with Feetech's FD software (Windows only — Ivan has dual-boot).

## Device types in this fork

| Type | Role | Transport |
|---|---|---|
| `so101_leader` + `--arm_profile=am-leader-6dof` | AM-ARM200 leader | Feetech STS, `/dev/ttyACM*` |
| `so101_follower` + `--arm_profile=am-follower-6dof` | AM-ARM200 follower | Feetech STS, `/dev/ttyACM*` |
| `rebot_b601_follower` | reBot B601 follower | Damiao CAN via `motorbridge` |
| `rebot_102_leader` | StarArm 102 / reBot Arm 102 leader | FashionStar UART servos |

Joint names are identical across all four — `shoulder_pan, shoulder_lift,
elbow_flex, wrist_flex, wrist_yaw, wrist_roll, gripper`.

## 1. Find ports

```bash
lerobot-find-port
ls /dev/ttyACM* /dev/ttyUSB*
```

The leader has come up as `/dev/ttyACM0` every time so far, but the number
depends on plug order once the follower is attached — check, don't assume.
Calibration is keyed by `--teleop.id`, **not** by port, so a renumbered port does
not invalidate it.

If the leader's port gets held hostage: `sudo apt remove brltty`.

B601 note: with `can_adapter=damiao` (default) `--robot.port` is the Damiao
*serial bridge* device, e.g. `/dev/ttyACM0`, at 921600 baud — **not** a `can0`
SocketCAN name. For a PCAN/slcan adapter instead use
`--robot.can_adapter=socketcan --robot.port=can0`.

## 2. Calibrate

### AM-ARM200 leader — ALREADY DONE, skip this

Done 2026-11-05 under id `am_leader`. **Don't repeat it.** Calibration is stored
in two places and lerobot skips the sweep only when both agree:

1. the JSON at
   `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/am_leader.json`
2. servo EEPROM — `Homing_Offset`, `Min_Position_Limit`, `Max_Position_Limit`

Both verified identical after cutting servo power and replugging USB, so it is
genuinely persistent. `connect()` loads the file and `is_calibrated` returns
True, so teleop and record start silently with no prompt.

Confirm in ~5 s any time:
```bash
cd ~/Claude/RobotArm && ./lerobot_alohamini/.venv/bin/python calib_check.py
```
Expect `REUSABLE: file and servo EEPROM agree on all 7 joints.` The reference
values, for comparison if something ever drifts:

| joint | homing_offset | range_min | range_max |
|---|---|---|---|
| shoulder_pan | 874 | 873 | 3597 |
| shoulder_lift | -1680 | 1002 | 3355 |
| elbow_flex | -676 | 1881 | 3998 |
| wrist_flex | -972 | 1045 | 3146 |
| wrist_yaw | -979 | 1202 | 2947 |
| wrist_roll | 730 | 0 | 4095 |
| gripper | -1409 | 1739 | 2914 |

`wrist_roll` is the `full_turn_motor` — its range is forced to 0/4095 and it is
excluded from the sweep. That is correct, not a missed joint.

Re-calibrate **only** if you replace a servo, re-flash an id, or `calib_check.py`
reports `DIFFERS`. Then use the wrapper, not `lerobot-calibrate` — same thing
plus a belt-and-braces firmware bypass:
```bash
cd ~/Claude/RobotArm && ./lerobot_alohamini/.venv/bin/python calibrate_leader.py
```

Two things to know before ever re-running it:

- **Repeatable ≠ reusable.** A second sweep produces *different* offsets and the
  old ones were not wrong. Homing offset is measured from wherever the arm is
  parked when you centre it. Don't diff two calibration runs and read the delta
  as drift.
- Re-IDing a servo does **not** clear its old limits. Donor calibration was
  still live in EEPROM on this arm — `shoulder_pan` was clamped to 651–3121 —
  so an uncalibrated salvaged arm silently loses travel at both ends.

### reBot B601 follower

Neither reBot arm stores persistent calibration — it re-zeroes against whatever
pose it is physically holding each connect. Move the arm to its zero pose
(sit-down, gripper closed) before pressing ENTER. Unlike the leader, this one
**must** be redone every session.

```bash
lerobot-calibrate \
  --robot.type=rebot_b601_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=b601_follower \
  --robot.can_adapter=damiao
```

Power-cycle the follower afterwards. The leader does not care — proven.

## 3. Teleoperate

**Native, works out of the box** — AM-ARM200 leader → AM-ARM200 follower:
```bash
lerobot-teleoperate \
  --teleop.type=so101_leader --teleop.port=/dev/ttyACM0 \
  --teleop.id=am_leader --teleop.arm_profile=am-leader-6dof \
  --robot.type=so101_follower --robot.port=/dev/ttyACM1 \
  --robot.id=am_follower --robot.arm_profile=am-follower-6dof
```

**Native** — StarArm 102 leader → B601 follower:
```bash
lerobot-teleoperate \
  --robot.type=rebot_b601_follower --robot.port=/dev/ttyACM0 \
  --robot.id=b601_follower --robot.can_adapter=damiao \
  --teleop.type=rebot_102_leader --teleop.port=/dev/ttyUSB0 \
  --teleop.id=leader
```

## ⚠️ The gap: AM-ARM200 leader → B601 follower

This pair **connects and runs but will not track correctly as-is.** Three things
line up and two do not.

Lines up:
- Joint names match key-for-key.
- Units match. `so_leader.use_degrees` defaults to `True`, and the B601 follower
  treats incoming `.pos` as degrees (`math.radians(position_deg)` at
  `rebot_b601_follower.py:289`).
- The B601 follower explicitly tolerates foreign leaders — missing `wrist_yaw`
  is held at 0.0 rather than raising (`rebot_b601_follower.py:268`).

Does not line up — the AM-ARM200 leader has **no output-range mapping**.
`SOLeaderConfig` exposes only `port`, `use_degrees`, `arm_profile`. The native
`rebot_102_leader` additionally carries `joint_ranges` and `joint_directions`,
commented *"Matches the reBot B601 follower joint limits so leader actions can
drive the follower key-for-key."* Without that mapping:

| Joint | AM leader emits | B601 limit | Result |
|---|---|---|---|
| `shoulder_lift` | roughly ±100° | `(-200, 1)` | positive half clips to `1` — arm parks at its limit |
| `elbow_flex` | roughly ±100° | `(-200, 1)` | same |
| `gripper` | `0..100` (`RANGE_0_100`) | `(-270, 0)` | always clips to `0` — **gripper never opens** |

Per-joint direction may also be inverted; the B601's own leader needs
`{"shoulder_pan":-1,"shoulder_lift":-1,"elbow_flex":1,"wrist_flex":1,"wrist_yaw":1,"wrist_roll":-1,"gripper":-6}`.

### Measured leader ranges (2026-11-05, `id=am_leader`)

From `find_leader_ranges.py`. Every joint near-symmetric about zero, which is
what a correct calibration produces — **this is a healthy arm, not a fault.**

| joint | min | max | span |
|---|---|---|---|
| shoulder_pan | -118.1 | 119.7 | 237.8 |
| shoulder_lift | -102.6 | 103.6 | 206.2 |
| elbow_flex | -92.3 | 92.9 | 185.1 |
| wrist_flex | -92.4 | 92.5 | 185.0 |
| wrist_yaw | -78.5 | 80.0 | 158.5 |
| wrist_roll | -170.1 | 172.3 | 342.3 |
| gripper | 0.1 | 99.0 | 98.9 |

`wrist_roll` spans ~342° because it is the `full_turn_motor` — expected.
`gripper` is a normalised 0–100 scale, not degrees.

Note `lerobot-find-joint-limits` is **not** the tool for this — it demands a
connected follower *and* a URDF because it measures end-effector workspace
bounds. `find_leader_ranges.py` does the leader-only sweep instead, read-only.

### Fix: the mapping, in `am2rebot_mapping.py`

The mismatch turned out to be mostly an **offset, not a stretch**. The leader
zeroes at mid-travel; the B601 zeroes at its sit-down pose and unfolds one way
only, hence its negative-only `(-200, 1)` limits. Shift by −100° and the two
arms line up at true 1:1 degrees:

| joint | direction | scale | offset | result | clipped |
|---|---|---|---|---|---|
| shoulder_pan | +1 | 1.0 | 0 | -118.1 … 119.7 | 0% |
| shoulder_lift | +1 | 1.0 | **-100** | -202.6 … 3.6 | 2.5% |
| elbow_flex | +1 | 1.0 | **-100** | -192.3 … -7.1 | 0% |
| wrist_flex | +1 | 0.86 | 0 | -79.5 … 79.5 | 0% |
| wrist_yaw | +1 | 1.0 | 0 | -78.5 … 80.0 | 0% |
| wrist_roll | +1 | **0.5** | 0 | -85.0 … 86.2 | 0% |
| gripper | **-1** | **2.727** | 0 | -270.0 … -0.3 | 0% |

2.5% lost on one joint, at the far extreme of `shoulder_lift` travel. Preview it
any time:
```bash
./lerobot_alohamini/.venv/bin/python am2rebot_mapping.py
```

1:1 was chosen over a full-span stretch deliberately — stretching would use more
of the B601's 767 mm envelope but makes teleop feel amplified. Degrees-to-degrees
means the follower mirrors your hand.

⚠️ **Directions are unverified.** Sign is the one thing range data cannot
reveal — a joint reading +90 on the leader may need -90 on the follower
depending on assembly. Check each joint on hardware at low speed before
trusting the table. The gripper also assumes **leader 0 == closed**; if it opens
backwards, flip to `direction=1, offset=-270`.

### Safety before the first cross-vendor run

`RebotB601FollowerConfig.max_relative_target` defaults to `None` — no cap on
per-step jumps. On a 24 V CNC arm with a 1.5 kg payload, a bad mapping becomes a
fast swing. Set it for the first runs:

```bash
--robot.max_relative_target=5
```

Keep a hand on the power. The follower logs an emergency-stop hint if the leader
USB drops mid-session.
