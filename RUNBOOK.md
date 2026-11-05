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

### Fix: a ~60-line shim teleoperator

Wrap `SOLeader` with the `joint_ranges` / `joint_directions` mapping that
`rebot_102_leader.get_action()` already implements (`rebot_102_leader.py:180-194`),
and register it as a new type. The AM-ARM200's small 1:1 joint sweep then maps
onto the B601's much larger 767 mm envelope.

### Safety before the first cross-vendor run

`RebotB601FollowerConfig.max_relative_target` defaults to `None` — no cap on
per-step jumps. On a 24 V CNC arm with a 1.5 kg payload, a bad mapping becomes a
fast swing. Set it for the first runs:

```bash
--robot.max_relative_target=5
```

Keep a hand on the power. The follower logs an emergency-stop hint if the leader
USB drops mid-session.
