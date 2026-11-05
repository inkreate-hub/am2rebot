# am2rebot — AM-ARM200 leader arm joint reader

A calibrated physical **AM-ARM200 leader arm** and a plain-Python joint-position
reader for it. No ROS 2, no MuJoCo, no follower.

The reader streams the 6 arm joints plus the gripper with timestamps, and is
meant to be consumed directly by downstream work.

```bash
./lerobot_alohamini/.venv/bin/python read_joints.py            # live table
./lerobot_alohamini/.venv/bin/python read_joints.py --csv       # machine-readable
```

## Status against the acceptance criteria

| Criterion | State |
|---|---|
| Assembled, mounted, freely movable, no interference | see [Assembly](#assembly) |
| 7 Feetech servos detected, mapped to correct joint/gripper | ✅ ids 1–7, all model 777, 60/60 reply rate |
| Calibration repeatable and reusable after reconnect/restart | ✅ verified across a power cycle, [RUNBOOK §2](RUNBOOK.md) |
| Live timestamped values, 6 joints + gripper, no ROS 2 | ✅ `read_joints.py`, 98.8 Hz measured |
| One joint moves → its reading changes; sign/order documented | 🟡 order ✅ documented; **signs pending** `characterize_joints.py --signs` |
| 1°/2°/5° detectable; noise and backlash noted | 🟡 noise ✅ measured; **backlash pending** |
| Demo video | ⬜ to record |
| README: assembly, deps, connection, calibration, run, troubleshooting | ✅ this file |

Out of scope, deliberately: MuJoCo, follower control, forward kinematics,
Cartesian retargeting, data collection.

## Assembly

AM-ARM200, 6+1 DoF, 3D-printed, 7 × Feetech **STS3215** serial-bus servos daisy-
chained on one TTL half-duplex bus through a Waveshare bus-servo adapter
(`/dev/ttyACM*`, 1 Mbaud). Upstream CAD and build instructions:
[liyiteng/AM-ARM](https://github.com/liyiteng/AM-ARM).

Servo ids must be **1–7 in joint order** (see [Joint order](#joint-order)). Two
things bite during assembly:

- **Servos arrive on factory id 1.** Two devices answering id 1 collide and
  *both* go silent — it reads exactly like two dead cables. A healthy servo
  answers 60/60 pings; a collision reads ~19/60. Flash ids with **one servo on
  the bus at a time**, using `set_one_id.py` (not `lerobot-setup-motors`, which
  walks a fixed prompt order and mis-assigns if you restart it per servo).
- **Re-IDing does not clear old limits.** Salvaged servos keep the donor arm's
  calibration in EEPROM and silently lose travel. Calibrate after re-IDing.

Before trusting any reading, check the arm moves freely by hand through its
whole range with no binding, cable snag or grinding. Friction and slop show up
as a joint that does not return to the same value from both directions —
measure that rather than eyeball it (see [Characterise](#5-characterise)).

## Dependencies

| | |
|---|---|
| Python | **3.12** (upstream requires ≥3.12; system Python 3.14 is too new) |
| Driver stack | [`liyiteng/lerobot_alohamini`](https://github.com/liyiteng/lerobot_alohamini) v0.6.1, pinned at `ec65318f` |
| Extras | `feetech,rebot,dataset,viz` — not `[all]`, which drags in every VLA policy plus a `reachy2` grpcio/protobuf conflict |
| torch | CPU-only build (this machine has Intel Iris Xe) |
| Env manager | `uv` |

One command from a bare machine:

```bash
./setup.sh
```

That installs `uv`, clones upstream at the pinned commit, applies `patches/`,
provisions CPython 3.12, installs dependencies and restores the leader
calibration. Idempotent — re-running skips whatever already exists.

`read_joints.py --signs` and `--help` work without any of it.

## Connection

```bash
ls /dev/ttyACM*
lerobot-find-port          # if more than one device is attached
```

The arm has come up as `/dev/ttyACM0` every time here, but the number depends on
plug order — check, don't assume. Calibration is keyed by `--id`, **not** by
port, so a renumbered port does not invalidate it.

You need to be in the `dialout` group:

```bash
groups | grep -q dialout || sudo usermod -aG dialout "$USER"   # then log out/in
```

## Calibration

**Already done** for `--id am_leader`, and verified to survive a power cycle.
Confirm in about five seconds:

```bash
./lerobot_alohamini/.venv/bin/python calib_check.py
```

Expect `REUSABLE: file and servo EEPROM agree on all 7 joints.`

Re-calibrate **only** if you replace a servo, re-flash an id, or that check
reports `DIFFERS`:

```bash
./lerobot_alohamini/.venv/bin/python calibrate_leader.py
```

Two things to understand first, both covered in [RUNBOOK §2](RUNBOOK.md):

- **Repeatable ≠ reusable.** A second sweep produces *different* offsets and the
  old ones were not wrong — homing offset is measured from wherever the arm is
  parked when you centre it. Don't diff two runs and read the delta as drift.
- `wrist_roll` is the `full_turn_motor`; its range is forced to 0/4095 and it is
  excluded from the sweep. Correct, not a missed joint.

## Run

### 1. Sign convention and joint order

```bash
./lerobot_alohamini/.venv/bin/python read_joints.py --signs
```

### 2. Live table

```bash
./lerobot_alohamini/.venv/bin/python read_joints.py
```

Arm is limp — move it by hand. Degrees for the arm joints, normalised 0–100 for
the gripper.

### 3. CSV, for downstream consumers

```bash
./lerobot_alohamini/.venv/bin/python read_joints.py --csv -o run.csv
./lerobot_alohamini/.venv/bin/python read_joints.py --csv --fps 100 --seconds 30
```

Columns: `t_mono, t_unix, dt_ms,` then the 7 joints in fixed order. Add `--raw`
to append raw servo ticks.

Two timestamps on purpose — `t_mono` (`time.monotonic`) for intervals and rates
because it cannot jump backwards on an NTP step, `t_unix` (`time.time`) to
correlate with a video file or another process. `dt_ms` is the measured bus read
time, which is what actually caps the rate.

### 4. One sample

```bash
./lerobot_alohamini/.venv/bin/python read_joints.py --once
```

### 5. Characterise

```bash
./lerobot_alohamini/.venv/bin/python characterize_joints.py --noise
./lerobot_alohamini/.venv/bin/python characterize_joints.py --all -o reports/full.md
```

`--noise` is non-interactive. `--signs`, `--steps` and `--backlash` prompt you to
move the arm.

## Measured performance

| | |
|---|---|
| Sustained sample rate | **98.8 Hz** at `--fps 100` |
| Bus read time | **1.3–1.8 ms** per 7-joint sample |
| Encoder resolution | **0.0879 °/tick**, 4096 ticks/rev (12-bit absolute) |
| Static noise, 6 of 7 joints | **0.000° peak-to-peak** over 300 samples |
| Static noise, `wrist_roll` | 0.176° (2 ticks) |

Full report: [reports/noise-floor.md](reports/noise-floor.md).

The sensor is **quantisation-limited, not noise-limited** — so 1° (11.4 ticks)
carries roughly 11× margin over the smallest resolvable step, and 2°/5° are
trivial. The practical limit on this arm is therefore **mechanical backlash in
the gearbox and printed linkage, not the encoder.** Measure it with
`characterize_joints.py --backlash` and quote that number; do not infer
sub-degree repeatability from the noise figures above.

## Measured joint ranges

From `find_leader_ranges.py`, id `am_leader`. Near-symmetric about zero on every
joint, which is what a correct calibration produces.

| joint | id | min | max | span | units |
|---|---|---|---|---|---|
| shoulder_pan | 1 | -118.1 | 119.7 | 237.8 | deg |
| shoulder_lift | 2 | -102.6 | 103.6 | 206.2 | deg |
| elbow_flex | 3 | -92.3 | 92.9 | 185.1 | deg |
| wrist_flex | 4 | -92.4 | 92.5 | 185.0 | deg |
| wrist_yaw | 5 | -78.5 | 80.0 | 158.5 | deg |
| wrist_roll | 6 | -170.1 | 172.3 | 342.3 | deg |
| gripper | 7 | 0.1 | 99.0 | 98.9 | 0–100 |

`wrist_roll` spans ~342° because it is the full-turn motor — expected. Zero is
calibrated mid-travel, **not** a mechanical hard stop.

## Joint order

Fixed, and matches servo ids 1–7. Downstream code can rely on it.

```
shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw, wrist_roll, gripper
```

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| No `/dev/ttyACM*` | `brltty` grabs WCH `1a86` chips | `sudo apt remove brltty` |
| `Permission denied` on the port | not in `dialout` | `sudo usermod -aG dialout $USER`, log out/in |
| Fewer than 7 servos found | two servos share an id and collide | `check_leader_standalone.py` to scan; re-flash with `set_one_id.py`, one servo on the bus |
| Whole bus looks dead | header read byte-at-a-time overruns the `cdc_acm` latency | read the frame in one bulk call — already handled in `check_leader_standalone.py` |
| `RuntimeError: Some Motors use different firmware versions` | mixed STS3215 firmware | patched to a warning, see [RUNBOOK](RUNBOOK.md); `setup.sh` re-applies it |
| Prompted to calibrate on every start | JSON and servo EEPROM disagree | `calib_check.py` to see which joint, then `calibrate_leader.py` |
| A joint reads offset or inverted | donor calibration still in EEPROM after re-IDing | re-calibrate |
| Values never change | you are reading a static pose — the encoder is absolute and genuinely reports the same tick | move a joint; `wrist_roll` jitters ~2 ticks even at rest |
| `MISMATCH` from `verify_mapping.py` | false positive from mechanical coupling | known bug, see [RUNBOOK](RUNBOOK.md). A real swap is symmetric |

## Repo layout

| Path | What | Writes? |
|---|---|---|
| **`read_joints.py`** | **The deliverable.** Live/CSV timestamped reader | no |
| `characterize_joints.py` | Noise floor, signs, 1/2/5° steps, backlash | no |
| `find_leader_ranges.py` | Per-joint travel sweep | no |
| `calib_check.py` | Is the calibration still valid? | no |
| `check_leader_standalone.py` | Bus scan, pyserial only, no lerobot needed | no |
| `verify_mapping.py` | Which id is which joint ⚠️ false-positives | no |
| `set_one_id.py` | Flash one servo id | **EEPROM** |
| `calibrate_leader.py` | The calibration sweep | **EEPROM + JSON** |
| `RUNBOOK.md` | Working notes, every gotcha found | — |
| `reports/` | Measurement output | — |
| `setup.sh` / `install.sh` | Environment rebuild | — |
| `patches/` | Local changes to upstream | — |
| `calibration/` | Backup of the leader calibration JSON | — |
| `lerobot_alohamini/` | Upstream clone — **gitignored**, not vendored | — |

Also present but **out of scope** for this deliverable, from the follower work:
`am2rebot_mapping.py`, `teleop_am_to_b601.py`.
