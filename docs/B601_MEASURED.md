# B601-RS — measured, not assumed

Numbers this arm actually produced. Anything here beat an assumption that was
wrong; see `B601_BRINGUP.md` for the procedure that produced them.

## Bus

| | |
|---|---|
| Transport | classic CAN, 1 Mbit/s, `gs_usb` native SocketCAN (`1d50:606f`) |
| Host / master id | **`0xFD`** — not the Damiao-style `0x10 + id` |
| Motor ids | 1–7 as shipped, `shoulder_pan`…`gripper`. None ever needed changing |
| Feedback | only arrives after `robstride_set_active_report(True)` |
| PMAX | **12.57 rad** for `rs-06`, measured. lerobot's `O6` (112.5) is 8.95× wrong |

## Gate 5 — tracking, 2026-11-12

±5°, 1° steps, 20 °/s, POS_VEL profile mode, kp 8, from a freshly zeroed arm.

| joint | model | worst err | peak torque | temp | verdict |
|---|---|---|---|---|---|
| `shoulder_pan` | rs-06 | 0.07° * | — | 27 flat | PASS (2026-11-10, ±3°) |
| `shoulder_lift` | rs-06 | — | — | — | not yet moved |
| `elbow_flex` | rs-06 | — | — | — | not yet moved |
| `wrist_flex` | rs-00 | **0.35°** | **+2.3 / −3.9 N·m** | **26→28** | PASS, gravity-loaded |
| `wrist_yaw` | rs-00 | 0.13° | ±1.4 N·m | 25→26 | PASS |
| `wrist_roll` | rs-00 | 0.13° | ±0.6 N·m | 26 flat | PASS |
| `gripper` | rs-00 | 0.08° | ±0.4 N·m | 26 flat | PASS, 3 runs |

\* before the arm had a valid zero.

Step ratio averages 1.00 on every joint, which is the PMAX constant confirming
itself independently on both motor models.

### `wrist_flex` is the loaded one

Its torque is position-dependent — about +2.2 N·m holding positive angles,
−3.9 N·m at negative ones — which is gravity on the gripper, and it is the only
joint that heated. Its 0.35° sits at the direction reversals, where backlash
and gravity add. Do not compare its gain against the other three wrists.

### The residual is deterministic

Cycle 2 reproduces cycle 1 almost digit-for-digit (target −3.00 → −2.92 both
times, −5.00 → −4.99 both times). So the ~0.1° residual is directional
hysteresis, not scatter: approach a target from below and the joint trails low,
from above it trails high. Repeatable error is correctable error.

## Torque-off behaviour — three separate effects, do not confuse them

| effect | magnitude | when |
|---|---|---|
| encoder noise at rest | ≤0.03° | always; one tick of quantisation |
| elastic spring-back | ~2° | once, immediately on `disable()` |
| free-joint displacement | **up to 10°** | while *another* joint is driven |

1. **At rest the arm does not move.** Three consecutive scans give bit-identical
   positions on all seven joints; 120 s apart, worst 0.03° in both directions.
   There is no creep.

2. **Releasing torque springs the joint back.** `wrist_yaw` finished a run at
   −0.04° under torque and settled at −2.11° once disabled, then held there
   across three scans. Gear-train wind-up unwinding. A torque-off pose is only
   comparable to another torque-off pose.

3. **Free joints walk while the arm is driven.** Bracketing the four wrist runs
   with scans: bit-identical before, bit-identical after, but `shoulder_pan`
   displaced −9.93° and `wrist_yaw` −6.09°. They did not creep — they moved
   during the runs and then stopped dead.

   This is the 11° seen on 2026-11-10: same joint, same magnitude, same
   conditions. The mechanism is not separable from the bench — reaction torque
   walking a backdrivable joint fits, so does a hand on the arm between runs.

   **Consequence: teleop must hold torque on all seven joints.** The
   one-joint-at-a-time bring-up pattern leaves six free while the seventh is
   driven, and on this arm free joints do not stay put.

## Calibration

`set_zero_position()` is **RAM-only**. `mb_bringup.py` does not call
`store_parameters()`, so the zero dies on the next power cycle. Raw pre-zero
positions are snapshotted to `calibration/b601_prezero_*.txt` before every
zeroing run, because `set_zero_position()` otherwise destroys the only record
of the previous reference.

## Travel, measured by hand 2026-11-12

Torque off, joint moved by hand to both ends. Endpoints are where the joint
stopped feeling free, not the true mechanical stop. Lives in
`calibration/b601_ranges.json`; `--move` refuses any joint missing from it.

| joint | min° | max° | travel° |
|---|---|---|---|
| `shoulder_pan` | −61.26 | 209.60 | 270.87 |
| `shoulder_lift` | −0.35 | 208.79 | 209.14 |
| `elbow_flex` | −0.18 | 241.79 | 241.97 |
| `wrist_flex` | −97.21 | 45.44 | 142.65 |
| `wrist_yaw` | −116.66 | 97.17 | 213.82 |
| `wrist_roll` | −139.66 | 300.29 | 439.95 |
| `gripper` | — | — | **not recorded** |

These travels are larger than a 6-DoF arm's joints usually move, which looked
like bad data: `lo`/`hi` is a bare min/max with no outlier rejection, so one
corrupt frame widens a range permanently and nothing ever narrows it. It is
not bad data. `elbow_flex` was swept twice, ten minutes apart, and came back
241.97° against 243.66° — 0.7% apart, with the negative stop reproducing to
0.08°. The numbers are real.

`shoulder_lift` and `elbow_flex` both run from ≈0 to positive only. That is
the same stop `elbow_flex` found at 58.7 N·m on the morning of 2026-11-12,
from a parked pose that had 1.4° of negative travel left.

**The gripper is deliberately absent.** It swept 434.74° (−333.97 → +100.77)
while disassembled, so that figure is the motor turning freely, not the jaw.
Re-sweep it assembled before trusting any gripper target — a missing range
only ever costs a refusal, a wide wrong one costs hardware. Note that the
`note:` field in the JSON cannot carry this warning: `_write_ranges`
regenerates it from a constant on every sweep.

## Diagnostics that lie

**An ERROR-PASSIVE link reports a healthy bus as dead.** A scan returned 0/7
with `bus-errors 0`, `error-warn 4`, `error-pass 1`, 3 frames out. Zero bus
errors with the TX error counter driven to 128 is pure ACK failure, which
correctly rules out bitrate, host id, protocol and motor ids all at once —
acknowledgement happens in the data-link layer beneath all of them. But the
controller was already error-passive with a jammed TX queue, and the same motor
replied 3/3 after `ip link set can0 down && up`.

Bounce the link before trusting any silent scan.
