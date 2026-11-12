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

## Six joints under power — 2026-11-12

±5°, 1° steps, 20 °/s, 90 °/s², POS_VEL profile mode. All passed.

| joint | worst err | peak torque | temp | notes |
|---|---|---|---|---|
| `shoulder_pan` | 0.08° | 0.67 N·m | 29 flat | vertical axis, unloaded |
| `wrist_roll` | 0.13° | 0.52 | 30 flat | roll axis, no gravity moment |
| `wrist_yaw` | 0.21° | 1.10 | 29 flat | |
| `shoulder_lift` | 0.22° | 3.00 | 29 flat | `--direction up` |
| `wrist_flex` | 0.47° | 2.55 | 29→31 | gravity-loaded wrist |
| `elbow_flex` | 0.47° | **6.18** | 29→30 | `--direction up` |
| `gripper` | — | — | — | disassembled, not tested |

Every ratio ≈1.00. **No scale error on an RS06 under load** — PMAX 12.57 rad
is confirmed in motion, not just in a decoded frame.

Gravity holding torque is ≈2.5 N·m on `shoulder_lift` and ≈4 N·m on
`elbow_flex`, against 36 N·m rated. The shoulders are barely working. A
`--max-torque 10` ceiling on an RS06 leaves ample headroom; real demand peaked
at 62% of it.

### `elbow_flex` was never damaged

It broke away from its stop at **6.18 N·m**. The morning of the same day it
hit **58.7 N·m** — 9.5× higher — being commanded −5° from a pose with 1.4° of
negative travel. That number was the hard stop, not the joint.

### Lost motion is mechanical, 0.2–0.5°

Three independent signatures, all repeatable:

1. **Breakaway.** First step off a stop loses motion and spikes torque:
   `elbow_flex` ratio 0.53 at 6.18 N·m, `shoulder_lift` 0.81 at 2.87. Both
   recover on the very next step (1.38, 1.16).
2. **Incomplete return.** The last step down to home stops ~0.2° high *every
   time* — `shoulder_lift` +0.20/+0.22, `elbow_flex` +0.18/+0.18, four of four.
3. **Gravity reversal.** `wrist_flex` descending: at the step where torque
   flips +0.69 → −2.03 N·m, it moves 0.73 of 1.00 commanded and errs 0.45°,
   then over-travels 1.34 recovering. Ascending, the same at −1.15°
   (ratio 0.75 → 1.30). The two crossings are **2.0° apart** — that is the
   hysteresis band. `wrist_yaw` shows it weaker at ≈−10.5°.

The joints with no gravity moment — `wrist_roll`, `shoulder_pan` — show none
of this and sit at 0.08–0.13° throughout.

**No gain setting removes any of this**, and in POS_VEL mode `kp` does not
reach the wire anyway. 0.2–0.5° is the accuracy floor for teleop on the loaded
joints; budget for it rather than tuning against it.

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

## Record and replay — 2026-11-12

`traj_20261112_141427.json`, 228 frames over 11.4 s, hand-moved then played
back through all six joints at once. **It ran clean — no guard abort, no stall,
nothing fell.** Per-frame tracking numbers were not captured off the terminal,
so treat this as a pass, not as a measurement.

Two things it establishes that nothing before it did: six joints can be driven
simultaneously off the LRS-600-48 without tripping it, and the measured travel
in `b601_ranges.json` is good enough to clamp a real path against.

The clamping it reported is benign and will recur. `shoulder_lift` rests at
0.51° against a measured min of −0.35°, so the 3° endpoint margin lifts it to
2.65° — 144 frames "clamped" is the *rest pose* touching the bottom margin, not
the 73° peak, which sits well inside 209° of travel.

## Teleop — two blockers found before anything moved, 2026-11-12

Both were found by reading, not by driving the arm into them.

### 1. lerobot's B601 follower is the wrong motor driver

`RebotB601Follower.connect` calls `bus.add_damiao_motor(...)` with a
`MOTOR_MODELS` table of Damiao part numbers — `4340P` shoulders, `4310` wrists.
This arm is RobStride: it needs `add_robstride_motor` and `rs-06` / `rs-00`.

`teleop_am_to_b601.py` is built on that follower, so **it cannot drive this
arm at all.** Not a tuning problem, a driver problem. The RS capability lives
one layer down in motorbridge, which is what `mb_bringup.py` has been using all
along.

### 2. `am2rebot_mapping.py`'s limits are sign-flipped on both shoulders

| joint | `B601_LIMITS` | measured here |
|---|---|---|
| `shoulder_lift` | −200 .. 1 | **−0.35 .. +208.79** |
| `elbow_flex` | −200 .. 1 | **−0.18 .. +241.79** |

Same span, opposite sign. Those limits are lerobot's
`RebotB601FollowerConfig.joint_limits`, which describe the **DM** arm; this is
the RS arm, zeroed against a different pose. Fed the leader's mid-pose, that
table asks for roughly −100° on both shoulders — into the stop they already
rest on, for every frame of the session.

`tools/mb_teleop.py` reads `calibration/b601_ranges.json` instead.

### Alignment must be by pose, not by centre

The textbook fit maps mid-leader-travel to mid-follower-travel. On this arm the
centres are `shoulder_lift` +104° and `elbow_flex` +121°, because it parks at
the bottom of its range. Measured against the arm's actual parked pose:

| alignment | move required just to engage |
|---|---|
| centre | pan 72.8°, **lift 103.2°, elbow 121.3°** |
| pose | pan 0.00°, lift 2.14°, elbow 3.02° |

So `--align pose` is the default: offsets are computed at engage time from
where both arms are standing, and the first commanded target equals the
follower's current position. The 2.14/3.02 are the 3° endpoint margin lifting
the two parked shoulders off their stop — the same effect replay shows.

**The cost is that the start pose fixes the reachable range.** Engaging from
the parked pose clips 51% of the leader's travel on `shoulder_lift` and 52% on
`elbow_flex`, because the whole lower half of the leader's swing maps below the
follower's floor. Park the arm half-raised and match the leader to it to get
travel in both directions.

### Directions are still unverified on hardware

`DEFAULT_DIRECTIONS` in `ah/mapping.py` was established by driving the **sim**
from the live leader. The physical arm has since been re-zeroed and measured.
A sign that was right in the model is not thereby right here — verify one joint
at a time with `--joints`, flip with `--invert`.

**Resolved 2026-11-12 evening.** All six were verified by hand, one at a time,
and are recorded in `calibration/teleop_directions.json`. Four of the six
needed the sign the sim gave; two did not. The signs that stuck:

| joint | direction | | joint | direction |
|---|---|---|---|---|
| `shoulder_pan` | `+1` | | `wrist_flex` | `−1` |
| `shoulder_lift` | `+1` | | `wrist_yaw` | `−1` |
| `elbow_flex` | `−1` | | `wrist_roll` | `+1` |

## The guard's torque ceiling was below the arm's own weight — 2026-11-12

**`shoulder_lift` aborted a teleop run at 18.28 N·m while it was not being
driven.** It was in the held set, target unchanged, tracking error `+0.00`. The
operator had extended `elbow_flex` to 143.7°, which swung the forearm and wrist
mass out from the base and raised the shoulder's gravity moment until standing
still cost 18.28 N·m against a ceiling of 18.0. It tripped by **0.28 N·m, 1.5%
over**. Nothing was obstructed.

The abort message said *"it is driving into something — a stop, the bench, or
another link"*. All three were false. The arm was holding itself up.

Then the abort returned into the `finally`, which disabled all six motors at
once, and the extended arm fell. `elbow_flex` went 143.7° → −0.11°.

### Two lessons, and they are independent

**A torque ceiling set from the datasheet is not a measurement.** `rated × 0.5`
was chosen for a bounded few-degree bring-up move on an unloaded arm. Teleop
breaks that premise: the payload is the arm's own outstretched mass. Real
numbers from that run — `elbow_flex` peaked at **17.78 N·m**, 0.22 from its own
trip. Both shoulders were effectively at the limit just existing in that pose.

**A stall and a gravity hold are identical on the torque channel.** They differ
everywhere else, and this is the whole basis of the current guard:

| signal | stall (morning, 58.7 N·m) | gravity hold (evening, 18.28 N·m) |
|---|---|---|
| torque | high and rising | high and steady |
| tracking error | 5.3° and growing | 0.00° |
| velocity | ~0 against a command | tracking normally |

There are now three ceilings per joint. `stall` is half of rated and **fixed** —
crossing it with a large error and no motion aborts. `warn` is raised by
`--gravity-map` measurements and only suppresses a notice. `hard` is 85% of
rated and aborts unconditionally. The stall threshold is deliberately not raised
by the gravity map: a gravity load appears with *zero* error by definition, so
knowing what the arm can hold says nothing about what is acceptable while
failing to reach a target. An earlier draft collapsed the two and left a mapped
shoulder blind to a real stall between 18 and 26 N·m.

### Aborting by cutting torque is worse than the condition

On a folded arm, releasing is harmless. At 143° of extension it converts a
warning into a fall. Every abort path now goes through `_settle_and_release`,
which **re-targets every joint to its measured position first** — that removes a
stall's commanded error, and with it the stall, without removing the torque that
holds the arm up — then streams that for `--hold-on-fault` seconds before
releasing.

Guards do not re-run during that hold. They have just fired; re-running them
would abort the abort handler on its first pass and release instantly, which is
the exact behaviour it exists to prevent.

### The window was the wrong shape — freeze, don't count down

The first version held for 5 s and then released. On the next run that window
expired while the arm was fully extended and **it fell anyway**. Five seconds is
not enough to read a warning and get a hand under 10 kg, and no fixed number is
— a clock knows nothing about the arm, so picking a bigger one moves the fall
rather than preventing it.

**Heat is the right thing to key on, because heat is the only reason holding is
not free.** A motor standing still under load dissipates that load continuously:
40 °C holding 15.34 N·m in the run that fell, 62 °C in the morning stall.

`--hold-on-fault` now defaults to **−1**: hold indefinitely, ending on whichever
comes first —

| ends on | behaviour |
|---|---|
| `--hold-max-temp` (50 °C) | announced 5° out, so the end is visible coming |
| Ctrl-C | operator has the weight |
| nothing | a cool arm holds all day |

A positive value restores the fixed window; `0` releases at once.

**Ctrl-C during a normal run freezes too.** That path used to fall straight
through to `disable()` — the same instant drop, reached by the one route the
operator takes every single time. A second Ctrl-C releases.

**Still not a controlled descent.** When the temperature limit arrives, the arm
goes limp wherever it is. Lowering an extended arm to the bench under power
remains unbuilt.

### Measure the envelope instead of arguing about it

```bash
python tools/mb_bringup.py --gravity-map --joint elbow_flex --to-deg 150
```

Walks one joint in `--max-step-deg` increments, dwells `--settle` at each, and
records peak |torque| on **all seven** — because the joint being moved is
usually not the one that ends up loaded. Writes
`calibration/gravity_torque.json`, keeping the highest peak seen per joint
across runs: one sweep visits one family of poses, and a ceiling that drops when
you measure more is worse than no ceiling.

Not yet run on this arm. Until it is, the ceilings are still the datasheet
guess — only now the guess no longer aborts on gravity alone.

## No payload, just extended — why a single ceiling can never work, 2026-11-12

The operator's question after the fall was the right one: *why was it about to
drop when there was no payload and the arm was only extended?* Because there
**is** a payload and it is the arm. At `lift 136.7 / elbow 143.7` the shoulder
measured **19.28 N·m** holding nothing but its own forearm, wrist and gripper
at full lever arm. The rs-06 is rated **36 N·m**. The motor was at 54% of rated
and in no difficulty whatever — the *software* was.

An unloaded arm at full extension is not a corner case. It is the most loaded
static pose the machine will ever see in normal use, and teleop reaches it
constantly.

### Mis-set in one of two ways, and no good value between them

Gravity torque at the shoulder is a function of pose. Over this arm's travel it
runs from near zero (links stacked over the axis) to its maximum (extended,
horizontal). A single number must sit either above that maximum or below it:

| ceiling | extended poses | folded poses |
|---|---|---|
| above the max | never false-aborts | **blind** — 17 N·m into the bench sits under an 18 N·m ceiling and says nothing |
| below the max | **false-aborts** on every reach — this is what dropped the arm | catches it |

Raising the number trades one failure for the other. This is structural, not a
tuning problem, and it is why the number was wrong twice in one day in opposite
directions.

### Ask what the pose explains, not whether the torque is large

Gravity load is *predictable from the joint angles*. An obstruction, a collision
or a payload is not. So the guard now compares measured torque against predicted
torque and watches the **residual** — the part gravity does not account for.

For a planar serial chain the gravity torque at joint *k* sums over every link
outboard of it:

```
tau_k = SUM_{j >= k} [ a_j*cos(THETA_j) + b_j*sin(THETA_j) ] + c
```

`THETA_j` is the **cumulative** angle base-to-link-*j*, because it is a link's
world attitude that gravity cares about. The `sin` terms carry each joint's
arbitrary zero phase — this arm has no persistent calibration and is zeroed
against whatever pose it is parked in, so nothing starts from horizontal. `c`
absorbs steady sensor bias.

That is **linear in its coefficients**: ordinary least squares via normal
equations, no dynamics library and no new dependency. `shoulder_lift` gets 7
free parameters, `elbow_flex` 5, `wrist_flex` 3. `shoulder_pan` is excluded on
purpose — it rotates about a vertical axis, so gravity exerts no moment about it
and a model would be fitting noise.

### Samples come from ordinary use, not another autonomous sweep

The obvious way to get data is to command a 2D grid over (lift, elbow). That
means the arm driving itself through its most heavily loaded poses to learn what
they cost, which is the one thing the day's history argues against.

Teleop already visits those poses, under a hand, with the operator choosing
where it goes. So samples are logged during normal sessions — only when every
chain joint is **near-stationary** (a moving link adds inertial torque the model
has no term for) and **tracking** (a frame taken mid-stall would teach the model
that an obstruction is normal for that pose, which is exactly what it exists to
notice). The fit runs offline, moves nothing, and improves every session.

```bash
python tools/mb_bringup.py --fit-gravity   # reads the log, moves nothing
```

It refuses to write a model unless the data earns it, because the guard
*trusts* a fitted prediction and a bad one explains away real obstructions:

| gate | why |
|---|---|
| ≥150 samples | 7 free parameters fitted to 20 points is a confident-looking model of nothing |
| ≥40° of travel on every angle it depends on | an arm that only moved its elbow teaches nothing about the shoulder's dependence on lift |
| residual must beat 0.8 × the spread in the data | if predicting the mean does as well, there is no pose dependence here and the coefficients are noise |

### What this buys, in both directions

Verified against a synthetic arm whose true torques are known, with phases set
so the shoulder reads 19.28 N·m at the exact pose that dropped the real one
(`test_posemodel.py`, 22/22):

- **19.28 N·m extended, tracking → no abort.** Including with the 2.43° lag
  actually seen on hardware.
- **17 N·m in a light pose, 4° of error, not moving → ABORT.** The same case
  with `--no-gravity-model` returns clean, which demonstrates the blind spot
  rather than asserting it.
- **+6 N·m over what an extended pose explains → ABORT.** Extension does not buy
  immunity.
- The **hard ceiling (85% of rated) is untouched** and still fires regardless of
  tracking. A measurement claiming a joint needs that much to stand still is
  evidence something is wrong with the arm, not a licence to allow it.

The fitted model lives in `calibration/gravity_model.json` and is committed — it
is the measured identity of this particular arm. The raw sample log is not: it
is append-only, grows at the control rate, and is regenerable.

### Still missing

The freeze is a hold, not a descent. When `--hold-max-temp` arrives the arm goes
limp wherever it is. **Retracting the elbow collapses the shoulder's lever arm
and takes the gravity load toward zero** — the fitted model can say which
direction does that, which makes a genuine controlled park possible rather than
a longer wait before the same fall. Not built.

## Diagnostics that lie

**An ERROR-PASSIVE link reports a healthy bus as dead.** A scan returned 0/7
with `bus-errors 0`, `error-warn 4`, `error-pass 1`, 3 frames out. Zero bus
errors with the TX error counter driven to 128 is pure ACK failure, which
correctly rules out bitrate, host id, protocol and motor ids all at once —
acknowledgement happens in the data-link layer beneath all of them. But the
controller was already error-passive with a jammed TX queue, and the same motor
replied 3/3 after `ip link set can0 down && up`.

Bounce the link before trusting any silent scan.

## "elbow_flex did not work" — it was parked on its limit, 2026-11-12

A six-joint run came back as the elbow not responding while `shoulder_lift`
moved fine. The follower elbow was healthy: `--scan` gave 7/7, torque 0.00,
33 °C, no fault. The map was healthy too — 2% clip.

The arm was **parked at the bottom of the elbow's mapped travel**:

```
elbow_flex   dir -1   offset 92.9   leader -92.3..92.9  ->  follower 0.0..185.2
elbow_flex   now 0.04   first target 2.82
```

Direction `−1` means leader-up → follower-down, and the follower was already at
0.04 against an image bottoming at 0.0. Working back from the 2.82° first
target, the leader elbow was at ≈90.1° against its own max of 92.9 — the end
that maps to the follower's zero. **2.8° of room one way, 182° the other.** The
clamp was doing its job and it reads exactly like a dead joint.

Every number needed to see that was already printed, spread across two tables,
and none of it said so. Preflight now computes `room down` / `room up` per joint
and warns when one side is under `CORNERED_DEG` (5°) while the other is not. It
warns rather than blocks: a joint parked on its limit is not an error, it is
where the arm happens to be standing.
