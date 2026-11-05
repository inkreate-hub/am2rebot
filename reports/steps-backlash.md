# AM-ARM200 leader characterisation

Generated 2026-11-05 14:55:45 from `characterize_joints.py`.
Port `/dev/ttyACM0`, calibration id `am_leader`, profile `am-leader-6dof`.

## Step detectability

| joint | nominal | measured + | measured - | verdict |
|---|---|---|---|---|

## Driven step response — `wrist_flex`

Measured 2026-11-05 15:27:46, start pose 80.308 deg, 5 reps per step.

| step | mean achieved | error | repeatability (stddev) | detectable |
|---|---|---|---|---|
| 1 deg | 0.369 | -0.631 | 0.2942 | NOT reliably detectable |
| 2 deg | 1.160 | -0.840 | 0.0393 | detectable |
| 5 deg | 4.097 | -0.903 | 0.0482 | detectable |

| approach | settled position |
|---|---|
| from below (−10 deg) | 0.484 |
| from above (+10 deg) | 1.275 |
| **backlash** | **0.791 deg** (9.0 ticks) |

