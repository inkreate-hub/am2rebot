# am2rebot

Driving a **Seeed reBot Arm B601** follower from an **AM-ARM200** leader, through
LeRobot.

Both arms are already supported by one upstream repo —
[`liyiteng/lerobot_alohamini`](https://github.com/liyiteng/lerobot_alohamini) —
so this repo holds no robot driver code. What lives here is the part that would
otherwise be lost: the environment recipe, the bring-up scripts, the one local
patch upstream needs, and the hard-won notes in [RUNBOOK.md](RUNBOOK.md).

## Status

| | |
|---|---|
| AM-ARM200 leader | ✅ assembled, ids flashed, calibrated, calibration survives power-cycle |
| reBot B601 follower | ⬜ not connected yet |
| Cross-vendor mapping shim | ⬜ not written — the pair will connect but not track, see RUNBOOK §gap |

## Get going

```bash
./setup.sh                                    # clone upstream, patch, venv, deps
source lerobot_alohamini/.venv/bin/activate
```

Then read [RUNBOOK.md](RUNBOOK.md) — it is the actual working document. Start at
*Where this stands*.

## Layout

| Path | What |
|---|---|
| `RUNBOOK.md` | **Read this.** Bring-up, calibration, teleop commands, every gotcha found so far |
| `setup.sh` | Rebuild the tree from scratch |
| `install.sh` | Dependency install only (CPU torch, lean extras) |
| `patches/` | Local changes to upstream, applied by `setup.sh` |
| `calibration/` | Backup of the leader's calibration JSON |
| `check_leader_standalone.py` | Bus scan, no lerobot needed. Read-only |
| `calib_check.py` | Is the calibration still valid? Read-only |
| `verify_mapping.py` | Which id is which joint. ⚠️ false-positives, see RUNBOOK |
| `set_one_id.py` | Flash one servo id. **Writes EEPROM** |
| `calibrate_leader.py` | The calibration sweep. **Writes EEPROM** |
| `tools/` | Read-only probes used during bring-up |
| `lerobot_alohamini/` | Upstream clone — **gitignored**, not vendored |

Scripts that write EEPROM are marked. Everything else only pings and reads, and
never enables torque.

## This machine

Intel Iris Xe only, so torch is CPU-only here: teleop and recording are fine,
**training has to happen elsewhere.**
