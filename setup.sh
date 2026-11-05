#!/usr/bin/env bash
# Rebuild this working tree from scratch on a fresh machine.
#
#   1. clone the upstream LeRobot fork at the pinned commit
#   2. apply our local patch (see patches/)
#   3. provision CPython 3.12 + venv
#   4. hand off to install.sh for the dependency install
#
# Idempotent: re-running skips whatever already exists.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$HERE/lerobot_alohamini"
UPSTREAM=https://github.com/liyiteng/lerobot_alohamini.git
PINNED=ec65318f4cc3d830242f6e987d43f46e6b61f0a5   # v0.6.1
UV="${UV:-$HOME/.local/uvenv/bin/uv}"

# uv itself. Ubuntu 26.04 marks the system Python externally-managed (PEP 668),
# so uv goes in its own venv rather than via `pip install --user`.
if [ ! -x "$UV" ]; then
  echo "=== installing uv into $HOME/.local/uvenv"
  python3 -m venv "$HOME/.local/uvenv"
  "$HOME/.local/uvenv/bin/pip" install --quiet uv
fi

if [ ! -d "$REPO" ]; then
  echo "=== cloning upstream"
  git clone "$UPSTREAM" "$REPO"
  git -C "$REPO" checkout "$PINNED"
fi

# The firmware-guard relaxation. Without it the AM-ARM200 leader's mixed
# STS3215 firmware (3.9 / 3.10) refuses to connect at all. See RUNBOOK.md.
if git -C "$REPO" apply --check --reverse "$HERE/patches/0001-relax-feetech-firmware-guard.patch" 2>/dev/null; then
  echo "=== patch already applied"
else
  echo "=== applying patches/0001-relax-feetech-firmware-guard.patch"
  git -C "$REPO" apply "$HERE/patches/0001-relax-feetech-firmware-guard.patch"
fi

if [ ! -d "$REPO/.venv" ]; then
  echo "=== creating venv (CPython 3.12)"
  "$UV" python install 3.12
  "$UV" venv --python 3.12 "$REPO/.venv"
fi

echo "=== dependencies"
"$HERE/install.sh"

# Leader calibration is keyed by --teleop.id, not by port, and lerobot looks for
# it under ~/.cache. Restoring the backup here avoids a needless re-sweep --
# which would produce different (not wrong) offsets. RUNBOOK.md §2 explains why.
CALIB_DIR="$HOME/.cache/huggingface/lerobot/calibration/teleoperators/so_leader"
if [ -f "$HERE/calibration/am_leader.json" ] && [ ! -f "$CALIB_DIR/am_leader.json" ]; then
  echo "=== restoring leader calibration"
  mkdir -p "$CALIB_DIR"
  cp "$HERE/calibration/am_leader.json" "$CALIB_DIR/"
  echo "    NOTE: servo EEPROM must match this file. Verify with:"
  echo "    ./lerobot_alohamini/.venv/bin/python calib_check.py"
fi

echo
echo "Done. Activate with:  source $REPO/.venv/bin/activate"
