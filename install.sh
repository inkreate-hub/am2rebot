#!/usr/bin/env bash
# AM-ARM200 + reBot B601 — LeRobot environment install
# Step 4 "Software setup", sub-step 1 "Install environment".
#
# Already done for you (no need to repeat):
#   - uv 0.13.7      -> /home/inko/.local/uvenv/bin/uv
#   - CPython 3.12.16 provisioned by uv
#   - repo cloned    -> /home/inko/Claude/RobotArm/lerobot_alohamini
#   - venv created   -> /home/inko/Claude/RobotArm/lerobot_alohamini/.venv
#
# This script only does the dependency install, which the sandbox blocked.

set -euo pipefail

UV=/home/inko/.local/uvenv/bin/uv
REPO=/home/inko/Claude/RobotArm/lerobot_alohamini
VPY="$REPO/.venv/bin/python"

# Extras chosen for this build:
#   feetech -> AM-ARM200 leader + follower (STS3215/STS3095 serial bus servos)
#   rebot   -> reBot B601 follower (motorbridge/CAN) + reBot Arm 102 leader
#   dataset -> episode recording and video encode/decode
#   viz     -> rerun dataset visualisation
#
# Deliberately NOT installing the full [all] extra: it drags in every VLA policy
# (pi, groot, wallx, molmoact2, smolvla ...) plus reachy2, which pins
# grpcio<=1.73.1 / protobuf<=6.32.0 and is the documented resolver-conflict
# hazard in pyproject.toml. None of it is needed to teleoperate or record.
#
# CPU torch: this machine has Intel Iris Xe only, no CUDA and no ROCm, so the
# nvidia-* wheels (~2.7 GB: cudnn 627M, cublas 567M, nccl 283M, ...) are dead
# weight.
#
# --torch-backend=cpu ALONE IS NOT ENOUGH. pyproject.toml:367-374 declares
#   [tool.uv.sources] torch = [{ index = "pytorch-cu128", marker = "...linux" }]
# and an explicit sources pin beats --torch-backend, so uv still resolves the
# CUDA wheels. --no-sources-package surgically ignores that pin for torch and
# torchvision only, leaving every other sources entry intact.

"$UV" pip install \
  --python "$VPY" \
  --no-sources-package torch \
  --no-sources-package torchvision \
  --torch-backend=cpu \
  -e "$REPO[feetech,rebot,dataset,viz]"

echo
echo "=== install finished, verifying ==="
"$VPY" -c 'import lerobot, torch; print("lerobot", lerobot.__version__ if hasattr(lerobot,"__version__") else "ok"); print("torch", torch.__version__, "cuda:", torch.cuda.is_available())'
"$VPY" -c 'import motorbridge; print("motorbridge ok")'
"$VPY" -c 'import scservo_sdk; print("feetech sdk ok")'
echo
echo "Activate with:  source $REPO/.venv/bin/activate"
