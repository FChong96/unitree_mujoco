#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
exec "$R1_MUJOCO_PYTHON" "$R1_MUJOCO_DIR/run.py" "$@"
