#!/usr/bin/env bash
# Shared environment for scripts launched from any working directory.
R1_MUJOCO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
R1_MUJOCO_ROOT="$(cd -- "$R1_MUJOCO_DIR/../.." && pwd)"
R1_MUJOCO_PYTHON="${R1_MUJOCO_PYTHON:-$R1_MUJOCO_ROOT/.venv-r1-a5-d/bin/python}"
if [[ -z "${CYCLONEDDS_HOME:-}" && -d "$R1_MUJOCO_ROOT/.deps/install/cyclonedds" ]]; then
    export CYCLONEDDS_HOME="$R1_MUJOCO_ROOT/.deps/install/cyclonedds"
fi
if [[ -n "${CYCLONEDDS_HOME:-}" ]]; then
    export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
