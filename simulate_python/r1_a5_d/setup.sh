#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"

# Build a private DDS installation unless the caller supplies an existing one.
if [[ -z "${CYCLONEDDS_HOME:-}" ]]; then
    R1_DDS_SOURCE="$R1_MUJOCO_ROOT/.deps/cyclonedds"
    if [[ ! -d "$R1_DDS_SOURCE/.git" ]]; then
        git clone --branch 0.10.2 --depth 1 https://github.com/eclipse-cyclonedds/cyclonedds.git "$R1_DDS_SOURCE"
    fi
    test "$(git -C "$R1_DDS_SOURCE" rev-parse HEAD)" = 9995905bce6c4cf9f740d6438bbf7fcfd1c83dfd
    cmake -S "$R1_DDS_SOURCE" -B "$R1_MUJOCO_ROOT/.deps/build/cyclonedds" \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$R1_MUJOCO_ROOT/.deps/install/cyclonedds" \
        -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DBUILD_IDLC=OFF -DBUILD_DDSPERF=OFF \
        -DENABLE_SECURITY=OFF -DENABLE_SSL=OFF
    cmake --build "$R1_MUJOCO_ROOT/.deps/build/cyclonedds" --parallel 2
    cmake --install "$R1_MUJOCO_ROOT/.deps/build/cyclonedds"
    export CYCLONEDDS_HOME="$R1_MUJOCO_ROOT/.deps/install/cyclonedds"
fi

R1_SDK_SOURCE="${UNITREE_SDK2PY_SRC:-$R1_MUJOCO_ROOT/.deps/unitree_sdk2_python}"
if [[ ! -d "$R1_SDK_SOURCE" ]]; then
    git clone https://github.com/unitreerobotics/unitree_sdk2_python.git "$R1_SDK_SOURCE"
    git -C "$R1_SDK_SOURCE" checkout --detach 814556d15970dd2ecf1c9984e845ca02ab07e206
fi
test "$(git -C "$R1_SDK_SOURCE" rev-parse HEAD)" = 814556d15970dd2ecf1c9984e845ca02ab07e206

python3 -m venv "$R1_MUJOCO_ROOT/.venv-r1-a5-d"
R1_MUJOCO_PYTHON="$R1_MUJOCO_ROOT/.venv-r1-a5-d/bin/python"
"$R1_MUJOCO_PYTHON" -m pip install -r "$R1_MUJOCO_DIR/requirements.txt"
# The upstream CRC helper loads bundled .so files from its source directory.
"$R1_MUJOCO_PYTHON" -m pip install --no-deps -e "$R1_SDK_SOURCE"
"$R1_MUJOCO_PYTHON" "$R1_MUJOCO_DIR/build_model.py"
