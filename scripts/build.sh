#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
stage=${1:-all}
jobs=${SAWYER_BUILD_JOBS:-4}
export MAKEFLAGS="-j$jobs"
cd "$ROOT"
mkdir -p docs/results
if [[ $stage == ros1 || $stage == all ]]; then
  env -u PYTHONPATH -u PYTHONHOME PYTHONNOUSERSITE=1 /usr/bin/python3 scripts/prepare_interfaces.py
  scripts/env.sh ros1 bash -c 'cd "$SAWYER_ROOT/ros1_ws"; catkin_make -j"$1" -DPYTHON_EXECUTABLE="$SAWYER_ROOT/scripts/python_noetic.sh" -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5' _ "$jobs" 2>&1 | tee docs/results/build_ros1.log
fi
if [[ $stage == ros2 || $stage == all ]]; then
  CMAKE_BUILD_PARALLEL_LEVEL="$jobs" scripts/env.sh ros2 bash -c 'cd "$SAWYER_ROOT/ros2_ws"; colcon build --symlink-install --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" -DCMAKE_POLICY_VERSION_MINIMUM=3.5' 2>&1 | tee docs/results/build_ros2.log
fi
if [[ $stage == bridge || $stage == all ]]; then
  scripts/apply_bridge_patch.sh
  SAWYER_BUILD=1 CMAKE_BUILD_PARALLEL_LEVEL="$jobs" scripts/env.sh bridge bash -c 'cd "$SAWYER_ROOT/bridge_ws"; colcon --log-base log_sawyer build --build-base build_sawyer --install-base install_sawyer --symlink-install --packages-select ros1_bridge --cmake-force-configure --cmake-args -DBUILD_TESTING=OFF -DCMAKE_C_COMPILER="$CC" -DCMAKE_CXX_COMPILER="$CXX" -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" -Dxmlrpcpp_DIR="$xmlrpcpp_DIR" -DCMAKE_POLICY_VERSION_MINIMUM=3.5' 2>&1 | tee docs/results/build_bridge.log
fi
[[ $stage == all || $stage == ros1 || $stage == ros2 || $stage == bridge ]] || { echo "Unknown stage: $stage" >&2; exit 2; }
