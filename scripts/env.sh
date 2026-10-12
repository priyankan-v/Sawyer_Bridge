#!/usr/bin/env bash
# Execute in an isolated Conda ROS runtime. Never source this file.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:?Usage: env.sh ros1|ros2|bridge|system-ros2 command [args...]}
shift
# Local Conda prefixes keep environment and build paths isolated.
NOETIC_PREFIX=${SAWYER_NOETIC_PREFIX:-$ROOT/.conda/envs/noetic}
HUMBLE_PREFIX=${SAWYER_HUMBLE_PREFIX:-$ROOT/.conda/envs/humble}
if [[ ${SAWYER_ENV_INTERNAL:-} != "$mode" ]]; then
  # Interactive shells are the daily Sawyer interface. Load network settings
  # once; explicit one-shot commands keep the caller's settings (including tests).
  if [[ $# == 0 && -f "$ROOT/config/network.env" ]]; then
    source "$ROOT/config/network.env"
  fi
  export SAWYER_ROOT="$ROOT"
  clean=(env -u PYTHONPATH -u PYTHONHOME -u ROS_PACKAGE_PATH -u CMAKE_PREFIX_PATH
    -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u LD_LIBRARY_PATH -u PKG_CONFIG_PATH
    -u CONDA_PREFIX -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION
    -u CC -u CXX -u CFLAGS -u CXXFLAGS -u LDFLAGS -u CPATH -u LIBRARY_PATH -u BASH_ENV -u ENV
    -u CONDA_DEFAULT_ENV -u CONDA_SHLVL -u ROS_ETC_DIR -u ROS_ROOT -u ROS_OS_OVERRIDE
    PATH=/usr/local/bin:/usr/bin:/bin PYTHONNOUSERSITE=1 SAWYER_ENV_INTERNAL="$mode")
  if [[ $mode == system-ros2 ]]; then
    exec "${clean[@]}" bash --noprofile --norc "$0" "$mode" "$@"
  fi
  case $mode in ros1) prefix=$NOETIC_PREFIX;; ros2|bridge) prefix=$HUMBLE_PREFIX;; *) exit 2;; esac
  [[ -x "$prefix/bin/python" ]] || { echo "Missing Conda environment: $prefix (see README.md)" >&2; exit 1; }
  # setup.bash supplies ROS paths; compiler paths are selected below. No inherited
  # /opt/ros or another activated Conda environment can leak into this process.
  exec "${clean[@]}" CONDA_PREFIX="$prefix" PATH="$prefix/bin:/usr/local/bin:/usr/bin:/bin" \
    bash --noprofile --norc "$0" "$mode" "$@"
fi
# ROS setup scripts aren't compatible with nounset.
set +u
# Legacy catkin setup.sh forwards positional arguments to its parser. Keep CLI
# flags (especially --help) out of ROS initialization.
source_setup() {
  local setup_file=$1
  shift
  source "$setup_file"
}
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11311}
export ROS_HOME="$ROOT/.runtime/ros"
export ROS_LOG_DIR="$ROOT/.runtime/ros/log"
export ROS2CLI_DISABLE_DAEMON=1
mkdir -p "$ROS_LOG_DIR"
case $mode in
  ros1)
    source_setup "$NOETIC_PREFIX/setup.bash"
    [[ ! -f "$ROOT/ros1_ws/devel_conda/setup.bash" ]] || source_setup "$ROOT/ros1_ws/devel_conda/setup.bash"
    ;;
  ros2|bridge)
    source_setup "$HUMBLE_PREFIX/setup.bash"
    [[ ! -f "$ROOT/ros2_ws/install_conda/local_setup.bash" ]] || source_setup "$ROOT/ros2_ws/install_conda/local_setup.bash"
    if [[ $mode == bridge ]]; then
      ROS1_PREFIX="$NOETIC_PREFIX"
      export PATH="$CONDA_PREFIX/bin:$ROS1_PREFIX/bin:$PATH"
      export ROS_PACKAGE_PATH="$ROOT/ros1_ws/src:$ROS1_PREFIX/share"
      export PKG_CONFIG_PATH="$ROOT/ros1_ws/devel_conda/lib/pkgconfig:$ROS1_PREFIX/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
      export PYTHONPATH="$ROOT/ros1_ws/devel_conda/lib/python3.12/site-packages:$ROS1_PREFIX/lib/python3.12/site-packages:${PYTHONPATH:-}"
      export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$ROOT/ros1_ws/devel_conda/lib:$ROS1_PREFIX/lib:${LD_LIBRARY_PATH:-}"
      # Only ROS 1-specific CMake configs; don't place all Noetic ahead of Humble.
      export xmlrpcpp_DIR="$ROS1_PREFIX/share/xmlrpcpp/cmake"
      export CMAKE_PREFIX_PATH="$ROOT/ros2_ws/install_conda/intera_motion_msgs:$ROOT/ros2_ws/install_conda/intera_core_msgs:$CONDA_PREFIX:$ROS1_PREFIX"
      export CC="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-cc"
      export CXX="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-c++"
      if [[ -x "$ROOT/bridge_ws/install_conda/ros1_bridge/lib/ros1_bridge/parameter_bridge" && ${SAWYER_BUILD:-0} != 1 ]]; then
        source_setup "$ROOT/bridge_ws/install_conda/local_setup.bash"
      fi
    fi
    ;;
  system-ros2)
    source_setup /opt/ros/humble/setup.bash
    export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages
    export PATH=/opt/ros/humble/bin:/usr/bin:/bin
    ;;
esac
set -u
if [[ $mode != system-ros2 ]]; then
  # Pure Python EmPy 3 for legacy Noetic/Humble generators, kept project-local.
  export PYTHONPATH="$ROOT/tools/empy3:${PYTHONPATH:-}"
fi
unset SAWYER_ENV_INTERNAL
if [[ $# == 0 ]]; then
  export PS1="[sawyer $mode] \\u@\\h:\\w\\$ "
  exec bash --noprofile --norc -i
fi
exec "$@"
