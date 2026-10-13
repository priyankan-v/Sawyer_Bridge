#!/usr/bin/env bash
# Development/test helper. Installed users source system Humble + package overlay.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:?Usage: env.sh ros2|bridge|ros1|system-ros2 command [args...]}
shift
prefix=${SAWYER_PACKAGE_PREFIX:-$ROOT/.runtime/pkg-prefix}
export SAWYER_PACKAGE_PREFIX="$prefix"
export SAWYER_ROOT="$ROOT"
if [[ $# == 0 && -f "$ROOT/config/network.env" ]]; then source "$ROOT/config/network.env"; fi
if [[ ${SAWYER_DEV_INTERNAL:-} != 1 ]]; then
  exec env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH -u CMAKE_PREFIX_PATH \
    -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u ROS_PACKAGE_PATH -u ROS_DISTRO \
    -u ROS_VERSION -u ROS_PYTHON_VERSION -u PKG_CONFIG_PATH -u CONDA_PREFIX \
    -u CC -u CXX -u CFLAGS -u CXXFLAGS -u LDFLAGS -u CPATH -u LIBRARY_PATH \
    PATH=/usr/bin:/bin PYTHONNOUSERSITE=1 SAWYER_DEV_INTERNAL=1 \
    bash --noprofile --norc "$0" "$mode" "$@"
fi
unset SAWYER_DEV_INTERNAL
export ROS_HOME="$ROOT/.runtime/ros"
export ROS_LOG_DIR="$ROS_HOME/log"
mkdir -p "$ROS_LOG_DIR"
case "$mode" in
  ros2|bridge|system-ros2)
    set +u
    source /opt/ros/humble/setup.bash
    if [[ "$mode" != system-ros2 ]]; then source "$prefix/local_setup.bash"; fi
    set -u
    if [[ "$mode" == bridge ]]; then
      export PKG_CONFIG_PATH="$prefix/libexec/sawyer/ros1/lib/pkgconfig"
      export CC=/usr/bin/cc CXX=/usr/bin/c++
    fi
    ;;
  ros1)
    # Private client dependencies for helper/test subprocesses; no ROS distro sourcing.
    private="$prefix/libexec/sawyer/ros1"
    export PYTHONPATH="$private/lib/python3/dist-packages"
    export LD_LIBRARY_PATH="$private/lib"
    export ROS_PACKAGE_PATH="$private/share"
    export ROS_VERSION=1 ROS_DISTRO=noetic ROS_PYTHON_VERSION=3
    export ROS_ETC_DIR="$private/etc/ros"
    if [[ ${1:-} == roscore || ${1:-} == rostopic ]]; then
      tool=$1
      shift
      exec /usr/bin/python3 "$ROOT/tests/private_ros1_tool.py" "$tool" "$@"
    fi
    ;;
  *) echo "Unknown mode: $mode" >&2; exit 2;;
esac
if [[ ${1:-} == python || ${1:-} == python3 ]]; then shift; exec /usr/bin/python3 "$@"; fi
if [[ $# == 0 ]]; then
  export PS1="[sawyer $mode] \\u@\\h:\\w\\$ "
  exec bash --noprofile --norc -i
fi
exec "$@"
