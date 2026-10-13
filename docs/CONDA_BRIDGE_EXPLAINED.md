# How the packaged Sawyer bridge works

This document describes `conda-pkg`. The earlier two-environment implementation
is preserved on `conda`.

## Installed architecture

```text
Existing /opt/ros/humble
  ├─ ROS 2 CLI, rclpy, rclcpp, middleware and standard interfaces
  └─ Sawyer package overlay
       ├─ Intera ROS 2 interfaces compiled for Python 3.10/system Humble
       ├─ move_joint, guarded command and camera_viewer applications
       ├─ selective C++ parameter_bridge
       └─ libexec/sawyer/ros1: private ROS 1 clients/interfaces
                                      ↕ TCPROS
                             Sawyer's onboard ROS 1 controller
```

Conda distributes the Sawyer-specific overlay and private client dependencies.
It does not install Humble or Noetic Conda environments. A normal bridge process
links ROS 2 libraries from system Humble and ROS 1 libraries bundled privately
inside the package. Required ROS 1 code is present; there is no separate user
Noetic installation or shell activation.

## Initialization

```bash
source /opt/ros/humble/setup.bash
source "$CONDA_PREFIX/local_setup.bash"
```

The first command initializes existing Humble. The second makes the installed
Sawyer ROS 2 packages discoverable. Private ROS 1 Python paths are never added to
this ROS 2 shell. Application entry points use system `/usr/bin/python3` 3.10.

Robot addresses and DDS domain are ordinary environment variables, set separately
as shown in README. The bridge connects to the master already running onboard
Sawyer; normal operation does not start a local master.

## Build

`packaging/build_package.py` verifies pinned ROS 1 archives and builds selected
client/interface dependencies with Ubuntu's compiler and Boost. It builds
`intera_core_msgs`, `intera_motion_msgs` and `sawyer_ros2_control` against system
Humble, then compiles the existing `ros1_bridge` source against both interfaces.
Python/EmPy generate C++ converters during the build. Only the selected core and
Intera packages are considered. Required custom definitions remain unchanged.

`packaging/relocate.py` sets relative ELF library paths. The Conda recipe packages
that staging tree and tests imports and CLI entry points. ROS 2 dependencies are
external prerequisites, so this artifact targets Ubuntu 22.04 x86_64 specifically.
The full maintainer procedure is in [PACKAGING.md](../PACKAGING.md).

## Launch and command path

`ros2 launch sawyer_ros2_control bridge.launch.py` locates installed files through
the ament package index. It runs configuration in a private ROS 1 Python subprocess,
loads selected topic settings into Sawyer's ROS 1 parameter server, then starts
the compiled bridge. Commands, cameras and head commands remain opt-in.

`move_joint` is a system-Humble Python node. Its ROS 2 JointCommand is converted
by the bridge into a ROS 1 JointCommand for Sawyer's `/realtime_loop`. JointState
feedback takes the reverse path. See [command details](MOVE_JOINT_COMMAND_PATH.md).

Preflight and explicit camera-control requests use isolated helper subprocesses.
`runtime.py` replaces repository-relative script/environment paths with installed
package paths. Helpers use `/usr/bin/python3` plus private ROS 1 modules while
preserving robot network settings. Camera images return through the bridge.

## Validation and safety

`ros2 run sawyer_ros2_control sawyer_runtime --check` checks platform and imports.
`ros2 run sawyer_ros2_control verify_bridge` explicitly starts a private localhost
fixture to test conversions and saves a transport report. Neither actuates Sawyer.

State/report files belong to the user, under `~/.local/state/sawyer_ros2` or
`SAWYER_STATE_DIR`, rather than inside the installed package. The guarded command
still requires operator-approved execution, passing transport validation, a fresh
preflight and live safety checks. Packaging makes no change to joint limits or
onboard safety settings. The existing unresolved passive-head issue is independent
of package installation.
