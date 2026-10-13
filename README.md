# Sawyer package for system ROS 2 Humble

The `conda-pkg` branch builds **`sawyer-ros2-bridge`**, a Linux Conda package for
Sawyer on **Ubuntu 22.04 x86_64 with an existing `/opt/ros/humble` installation**.
It installs Intera interfaces, the selective C++ `ros1_bridge`, joint-control
applications and a camera viewer. Required ROS 1 clients live privately inside
the package; users do not create or source Noetic/Humble Conda environments.

```text
System Humble applications ↔ packaged C++ bridge ↔ Sawyer's onboard ROS 1
```

The working two-environment implementation remains on `conda`; the original
implementation remains on `main`. This package is platform-specific because its
C++ libraries and Python extensions use Ubuntu's Humble/Python 3.10 ABI.

## Install on a user's machine

Prerequisites: Ubuntu 22.04 x86_64, system Humble, Conda, Python 3.10, NumPy, Tk,
PyYAML, rospkg, and Ubuntu Boost 1.74/console_bridge libraries. The package does
not replace system ROS or Python. Confirm the Python helpers are available:

```bash
/usr/bin/python3 -c 'import numpy, tkinter, yaml, rospkg'
```

If a Python dependency is absent, install the corresponding Ubuntu package
(`python3-numpy`, `python3-tk`, `python3-yaml`, `python3-rospkg`). Boost/system
runtime requirements are documented in [PACKAGING.md](PACKAGING.md).

For a locally supplied artifact:

```bash
# Run in your existing Conda installation/prefix; no new ROS environment is needed.
conda install --offline /path/to/sawyer-ros2-bridge-0.2.0-humble_py310_1.conda
source /opt/ros/humble/setup.bash
source "$CONDA_PREFIX/local_setup.bash"
ros2 run sawyer_ros2_control sawyer_runtime --check
ros2 run sawyer_ros2_control verify_bridge
```

The self-test connects exclusively to its own localhost graph. It writes transport
validation under `~/.local/state/sawyer_ros2/`. Override the location with
`SAWYER_STATE_DIR`. Source Humble and the package overlay in each new terminal.
Package executables use `/usr/bin/python3`, regardless of Conda's Python version.
Do not add private ROS 1 Python paths to your ROS 2 shell.

The package has not been uploaded to a public channel. Maintainers can build and
index a local channel using [PACKAGING.md](PACKAGING.md); publication is separate.

## Connect to Sawyer

In both terminals, set the robot's existing network addresses. The values below
are this project's lab configuration; a new user must substitute their own robot
and workstation addresses:

```bash
export ROS_MASTER_URI=http://169.254.121.3:11311
export ROS_IP=169.254.121.2
unset ROS_HOSTNAME
export ROS_DOMAIN_ID=42
```

Keep robot networking separate from Conda dependency configuration. The package
never changes onboard software, network settings, motor enable state or homing.

## Terminal 1: telemetry bridge

After sourcing Humble and the package overlay and setting networking:

```bash
ros2 run sawyer_ros2_control preflight
ros2 launch sawyer_ros2_control bridge.launch.py
```

Only configured telemetry is forwarded. Run one operational bridge per robot and
DDS domain; stop any previous bridge before switching implementations.

For existing camera images:

```bash
# Stop the previous bridge with Ctrl-C before starting another.
ros2 launch sawyer_ros2_control bridge.launch.py cameras:=true
```

Arm command transport is a separate, explicit opt-in: `commands:=true`.
Head command transport uses `head_commands:=true`. These enable forwarding;
they do not send a motion command or enable the robot.

## Terminal 2: feedback and cameras

```bash
source /opt/ros/humble/setup.bash
source "$CONDA_PREFIX/local_setup.bash"
# Apply the same ROS_MASTER_URI, ROS_IP and ROS_DOMAIN_ID as Terminal 1.
ros2 topic echo /robot/joint_states sensor_msgs/msg/JointState --once --qos-reliability best_effort
ros2 run sawyer_ros2_control command read
ros2 run sawyer_ros2_control camera_viewer
```

The viewer opens a resizable desktop window with two rows. Use `--layout columns`
for side-by-side views, `--camera head` or `--camera hand` to select a stream,
`--start-both` to request both, and `--stop-stream all` to stop acquisition.
Default viewing is read-only. Close with Q, Esc, window close or Ctrl-C; use
`--stop-on-exit` with a start/selection command to also stop that stream.
See [camera guide](docs/CAMERAS.md) for details and measured specifications.

## Prepare joint-position control

Before physical motion, run a readiness check and review its errors:

```bash
mkdir -p "${SAWYER_STATE_DIR:-$HOME/.local/state/sawyer_ros2}"
ros2 run sawyer_ros2_control preflight --require-ready \
  --output "${SAWYER_STATE_DIR:-$HOME/.local/state/sawyer_ros2}/preflight.json"
ros2 run sawyer_ros2_control move_joint --joint right_j5 --position 0.0 --time 5 --dry-run
```

`move_joint` publishes if `--dry-run` is omitted. Its checks concern the selected
joint; it does not run the full-arm preflight or require an approval flag.
Physical execution remains an operator decision. The separate guarded application
requires explicit `--execute --operator-approved`, successful local transport
validation, a preflight less than 30 seconds old and live safety checks.

Do not proceed on `ready: false` or a limit/collision rejection. The head's last
physical test remained passive despite receiving commands; packaging does not
resolve that controller-side issue. No movement is part of package installation,
checks or build tests.

Press Ctrl-C to stop an application and then the bridge. Stopping publishing does
not itself disable motors or issue an emergency stop.

## Maintainer build and validation

See [PACKAGING.md](PACKAGING.md) for build dependencies, pinned source downloads,
Conda artifact creation, relocation checks and recorded test results.

```bash
/usr/bin/python3 packaging/build_package.py --prefix .runtime/pkg-prefix
# Development wrapper uses system Humble + the staged package, never old Conda ROS environments.
./scripts/env.sh ros2 ros2 run sawyer_ros2_control sawyer_runtime --check
```

## Files

| Path | Purpose |
|---|---|
| `packaging/` | Pinned private ROS 1 sources, native builder, relocation and Conda recipe |
| `bridge_ws/ros1_bridge/` | Existing bridge source with Sawyer interface selection |
| `intera_common/` | Original ROS 1 Intera definitions |
| `ros2_ws/src/intera_*_msgs/` | Compatible ROS 2 interfaces |
| `ros2_ws/src/sawyer_ros2_control/` | ROS 2 applications, launch and installed-file discovery |
| `scripts/move_joint.py` | Canonical joint-motion source, linked into the ROS 2 package |
| `scripts/{preflight,camera_control,configure_bridge}.py` | Helpers installed with isolated private ROS 1 dependencies |
| `scripts/env.sh` | Developer/test convenience only; not shipped or needed by users |
| `tests/` | Unit, native conversion and localhost integration tests |

[Package architecture](docs/CONDA_BRIDGE_EXPLAINED.md) ·
[Joint command path](docs/MOVE_JOINT_COMMAND_PATH.md)
