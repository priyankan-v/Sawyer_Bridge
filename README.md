# Sawyer ROS 1–ROS 2 bridge

Connect Sawyer's ROS 1 controller to ROS 2 Humble through `ros1_bridge`.
Two Conda environments provide Noetic and Humble with Python 3.12.15.

`Sawyer (ROS 1) ↔ parameter_bridge ↔ ROS 2 joint-control application`

[Daily use](#daily-use) · [Camera video](#camera-video) · [Gripper](#gripper) · [Cartesian pick and place](#cartesian-pick-and-place) · [Position control](#position-control) ·
[First-time setup](#first-time-setup) · [Build](#build) ·
[Tests](#tests) · [Troubleshooting](#troubleshooting)

## Daily use

Complete [setup](#first-time-setup) and [build](#build) once. Use two terminals.
`env.sh` initializes isolated ROS paths; execute it, never source it.
Do not source `/opt/ros/humble/setup.bash` into these environments.

### Terminal 1: start the bridge

```bash
cd ~/sawyer_rs
conda activate "$PWD/.conda/envs/humble"
source config/network.env
./scripts/env.sh ros1 python scripts/preflight.py && \
./scripts/env.sh ros2 ros2 launch sawyer_ros2_control bridge.launch.py
#./scripts/env.sh ros2 ros2 launch sawyer_ros2_control bridge.launch.py commands:=true cameras:=true head_commands:=true gripper_commands:=true kinematics:=true

```

**This starts telemetry only. Arm command transport is disabled by default.**
Run one bridge per robot/domain.

### Terminal 2: check feedback

```bash
cd ~/sawyer_rs
./scripts/env.sh ros2
ros2 topic type /robot/joint_states
ros2 topic echo /robot/joint_states sensor_msgs/msg/JointState --once --qos-reliability best_effort
ros2 run sawyer_ros2_control command read
ros2 topic hz /robot/joint_states
#ros2 run sawyer_ros2_control move_joint --joint right_j5 --position 0.0 --time 5
```

Expect all seven arm joints (`right_j0`–`right_j6`), finite positions/velocities,
and roughly 100 Hz arm feedback. Press `Ctrl-C` to stop `hz`.

## Camera video

`camera_viewer` opens a desktop window. Default viewing is read-only; explicit
options select, start or stop camera streams. Both physical cameras streamed
concurrently in testing. See [Camera commands and specifications](docs/CAMERAS.md)
for two-node operation, display details, measured FPS and maximum-spec limitations.

If updating an existing installation, rebuild the application once:

```bash
./scripts/env.sh ros2
cd "$SAWYER_ROOT/ros2_ws"
colcon --log-base log_conda build --build-base build_conda --install-base install_conda \
  --symlink-install --packages-select sawyer_ros2_control
exit
```

**Terminal 1:** from the repository root, stop your existing bridge with `Ctrl-C`
and restart with image forwarding:

```bash
source config/network.env
./scripts/env.sh ros2 ros2 launch sawyer_ros2_control bridge.launch.py cameras:=true
```

**Terminal 2:** enter the ROS 2 shell, then choose a viewer command:

```bash
./scripts/env.sh ros2
ros2 run sawyer_ros2_control camera_viewer --camera head
# Or select hand: --camera hand
# Or explicitly start both: --start-both
# Or view existing streams without controlling them: no options
```

Alternatively, in that shell: `python -m sawyer_ros2_control.camera_viewer`.
Close with **Q**, **Esc**, the window close button or `Ctrl-C`. Closing leaves the
source running unless you add `--stop-on-exit` to a selection/start command.
Two cameras appear in **two rows** by default. Add `--layout columns` for side-by-side
panes; add `&` to free the terminal, e.g.:

```bash
ros2 run sawyer_ros2_control camera_viewer --start-both --layout columns &
```

Drag the window borders or maximize it; panes and images resize automatically
while preserving aspect ratio.

Switch an existing two-pane viewer, stop acquisition, or inspect settings:

```bash
ros2 run sawyer_ros2_control camera_viewer --camera hand --select-only
ros2 run sawyer_ros2_control camera_viewer --stop-stream all
ros2 run sawyer_ros2_control camera_viewer --stream-info
```

For terminal-only validation, use `--start-both --headless --duration 6 --require-both
--stop-on-exit`. The viewer uses existing Humble NumPy and Tk; no OpenCV/cv_bridge
installation is needed. A graphical desktop is required unless using `--headless`.

## Gripper

The ROS 2 `gripper` node controls the standard electric `right_gripper` on the
end effector using Intera I/O signals. Rebuild `sawyer_ros2_control` using the
application build command above after updating. The default bridge forwards
gripper feedback; restart it after updating the configuration.

In the ROS 2 shell, inspect the attached gripper or preview an opening command:

```bash
ros2 run sawyer_ros2_control gripper status
ros2 run sawyer_ros2_control gripper open --dry-run
```

To allow gripper actuation, start Terminal 1's bridge with:

```bash
./scripts/env.sh ros2 ros2 launch sawyer_ros2_control bridge.launch.py gripper_commands:=true
```

Then choose an operation in Terminal 2's ROS 2 shell:

```bash
ros2 run sawyer_ros2_control gripper calibrate  # Explicitly calibrate if needed; moves the fingers
ros2 run sawyer_ros2_control gripper open
ros2 run sawyer_ros2_control gripper close
ros2 run sawyer_ros2_control gripper position --position 0.02
ros2 run sawyer_ros2_control gripper stop
```

Position is in metres, from `0` (closed) to `0.041667` (open), matching the bundled
Intera electric gripper interface. Other end-effector hardware needs its own
device interface. `status` is the default operation and publishes no commands;
`--dry-run` checks feedback and prints the requested command without publishing.
The bridge retains the latest gripper state because Sawyer publishes it only
when it changes. Motion requires an available state publisher, a calibrated
gripper and no reported error.
Calibration is explicit and never runs automatically. The node retries the same
timestamped command until acknowledged and then waits for completion; a close
that reports an object grasp also succeeds. Timeouts and faults exit nonzero.
Position completion uses the device's reported `dead_zone_m` (currently `0.002`,
or 2 mm), with a 1 mm fallback if that signal is absent. Add `--tolerance 0.001`
to require a stricter 1 mm check; a device that stops outside that tolerance
will time out. Position results include the target, tolerance and measured error.
Use `--timeout 15` for a longer wait. `Ctrl-C` exits the client; an accepted
operation may continue on the device, so use `gripper stop` to halt it.

## Cartesian pick and place

Pose capture, IK/FK goal checks, a MoveIt planning model and guarded pick/place
execution are implemented and tested with a simulated controller. See the
[work plan and workflow](docs/PICK_AND_PLACE_PLAN.md) for model preparation,
planning, preview and explicit execution commands. Object/obstacle dimensions
are deferred for the initial demonstration, so table/fixture/carried-object
avoidance is not provided. The physical task has not been executed.

After rebuilding `sawyer_ros2_control`, restart the bridge with `kinematics:=true`
(alongside any existing options). In the ROS 2 application shell:

```bash
ros2 run sawyer_ros2_control cartesian read
# Teach each pose using the robot's normal operator procedure, then capture while stationary:
ros2 run sawyer_ros2_control cartesian capture --output "$SAWYER_ROOT/config/poses/pick.json"
ros2 run sawyer_ros2_control cartesian capture --output "$SAWYER_ROOT/config/poses/place.json"
ros2 run sawyer_ros2_control cartesian ik --pose "$SAWYER_ROOT/config/poses/pick.json"
```

The tool defaults to `right_gripper_tip`, positions use metres in `base`, and
orientation is a quaternion in `[x, y, z, w]` order. Captures preserve the tool
name and joint configuration and never overwrite an existing file. These
commands do not move the robot. `ik_valid` does not certify an obstacle-free path.

The captured wrist configuration currently lies about 0.00226 rad below its
lower limit. The planning tools report and reject that start; follow the robot
operator's normal procedure to resolve it before planning physical movements.

With a valid live plan, `ros2 run sawyer_ros2_control pick_place` previews the
stages; adding `--execute` runs them. The task checks readiness, collision and
tracking feedback, validates each retimed path with the current gripper opening,
and requires a detected grasp before transport. Failure handling does not open
the gripper automatically. See the workflow above before physical execution.

## Position control

See [How `move_joint` reaches Sawyer](docs/MOVE_JOINT_COMMAND_PATH.md) for the
execution path, source files, bridge conversion and feedback checks.

Keep Terminal 2's `[sawyer ros2]` shell open. Before motion, check readiness:

```bash
"$SAWYER_ROOT/scripts/env.sh" ros1 python "$SAWYER_ROOT/scripts/preflight.py" --require-ready
```

Proceed only with `"ready": true` and `"errors": []`. The last reported
`right_j6` position was below its live lower limit; resolve any limit rejection
through the robot operator's normal procedure.

### Prepare a small movement (no actuation)

```bash
ros2 run sawyer_ros2_control move_joint --joint right_j2 --distance 0.01 --time 5 --dry-run
```

`move_joint` publishes when `--dry-run` is omitted. Use the guarded `command`
application below for the first physical test; it also checks full-arm readiness,
collision state and feedback freshness.

Capture and review a target 0.01 rad above the current position:

```bash
ros2 run sawyer_ros2_control command read > "$SAWYER_ROOT/.runtime/current-joints.json"
TARGET=$(python -c 'import json,os; from pathlib import Path; p=Path(os.environ["SAWYER_ROOT"])/".runtime/current-joints.json"; print(json.loads(p.read_text())["right_j2"]+0.01)')
ros2 run sawyer_ros2_control command one --joint right_j2 --position "$TARGET" --duration 5
```

If the planner rejects the target or robot state, resolve the reported cause.
Run the [local tests](#tests) before the first physical test.

### Execute only after operator approval

**Terminal 1:** stop the telemetry bridge with `Ctrl-C`, then restart:

```bash
./scripts/env.sh ros2 ros2 launch sawyer_ros2_control bridge.launch.py commands:=true
```

**Terminal 2:** verify command transport:

```bash
ros2 topic info /robot/limb/right/joint_command --verbose
ros2 topic info /robot/limb/right/set_speed_ratio --verbose
```

Both must show a bridge subscription. Zero publishers is normal while idle.
With the reviewed `TARGET` still set, run a fresh preflight followed by execution:

```bash
"$SAWYER_ROOT/scripts/env.sh" ros1 python "$SAWYER_ROOT/scripts/preflight.py" \
  --require-ready --output "$SAWYER_ROOT/.runtime/preflight.json" && \
ros2 run sawyer_ros2_control command one --joint right_j2 --position "$TARGET" \
  --duration 5 --execute --operator-approved --report "$SAWYER_ROOT/.runtime/approved-motion.json"
```

The guarded application requires a preflight under 30 seconds old and a passing
local transport test. It retains live limits, collision checks and command timeout.

### Stop

Press `Ctrl-C` on the motion program, then on the bridge. In Terminal 2, `exit`
returns from the isolated shell. In Terminal 1's parent shell:

```bash
unset PYTHONPATH
conda deactivate
```

Use physical stop controls for an emergency.

## First-time setup

Requires Ubuntu 22.04 **x86_64**, Git, Conda, about 10 GB free, and wired access
to Sawyer. If needed, install [Miniforge](https://github.com/conda-forge/miniforge#install).
Initialize Conda in each new parent terminal when `conda activate` is unavailable:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
# For Miniforge, use ~/miniforge3/etc/profile.d/conda.sh instead.
```

On a new PC, clone the `conda` branch; existing users start with `cd ~/sawyer_rs`:

```bash
git clone --branch conda https://github.com/priyankan-v/Sawyer_Bridge.git ~/sawyer_rs
cd ~/sawyer_rs
export CONDA_PKGS_DIRS="$PWD/.conda/pkgs"
export CONDA_CHANNEL_PRIORITY=strict
conda create -y -p "$PWD/.conda/envs/noetic" --file config/conda/noetic-linux-64.explicit.txt
conda create -y -p "$PWD/.conda/envs/humble" --file config/conda/humble-linux-64.explicit.txt
```

These locks install the tested package versions. `environment*.yml` lists the
direct dependencies; fresh YAML solves require revalidation.

Configure the network once:

```bash
cp -n config/network.env.example config/network.env
nano config/network.env
ip -br addr
ip route get 169.254.121.3
```

| Setting | Existing lab value |
|---|---|
| `ROS_MASTER_URI` | `http://169.254.121.3:11311` |
| `ROS_IP` | `169.254.121.2` — change to your PC's wired address |
| `ROS_DOMAIN_ID` | `42` — match across ROS 2 terminals |

Sawyer must be able to connect back to `ROS_IP`. Verify addresses before use;
keep `ROS_HOSTNAME` unset. Scripts do not configure the network or enable the robot.

## Build

From the repository root, run these blocks in order. Each `exit` returns to the
parent shell. Build outputs use separate `*_conda` directories.

### 1. ROS 1 interfaces

```bash
./scripts/env.sh ros1
cd "$SAWYER_ROOT/ros1_ws"
catkin_make --build build_conda -j4 \
  -DCATKIN_DEVEL_PREFIX="$SAWYER_ROOT/ros1_ws/devel_conda" \
  -DCMAKE_INSTALL_PREFIX="$SAWYER_ROOT/ros1_ws/install_conda" \
  -DPYTHON_EXECUTABLE="$SAWYER_ROOT/scripts/python_noetic.sh" \
  -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5
exit
```

### 2. ROS 2 interfaces and application

```bash
./scripts/env.sh ros2
cd "$SAWYER_ROOT/ros2_ws"
export CMAKE_BUILD_PARALLEL_LEVEL=4
colcon --log-base log_conda build --build-base build_conda --install-base install_conda \
  --symlink-install --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" -DCMAKE_POLICY_VERSION_MINIMUM=3.5
exit
```

### 3. Bridge

```bash
SAWYER_BUILD=1 ./scripts/env.sh bridge
cd "$SAWYER_ROOT/bridge_ws"
export CMAKE_BUILD_PARALLEL_LEVEL=4
colcon --log-base log_conda build --base-paths ros1_bridge \
  --build-base build_conda --install-base install_conda --symlink-install \
  --packages-select ros1_bridge --cmake-force-configure --cmake-args -DBUILD_TESTING=OFF \
  -DCMAKE_C_COMPILER="$CC" -DCMAKE_CXX_COMPILER="$CXX" \
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" -Dxmlrpcpp_DIR="$xmlrpcpp_DIR" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
exit
```

The included bridge source is already patched; no submodule or patch step is needed.

## Tests

From the repository root, run interface/unit checks and isolated simulated control:

```bash
./scripts/env.sh ros1 python scripts/verify_ros1.py
./scripts/env.sh ros2 python scripts/verify_interfaces.py
./scripts/env.sh bridge ros2 run ros1_bridge dynamic_bridge -- --print-pairs > .runtime/conda-pairs.txt
/usr/bin/python3 scripts/check_pairs.py .runtime/conda-pairs.txt
./scripts/env.sh ros2 python -m unittest discover -s tests -v
./scripts/env.sh ros1 python tests/camera_control_checks.py
env -u PYTHONPATH -u PYTHONHOME PYTHONNOUSERSITE=1 /usr/bin/python3 tests/run_local.py
env -u PYTHONPATH -u PYTHONHOME PYTHONNOUSERSITE=1 /usr/bin/python3 tests/run_camera_local.py
env -u PYTHONPATH -u PYTHONHOME PYTHONNOUSERSITE=1 /usr/bin/python3 tests/run_gripper_local.py
env -u PYTHONPATH -u PYTHONHOME PYTHONNOUSERSITE=1 /usr/bin/python3 tests/run_cartesian_local.py
```

The local suite enforces a private localhost master and DDS domain; it cannot
actuate Sawyer. It writes the transport result required for guarded execution.

<details>
<summary>Additional native and read-only hardware checks</summary>

Native C++ conversions:

```bash
./scripts/env.sh bridge bash -c 'cmake -S "$SAWYER_ROOT/tests/native" -B "$SAWYER_ROOT/.runtime/native_build_conda" -DCMAKE_C_COMPILER="$CC" -DCMAKE_CXX_COMPILER="$CXX"'
./scripts/env.sh bridge cmake --build .runtime/native_build_conda --parallel 2
./scripts/env.sh bridge python tests/native/link_test.py
./scripts/env.sh bridge .runtime/native_build_conda/check_conversion
```

For extended telemetry and FK/IK checks, stop Terminal 1's bridge first:

```bash
source config/network.env
env -u PYTHONPATH -u PYTHONHOME PYTHONNOUSERSITE=1 /usr/bin/python3 tests/run_hardware_readonly.py
```

This suite publishes no robot commands; extra head/gripper interfaces must exist.

</details>

## Troubleshooting

| Symptom | Action |
|---|---|
| `No bridge subscriber` | Restart Terminal 1 with `commands:=true`; check both command-topic subscriptions. |
| Preflight `ready: false` | Read `errors` and `position_limit_errors`; resolve the robot state before motion. |
| No joint states | Check wired routing, `ROS_MASTER_URI`, `ROS_IP`, DDS domain and best-effort QoS. |
| Camera pane waiting | Check `cameras:=true` and `--stream-info`; select a camera or use `--start-both`. An advertised image topic may have no frames. |
| Camera display unavailable | Run from a graphical desktop or use `--headless --duration 6`. |
| Gripper state timeout | Restart the bridge with the updated config: gripper state must use `durability: transient_local` to retain Sawyer's latched feedback. Add `gripper_commands:=true` for actuation and match `ROS_DOMAIN_ID` in both terminals. |
| Gripper not calibrated | Run `gripper status`, then explicitly run `gripper calibrate` before open/close. Calibration moves the fingers. |
| Python/NumPy import errors | Use `env.sh`; keep system ROS Python 3.10 paths out of Conda Python 3.12. |
| Conda/OpenSSL deactivation error | Run `unset PYTHONPATH` before `conda deactivate`. |
| Missing mappings | Build ROS 1, ROS 2, then the bridge; run `check_pairs.py`. |
| EmPy build errors | Keep `tools/empy3` and `python_noetic.sh`; they enforce EmPy 3 ordering. |
| CMake paths point to another environment | Archive the affected Conda build/install outputs and rebuild. |

## Essential files

| Path | Purpose |
|---|---|
| `environment*.yml`, `config/conda/` | Dependency specifications and tested package locks |
| `config/network.env`, `config/sawyer_bridge.yaml` | Local network settings and topic directions/QoS |
| `ros1_ws/src/`, `ros2_ws/src/` | Intera interfaces and ROS 2 control application |
| `bridge_ws/ros1_bridge/` | Customized bridge source used by the Conda build |
| `scripts/env.sh`, `scripts/python_noetic.sh` | Environment isolation and legacy message generation |
| `scripts/camera_control.py`, `docs/CAMERAS.md` | Camera IO control, CLI guide and measured specifications |
| `intera_common/`, `intera_sdk/`, `sawyer_robot/` | Original message, SDK and robot-description sources |
| `tests/`, `docs/results/conda/` | Validation programs and recorded evidence |

[MIGRATION.md](MIGRATION.md) records changes and validation.
ROS action protocol bridging is unsupported.

For a detailed explanation of environments, message conversion and data flow,
see [How the Conda bridge works](docs/CONDA_BRIDGE_EXPLAINED.md).


