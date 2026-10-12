# Sawyer ROS 1–ROS 2 bridge with Conda

This workstation connects ROS 2 applications to Sawyer's existing ROS 1
controller using the customized C++ `ros1_bridge`. It receives joint states and
safety feedback and supports the existing position-command applications.
The robot's software and network configuration are unchanged.

The original working Pixi implementation is preserved on `main` at
`37d2c20`. This branch is `conda`. See [MIGRATION.md](MIGRATION.md) for inspection,
script decisions, test results, and remaining hardware validation.

```text
Sawyer ROS 1 master/controller (Ethernet)
       ↕ ROS 1 messages
C++ parameter_bridge (Conda Humble + selected Noetic libraries)
       ↕ ROS 2 DDS
sawyer_ros2_control (Conda Humble)

ROS 1 diagnostics and configuration run separately in Conda Noetic.
```

Two environments are necessary. The inspected working RoboStack packages use
**Python 3.12.15 in both environments**, NumPy 2.5.3 and Boost 1.88.0. These are
Conda builds of Noetic/Humble, rather than Ubuntu's Python 3.8/3.10 builds.
System Humble under `/opt/ros/humble` uses Python 3.10 and is not part of this
bridge's build or runtime. Do not source it into a project environment.
[RoboStack documents this isolation requirement](https://robostack.github.io/GettingStarted.html).

Conda manages the same package ecosystem as the working Pixi installation.
`environment.yml` describes Humble; `environment-noetic.yml` describes Noetic.
Desktop GUIs, RViz and tutorials are excluded. Exact transitive package builds
and MD5 checksums are in `config/conda/*-linux-64.explicit.txt`; use these locks
for the reproducible installation tested here. Vendored EmPy 3.3.4 is retained
for legacy generators because the package environments include incompatible
EmPy 4. System Python and `/opt/ros` are not changed.

## Install on a new PC

Prerequisites: Ubuntu 22.04 **x86_64**, Git, Conda/Miniforge, about 10 GB free
for environments/builds, a wired connection routable to Sawyer, and permission
to access the controller. An existing system ROS installation may remain.
No system Noetic installation, Docker, Intera SDK build, or `sudo pip` is needed.

If Conda is absent, install [Miniforge](https://github.com/conda-forge/miniforge#install)
using its Linux x86_64 installer, then open a Conda-enabled Bash terminal.
For existing Miniconda, initialize the shell with its installation path, e.g.:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
# Or: source ~/miniforge3/etc/profile.d/conda.sh
conda --version
```

Clone this branch (the customized bridge source is included; no submodule step):

```bash
git clone --branch conda https://github.com/priyankan-v/Sawyer_Bridge.git ~/sawyer_rs
cd ~/sawyer_rs
```

The clone command requires this branch to have been pushed by its owner.
Until then, transfer this working checkout including the new tracked sources.
For this existing PC, simply use the current `conda` checkout.

Create the two environment prefixes inside the repository. Run from its root:

```bash
export CONDA_PKGS_DIRS="$PWD/.conda/pkgs"
export CONDA_CHANNEL_PRIORITY=strict
conda create -y -p "$PWD/.conda/envs/noetic" --file config/conda/noetic-linux-64.explicit.txt
conda create -y -p "$PWD/.conda/envs/humble" --file config/conda/humble-linux-64.explicit.txt
```

The readable YAML alternative performs a fresh dependency solve and may select
newer transitive packages. Revalidate it before using hardware:

```bash
conda env create -p "$PWD/.conda/envs/noetic" -f environment-noetic.yml
conda env create -p "$PWD/.conda/envs/humble" -f environment.yml
```

Choose one creation method. Do not update an existing tested environment
silently. [Conda explains explicit locks and YAML environments](https://docs.conda.io/projects/conda/en/stable/user-guide/tasks/manage-environments.html).

## Build once

Build outputs have separate `*_conda` names so existing Pixi outputs stay usable.
Run each block in sequence. `env.sh` starts an isolated shell and initializes
its ROS environment; `exit` returns to the parent terminal. It intentionally
clears inherited ROS/Python/compiler paths. Do not **source** this script.

**Noetic build shell:**

```bash
cd ~/sawyer_rs
./scripts/env.sh ros1
cd "$SAWYER_ROOT/ros1_ws"
catkin_make --build build_conda -j4 \
  -DCATKIN_DEVEL_PREFIX="$SAWYER_ROOT/ros1_ws/devel_conda" \
  -DCMAKE_INSTALL_PREFIX="$SAWYER_ROOT/ros1_ws/install_conda" \
  -DPYTHON_EXECUTABLE="$SAWYER_ROOT/scripts/python_noetic.sh" \
  -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5
exit
```

**Humble build shell:**

```bash
./scripts/env.sh ros2
cd "$SAWYER_ROOT/ros2_ws"
export CMAKE_BUILD_PARALLEL_LEVEL=4
colcon --log-base log_conda build --build-base build_conda --install-base install_conda \
  --symlink-install --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" -DCMAKE_POLICY_VERSION_MINIMUM=3.5
exit
```

**Bridge build shell:**

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

The original ROS 1 sources are relative links into `intera_common`. The ROS 2
ports and complete field mappings are already checked in. Only regenerate
them when intentionally updating interfaces:
`/usr/bin/python3 scripts/prepare_interfaces.py`.
The catkin toplevel file is a portable copy of the upstream file, replacing
its former absolute Pixi symlink. The bridge's Sawyer patch is already present
in `bridge_ws/ros1_bridge`; do not apply it again.

## Connect and run (two terminals)

Sawyer's previously used controller address is `169.254.121.3`; this PC's wired
address is `169.254.121.2`. Verify routing and the actual workstation IP before
using these values on another PC. Never assign the same IP to two PCs.
The controller must be able to connect back to `ROS_IP` for ROS 1 TCPROS.

```bash
cd ~/sawyer_rs
ip -br addr
ip route get 169.254.121.3
cp -n config/network.env.example config/network.env
# Edit config/network.env to reflect your workstation; retain the correct master.
cat config/network.env
```

The example uses `ROS_MASTER_URI=http://169.254.121.3:11311` and DDS domain 42.
Scripts do not change interface addresses, the controller, or the robot's state.

**Terminal 1 — activate and start read-only bridging:**

```bash
cd ~/sawyer_rs
conda activate "$PWD/.conda/envs/humble"
source config/network.env
./scripts/env.sh ros1 python scripts/preflight.py --output .runtime/readonly-preflight.json
./scripts/env.sh ros2 ros2 launch sawyer_ros2_control bridge.launch.py
```

Run preflight successfully before starting the launch command. This initializes
ROS 1 separately, loads `/sawyer_bridge/*` ROS 1 parameters, then starts the C++
`parameter_bridge` in its mixed-library environment. If configuration fails,
the bridge is not started. The default configuration has command paths disabled.
No node automatically enables the robot. Start only one bridge per robot/domain.

**Terminal 2 — initialize ROS 2, then inspect:**

```bash
cd ~/sawyer_rs
conda activate "$PWD/.conda/envs/humble"
./scripts/env.sh ros2
# The isolated shell automatically reads config/network.env and the Conda overlay.
ros2 topic list --no-daemon
ros2 topic type /robot/joint_states
ros2 topic echo /robot/joint_states sensor_msgs/msg/JointState --once --qos-reliability best_effort
ros2 topic hz /robot/joint_states
ros2 interface show intera_core_msgs/msg/JointCommand
ros2 run sawyer_ros2_control command read
```

Check `right_j0` through `right_j6`, finite positions and velocities, timestamps,
and a steady rate. Sawyer may publish separate arm/head/gripper JointState
messages on the same topic; applications merge arm subsets. Joint states use
best-effort QoS. `Ctrl-C` stops `hz`; the guarded `command read` collects all
seven arm joints and requires their velocities.

For ROS 1 troubleshooting without leaving Terminal 2, run an isolated subprocess:

```bash
"$SAWYER_ROOT/scripts/env.sh" ros1 rostopic info /robot/limb/right/joint_command
"$SAWYER_ROOT/scripts/env.sh" ros1 rosmsg show intera_core_msgs/JointCommand
"$SAWYER_ROOT/scripts/env.sh" ros1 python "$SAWYER_ROOT/scripts/preflight.py"
```

## Prepare position control without movement

Run the isolated forwarding/safety tests below first. Inspect a small target
using the existing application in Terminal 2:

```bash
ros2 run sawyer_ros2_control move_joint --joint right_j2 --distance 0.01 --time 5 --dry-run
```

`move_joint` is the original single-joint application: **it publishes unless
`--dry-run` is supplied**. It validates measured feedback, limits and minimum
trajectory duration, but does not implement the guarded application's full
readiness/collision monitoring. Its source remains unchanged.

For the first approved hardware test, use the existing guarded `command`
application, which defaults to a plan and requires explicit execution flags.
Capture a target 0.01 rad from the current `right_j2` feedback:

```bash
ros2 run sawyer_ros2_control command read > "$SAWYER_ROOT/.runtime/current-joints.json"
TARGET=$(python -c 'import json,os; from pathlib import Path; p=Path(os.environ["SAWYER_ROOT"])/".runtime/current-joints.json"; print(json.loads(p.read_text())["right_j2"]+0.01)')
ros2 run sawyer_ros2_control command one --joint right_j2 --position "$TARGET" --duration 5
```

The planner rejects a target outside live/URDF limits, excessive displacement,
an unready/moving robot, stale feedback or a collision. If this direction is
invalid, stop and choose a reviewed in-range target. Do not bypass the checks.

**Only after explicit operator approval:** stop Terminal 1's read-only launch
with `Ctrl-C`, then restart it with command transport enabled:

```bash
./scripts/env.sh ros2 ros2 launch sawyer_ros2_control bridge.launch.py commands:=true
```

In Terminal 2, with the reviewed `TARGET` still set, the exact motion command is:

```bash
"$SAWYER_ROOT/scripts/env.sh" ros1 python "$SAWYER_ROOT/scripts/preflight.py" \
  --require-ready --output "$SAWYER_ROOT/.runtime/preflight.json" && \
ros2 run sawyer_ros2_control command one --joint right_j2 --position "$TARGET" \
  --duration 5 --execute --operator-approved --report "$SAWYER_ROOT/.runtime/approved-motion.json"
```

This uses POSITION_MODE, 0.05 speed ratio, 0.02 rad maximum displacement,
0.2 s command timeout, live limits, collision/freshness checks and tracking-error
monitoring. It requires a preflight less than 30 seconds old and a passing local
transport result. No physical motion was run during migration.

**Shutdown:** stop the command program with `Ctrl-C`, then stop the bridge in
Terminal 1. The guarded application attempts a measured-position hold only
when feedback and safety state are valid, then stops publishing; the controller's
command timeout remains active. An emergency requires the physical stop controls.
Exit the isolated shell, then run `unset PYTHONPATH; conda deactivate` in each
parent shell. Clearing ROS Python paths before calling Conda avoids loading
Python 3.12 ROS dependencies in Conda base's different Python version. Do not kill
unrelated ROS processes or shut down Sawyer's master.

## Validation commands

Run from the repository root. These check interfaces and print converters
without publishing commands:

```bash
./scripts/env.sh ros1 python scripts/verify_ros1.py
./scripts/env.sh ros2 python scripts/verify_interfaces.py
./scripts/env.sh bridge ros2 run ros1_bridge dynamic_bridge -- --print-pairs > .runtime/conda-pairs.txt
/usr/bin/python3 scripts/check_pairs.py .runtime/conda-pairs.txt
./scripts/env.sh ros2 python -m unittest discover -s tests -v
```

Run the existing integration suite on its private localhost master and DDS
domain. It exercises real bridge conversions with simulated joints only:

```bash
env -u PYTHONPATH -u PYTHONHOME PYTHONNOUSERSITE=1 /usr/bin/python3 tests/run_local.py
```

The runner enforces master `127.0.0.1:11421`, ROS_IP `127.0.0.1`, DDS domain 87
and `ROS_LOCALHOST_ONLY=1`, refuses an occupied master port, and cleans up only
its own processes. It never attaches motion fixtures to Sawyer's graph.

Native C++ wire/ABI checks:

```bash
./scripts/env.sh bridge bash -c 'cmake -S "$SAWYER_ROOT/tests/native" -B "$SAWYER_ROOT/.runtime/native_build_conda" -DCMAKE_C_COMPILER="$CC" -DCMAKE_CXX_COMPILER="$CXX"'
./scripts/env.sh bridge cmake --build .runtime/native_build_conda --parallel 2
./scripts/env.sh bridge python tests/native/link_test.py
./scripts/env.sh bridge .runtime/native_build_conda/check_conversion
```

Optional extended **read-only hardware** suite, with Terminal 1 stopped:

```bash
source config/network.env
env -u PYTHONPATH -u PYTHONHOME PYTHONNOUSERSITE=1 /usr/bin/python3 tests/run_hardware_readonly.py
```

It checks extra head/gripper telemetry and FK/IK services that may not be exposed
on every controller. It never publishes robot commands. A failure in extra
telemetry is reported rather than treated as a successful core bridge test.

## Troubleshooting and files

- Missing prefix: create both `.conda/envs/*` prefixes. For external prefixes,
  export `SAWYER_NOETIC_PREFIX` and `SAWYER_HUMBLE_PREFIX` before building/running.
- Conda base plugin/OpenSSL errors on deactivation: `unset PYTHONPATH` before
  `conda deactivate` or changing environments. RoboStack activation sets ROS
  Python paths that must not enter Conda base's Python.
- Python/NumPy import errors: use `scripts/env.sh`; remove system ROS sourcing
  from the parent workflow. Do not add system Python 3.10 paths to Python 3.12.
  Check `./scripts/env.sh ros2 python -c 'import sys,numpy; print(sys.executable,numpy.__version__)'`.
- No telemetry: inspect wired routing, `ROS_IP`, controller master availability,
  matching `ROS_DOMAIN_ID`, launch output, and best-effort subscriber QoS.
- Missing converters: build both interface workspaces before the bridge and run
  `check_pairs.py`. All 46 Intera messages and three services are expected.
  The two ported action definitions do not provide ROS action interoperability.
- CMake errors after changing prefixes: archive/rename only the Conda build and
  install directories and rebuild. Cached paths cannot be shared with Pixi.
- EmPy errors: keep `tools/empy3` and `python_noetic.sh`; catkin's cached generator
  environment otherwise restores EmPy 4 ahead of the compatibility module.
- Legacy CMake warnings: `CMAKE_POLICY_VERSION_MINIMUM=3.5` accommodates CMake 4;
  warnings are retained in build logs. Compiler/generator errors are failures.
- Preflight or motion rejection: inspect the reported state/limits and correct
  the underlying condition. No auto-enable, homing or calibration is provided.

```text
environment.yml, environment-noetic.yml   direct, versioned dependencies
config/conda/                            exact Linux x86_64 package locks
config/network.env.example               robot-specific network template
config/sawyer_bridge.yaml                selective direction/QoS topic policy
ros1_ws/src/                             original catkin interfaces
ros2_ws/src/                             ament interfaces and existing control app
bridge_ws/ros1_bridge/                   unchanged customized bridge source
patches/                                 upstream base and existing Sawyer patch
scripts/env.sh                           essential environment isolation
scripts/python_noetic.sh                 essential EmPy generator ordering
scripts/configure_bridge.py              ROS 1 parameter configuration
scripts/preflight.py                     read-only graph/safety checks
scripts/verify_*.py, check_pairs.py        interface/mapping validation
scripts/prepare_interfaces.py             optional reproducible interface generation
tests/                                   simulated transport and safety validation
tools/empy3/                             licensed legacy generator compatibility
intera_common/, intera_sdk/, sawyer_robot/ original source references and limits
docs/interface_manifest.json             source hashes and field renames
docs/results/                            prior results and migration evidence
MIGRATION.md                             migration audit and validation results
```

Generated setup `.sh` files under workspace outputs belong to catkin/colcon;
they are ignored, never hand edited, and regenerated by builds. The historical
SDK `intera_sdk/intera.sh` is preserved as upstream source but is not used: it
assumes system ROS Kinetic and placeholder addresses. The two project shell
scripts above are the only maintained wrappers. `config/bridge.yaml` retains
the previous joint-state-only configuration for compatibility.
