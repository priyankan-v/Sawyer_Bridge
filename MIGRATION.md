# Conda migration audit

Migration date: 2026-10-12 (America/Chicago). Results below distinguish local
simulation, interface checks and physical read-only observations. Physical
motion remains untested and requires explicit operator approval.

## Inspected architecture and preservation

Initial branch: `main`, commit `37d2c20` (`Remove generated ROS build files`).
The tracked working tree was clean, with no untracked, nonignored files. Existing
ignored data included Pixi environments, ROS build outputs, local network
settings and runtime artifacts. No stash/reset was necessary. `conda` did not
exist; it was created from that commit. `main` has not been committed to or reset.

The only nested Git repository was `bridge_ws/src/ros1_bridge`, on
`sawyer-custom` at `5423daa76629d86e6fbe6c19b656b74c017c4e23`, clean. The outer
repository recorded it as a gitlink but had no `.gitmodules`; `git submodule
status` failed. Its customized commit consists of the existing Sawyer patch on
upstream base `611755fd917285316051cbea80507e8b2f6b7ec1`.

For a reproducible new-PC checkout, this branch replaces the gitlink with the
same **44 versioned source files** under `bridge_ws/ros1_bridge`. Their contents
were compared byte-for-byte with the original checkout. The original nested
repository remains untouched on disk and is ignored on `conda`; `main` retains
its original gitlink. The patch and upstream base records remain available.

Pixi supplied two RoboStack environments, Noetic and Humble, both Python 3.12.
The Noetic workspace builds the unchanged Intera messages with catkin. Separate
ROS 2 ports build with ament/rosidl and explicit camelCase-to-snake_case mappings.
The bridge is compiled against Humble and discovers Noetic via pkg-config,
explicit message Python paths and `xmlrpcpp_DIR`. Humble CMake/library paths
precede Noetic to avoid same-named ROS package conflicts. The robot's ROS 1
master is used directly; communication remains C++ `ros1_bridge` and ROS 2 DDS.

The original selective parameter bridge already supports direction, enabled
flags and QoS. It loads ROS **1** parameters under `/sawyer_bridge`; it does not
consume ROS 2 parameter YAML. The existing patch additionally handles service
field renames, time/duration, bool arrays and empty arrays. No converter was
rewritten. All 46 ordinary Intera messages and three services must map, including
JointCommand. Both action definitions build, but action protocol bridging is
not implemented by the existing bridge.

Telemetry includes joint states, assembly state, endpoint state, collision state,
live limits and head state. Arm command, timeout and speed-ratio topics remain
disabled by default. Robot enable and head commands remain disabled. The original
`move_joint.py`, guarded `command.py`, `safety.py`, joint limits and all interface
definitions are unchanged. The package's `move_joint.py` is an intentional
relative link to `scripts/move_joint.py`, not duplicate application logic.

System Humble Python 3.10 is isolated from Conda Python 3.12. No system package,
Python installation, `/opt/ros` file, robot configuration or onboard software
was changed. The network template/local configuration is unchanged:
master/controller `169.254.121.3`, workstation `169.254.121.2`, DDS domain 42.

## Environment configuration

- `environment.yml`: Humble direct dependencies, exact Python/NumPy/ROS/tool versions.
- `environment-noetic.yml`: Noetic direct dependencies for catkin and ROS diagnostics.
- `config/conda/humble-linux-64.explicit.txt`: 305 exact transitive packages.
- `config/conda/noetic-linux-64.explicit.txt`: 172 exact transitive packages.

Locks were derived from the dependency closure of required seeds in the working
Pixi package metadata, with package URLs and original MD5 checksums. This is
not a wholesale copy of the desktop environments (formerly 715 and 526 packages).
ROS core, standard interface dependencies, rosidl generators, ROS 2 CLI, launch,
colcon, pkg-config, CMake and the existing Conda compiler are retained. GUI,
visualization and tutorial dependencies were excluded. Git is supplied by the
OS. The same Python 3.12.15, NumPy 2.5.3, Boost 1.88.0, CMake 4.2.3 and
libstdc++ 16.2.0 builds remain in the dependency closure.

The channels are RoboStack for the corresponding distro followed by conda-forge,
with `nodefaults`; README commands select strict priority. Explicit locks bypass
fresh solving; the YAML alternative may change transitive versions and requires
revalidation. Local prefixes are `.conda/envs/noetic` and `.conda/envs/humble`.
External prefixes can be selected with `SAWYER_NOETIC_PREFIX` and
`SAWYER_HUMBLE_PREFIX` before building. No Pixi environment was deleted.

## Shell-script audit

All existing project `.sh` files and generated workspace helper contents were
inspected. Generated catkin/ament/colcon environment helpers are build outputs,
not maintained setup scripts; they are ignored and preserved/regenerated rather
than individually deleted. Installed dependency environments are package-owned.

| Original script | Classification | Replacement / reason |
|---|---|---|
| `scripts/env.sh` | Essential | Retained for clean ROS/Python/library/compiler paths and the bridge's ordered mixed environment; now uses Conda prefixes. |
| `scripts/python_noetic.sh` | Essential, build-only | Retained because catkin's cached generator environment otherwise restores EmPy 4 ahead of the project EmPy 3 module. |
| `scripts/build.sh` | Build-only, replaceable | Standard catkin/colcon commands in README; separate Conda build directories. |
| `scripts/apply_bridge_patch.sh` | Build-only, obsolete | The exact customized bridge source is included and already patched. |
| `scripts/start_bridge.sh` | Replaceable | `bridge.launch.py` plus `configure_bridge.py`; preserves settings and process ordering. |
| `scripts/sawyer.sh` | Replaceable | Source network configuration and run the existing read-only Python preflight directly. |
| `scripts/control.sh` | Replaceable | Documented preflight followed by `ros2 run ... command`; application authorization/readiness checks unchanged. |
| `scripts/test_local.sh` | Test-only, replaceable | Isolation moved into `tests/run_local.py`, which runs directly. |
| `scripts/test_native.sh` | Test-only, replaceable | Documented CMake and Python link/check commands. |
| `scripts/test_hardware_readonly.sh` | Test-only, replaceable | Documented network sourcing plus the existing Python test runner. |
| `intera_sdk/intera.sh` | Obsolete for this workflow; upstream reference | Preserved with the original SDK. It assumes Kinetic, placeholder addresses and system catkin; do not use it here. |

The maintained project shell-script count is **10 → 2**. No new shell wrapper
was added. Launch uses separate isolated ROS 1 configuration and bridge processes;
it never sources complete Noetic and Humble environments into one shell.

## Final structure and changed files

See the compact tree and essential-file descriptions in README. The substantive
new files are the two YAML environments, two explicit locks, this report,
`configure_bridge.py`, `bridge.launch.py`, and the unchanged bridge source tree.

Modified files: `.gitignore`, `.vscode/settings.json`, `README.md`,
`ros1_ws/src/CMakeLists.txt` (absolute symlink replaced by unchanged upstream
file), `scripts/env.sh`, `scripts/python_noetic.sh`, control-package `setup.py`
and `package.xml`, local/hardware test runners, native test paths, and read-only
hardware telemetry validation. Existing motion/safety source files are unchanged.

Removed from this branch: `pixi.toml`, `pixi.lock`, the eight replaceable project
shell wrappers listed above, and redundant startup/setup/explanation/progress
Markdown guides. Their contents remain in `main`. The broken gitlink was removed
from this branch's index; its original nested checkout was not removed from disk.
The removed guides are `0_DAILY_STARTUP.md`, `1_INITIAL_SETUP.md`,
`2_RUNTIME_SETUP_AND_VERIFICATION.md`, `docs/HOW_THE_SETUP_WORKS.md`,
`docs/INTERFACES.md`, `docs/PROGRESS.md`, `docs/SINGLE_JOINT_CONTROL.md`, and
`docs/VALIDATION.md`. Original source repositories, licenses, message-generation
manifest and prior validation data remain. New evidence is stored under `docs/results/conda/`.

The simplified main documentation consists of README and this requested audit.
Upstream reference documentation remains with its source. `config/bridge.yaml`
is retained for the existing joint-state-only configuration.

## Exact operating commands

From `~/sawyer_rs`, with Conda available:

```bash
export CONDA_PKGS_DIRS="$PWD/.conda/pkgs"
export CONDA_CHANNEL_PRIORITY=strict
conda create -y -p "$PWD/.conda/envs/noetic" --file config/conda/noetic-linux-64.explicit.txt
conda create -y -p "$PWD/.conda/envs/humble" --file config/conda/humble-linux-64.explicit.txt
```

The three complete build blocks in README are the commands tested here. They
select `build_conda`, `devel_conda`, `install_conda` and `log_conda`, and explicitly
select the Noetic generator Python, Humble Python and Conda bridge compiler.

Terminal 1:

```bash
conda activate "$PWD/.conda/envs/humble"
source config/network.env
./scripts/env.sh ros1 python scripts/preflight.py --output .runtime/readonly-preflight.json
./scripts/env.sh ros2 ros2 launch sawyer_ros2_control bridge.launch.py
```

Terminal 2:

```bash
./scripts/env.sh ros2
ros2 topic type /robot/joint_states
ros2 topic echo /robot/joint_states sensor_msgs/msg/JointState --once --qos-reliability best_effort
ros2 topic hz /robot/joint_states
ros2 interface show intera_core_msgs/msg/JointCommand
ros2 run sawyer_ros2_control command read
ros2 run sawyer_ros2_control move_joint --joint right_j2 --distance 0.01 --time 5 --dry-run
```

For the exact reviewed-target calculation, guarded dry run, command-bridge
restart, fresh preflight and approved motion command, use README's
“Prepare position control without movement” procedure. No motion is executed
without `--execute --operator-approved` in the guarded application. The legacy
single-joint application needs `--dry-run` to avoid publishing.

## Validation results

| Check | Result | Evidence / scope |
|---|---|---|
| Git isolation | PASS | `main` remains `37d2c20e073069b416bb8571a461b1345792aa80`; nested bridge checkout remains clean at `5423daa...`; migration only on `conda`. |
| Conda creation | PASS | Both exact package locks installed into new prefixes; 172 Noetic and 305 Humble packages. |
| YAML configuration | PASS for dependency solving | Both `conda env create --solver libmamba --dry-run` commands resolved; runtime tests used the explicit locks. |
| Python/NumPy/activation | PASS | Both prefixes select Python 3.12.15 and NumPy 2.5.3; direct `conda activate` imports and ROS CLI checks pass. |
| ROS 1 build/interfaces | PASS | Catkin build succeeded; all 51 original interfaces/action definitions load and serialize, with original source hashes unchanged. |
| ROS 2 build/interfaces | PASS | Three packages built; all 51 interface definitions load compiled type support and serialize; Humble CLI works. |
| System Humble isolation | PASS | System Python 3.10 CLI works separately; no project Python 3.12 interfaces loaded into it. |
| Bridge build/mappings | PASS | Bridge built in 4 min 30 s; all 46 Sawyer messages, three services and standard JointState converters present. |
| Native C++ conversion | PASS | Exact ROS 1 wire bytes, empty arrays and boolean arrays round-trip successfully. |
| Existing unit tests | PASS | 34 tests, including duration/limits/readiness/freshness/collision checks. |
| Local bidirectional transport | PASS | Real C++ bridge on private ROS/DDS graphs: 200/200 JointCommands, identical fields, zero loss; 211 JointStates at 100.005 Hz, zero sequence gaps; three services. |
| Local application behavior | PASS | Relative/absolute arm and head trajectories on fixtures, no-op repeated targets, limit recovery, too-short/out-of-range rejection, and collision abort; no hardware commands. |
| Missing master/config failure | PASS | Unavailable-master preflight fails promptly; invalid launch configuration exits nonzero without starting the bridge. |
| Physical ROS 1 graph/subscriber | PASS, read-only | Existing master reached; `/realtime_loop` subscribes to `intera_core_msgs/JointCommand` on the expected arm command topic. |
| Physical ROS 2 joint states | PASS, read-only | All seven arm names, finite positions and velocities; 99.994 Hz; 851 matched ROS 1/ROS 2 samples with no difference in the common window. |
| Physical extra telemetry/FK/IK | PASS, read-only | State, endpoint, collision, limits, head and gripper telemetry; valid FK and one IK solution. |
| Guarded physical dry run | Expected rejection | `right_j6=-4.7138359375` is outside live lower limit `-4.7124`; return code 1, no commands published. |
| Physical movement | NOT RUN | Requires explicit approval and resolution of current guarded readiness rejection. |

Source/build logs and JSON evidence are under `docs/results/conda/`. The current
`docs/results/local_transport.json` is the fresh Conda result required by the
unchanged guarded application. Other historical tracked results were restored
unchanged after testing; the previous transport result remains on `main` and
in `.runtime/pixi-validation/`.

The first environment attempt failed because sandbox DNS was unavailable; the
network-enabled retry succeeded. An initial local-master readiness probe timed
out; a bounded standalone localhost master check and the full rerun passed.
During migration an attempted compatibility-entry simplification hit the existing
application symlink; it was reverted, restoring the original application byte
for byte before passing tests. The first launch import used an action absent in
Humble; this was corrected before runtime tests. These intermediate failures
were fixed rather than counted as passing validation.

Direct Conda deactivation initially exposed a base-environment OpenSSL plugin
conflict: ROS activation puts Python 3.12 packages on `PYTHONPATH`, while this
PC's Conda base uses Python 3.13. `unset PYTHONPATH` before `conda deactivate`
resolves it; the corrected activation/deactivation test passed. This does not
require modifying base Conda or installing/removing any system library. The
workflow also prevents command flags from entering catkin's setup parser.

Legacy CMake deprecation/CMP0148 warnings remain visible in the build logs.
Installed bridge CMake emits a compileall “Can't list” diagnostic for a relative
Python directory; its Python module, C++ binaries and all conversions were
validated independently. These diagnostics do not prevent the successful build.

## Limitations and hardware test approval

The complete environment, build, local transport and physical read-only workflow
are validated. Physical actuation has deliberately not been validated. The
existing guarded application rejects the current `right_j6` position, about
0.001436 rad below its live lower limit, even though controller state reports
homed/enabled/ready and collision-free. Resolve this with the robot operator's
normal procedure before approving the first movement; do not relax the limits.

The prepared test is a reviewed `right_j2` target 0.01 rad from measured feedback,
over five seconds, using the existing guarded POSITION_MODE application. README
contains the exact dry-run, fresh preflight and `--execute --operator-approved`
commands. Command transport must be enabled explicitly after approval. No robot
enable, homing, calibration, or movement command was sent during migration.

Runtime validation used the exact explicit locks. The YAML solver succeeded,
but its independently solved transitive packages were not installed/tested.
Package-channel availability remains a prerequisite on a new PC. The Git branch
is local; it must be pushed by the owner before the documented remote clone can
retrieve it. ROS action protocol bridging remains unsupported, as before.

The original Pixi branch, environments, build outputs, nested bridge source and
network configuration remain preserved. No system or robot changes were made.
