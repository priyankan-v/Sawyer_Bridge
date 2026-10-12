# Conda migration report

Recorded validation: **2026-10-12**. For commands, use the
[README](README.md): [setup](README.md#first-time-setup), [build](README.md#build),
[daily use](README.md#daily-use), [position control](README.md#position-control),
and [tests](README.md#tests).

## What changed

The `conda` branch uses two isolated RoboStack Conda environments.
The existing ROS 1–ROS 2 architecture, message definitions, bridge conversions,
motion programs and safety checks are preserved.

| Component | Result |
|---|---|
| Original branch | `main` preserved at `37d2c20`; environments and builds retained |
| Noetic environment | 526 → 172 required packages |
| Humble environment | 715 → 305 required packages |
| Maintained shell scripts | 10 → 2 |
| Bridge source | 44 unchanged files included in `bridge_ws/ros1_bridge/` |
| Build outputs | Separate `build_conda`, `devel_conda`, `install_conda`, `log_conda` |

The former bridge gitlink lacked `.gitmodules`. The Conda branch includes its
source directly. The original nested checkout remains preserved locally at
`5423daa`; the existing patch/base records remain in `patches/`.
No system ROS/Python installation, network setting or onboard software was changed.

## Environment and essential scripts

| File | Purpose |
|---|---|
| `environment.yml` | Humble direct dependencies |
| `environment-noetic.yml` | Noetic direct dependencies |
| `config/conda/*-linux-64.explicit.txt` | Exact dependency builds and MD5 checksums used in validation |
| `scripts/env.sh` | Clears inherited paths and selects isolated ROS 1, ROS 2 or bridge runtime |
| `scripts/python_noetic.sh` | Keeps EmPy 3 ahead of EmPy 4 in catkin's cached generator environment |

Both environments retain Python **3.12.15**, NumPy **2.5.3**, Boost **1.88.0**,
CMake **4.2.3** and libstdc++ **16.2.0** from the working package set. Channels
are the matching RoboStack distro and conda-forge, with strict priority and
`nodefaults`. Desktop/tutorial extras were excluded.

The bridge uses Humble first, selected Noetic libraries, pkg-config and
`xmlrpcpp_DIR`. System Humble's Python 3.10 stays separate. The explicit locks
were installed and tested; the YAML alternatives passed dependency solving only.

## File changes

**Created:** two environment YAML files, two explicit locks,
`bridge_ws/ros1_bridge/`, `scripts/configure_bridge.py`,
`ros2_ws/src/sawyer_ros2_control/launch/bridge.launch.py`, this report and
`docs/results/conda/` evidence.

**Modified:** `.gitignore`, `.vscode/settings.json`, `README.md`,
`ros1_ws/src/CMakeLists.txt` (portable upstream file replacing an absolute symlink),
`env.sh`, `python_noetic.sh`, control-package `setup.py`/`package.xml`, local and
hardware test runners, native-test paths, hardware telemetry checks and the
current `docs/results/local_transport.json`.

**Removed:** legacy environment configuration, historical validation outputs,
the broken gitlink entry, the wrappers
below, and eight redundant guides: `0_DAILY_STARTUP.md`, `1_INITIAL_SETUP.md`,
`2_RUNTIME_SETUP_AND_VERIFICATION.md`, and `docs/{HOW_THE_SETUP_WORKS,INTERFACES,
PROGRESS,SINGLE_JOINT_CONTROL,VALIDATION}.md`. Their original versions remain on `main`.

| Removed wrapper (`scripts/`) | Replacement |
|---|---|
| `build.sh` | Documented catkin/colcon commands |
| `apply_bridge_patch.sh` | Already-patched bridge source |
| `start_bridge.sh` | ROS launch file and Python parameter configuration |
| `sawyer.sh` | Existing Python preflight |
| `control.sh` | Preflight followed by `ros2 run` |
| `test_local.sh` | Python runner with enforced localhost isolation |
| `test_native.sh` | CMake and native conversion checks |
| `test_hardware_readonly.sh` | Existing Python hardware runner |

Generated workspace setup scripts remain build outputs. Upstream
`intera_sdk/intera.sh` is retained as a reference; it assumes Kinetic and is
unused here. Original sources, licenses and historical validation data remain.

## Validation results

| Check | Recorded result |
|---|---|
| Git preservation | `main` unchanged; original nested bridge clean; migration on `conda` |
| Conda environments | Both exact locks installed; activation, imports and CLI checks passed |
| ROS 1 / ROS 2 builds | Passed; all 51 interface definitions load and serialize in each environment |
| Bridge mappings | All 46 Intera messages, three services and standard JointState present |
| Native conversions | Exact ROS 1 wire bytes, empty arrays and bool arrays passed |
| Unit tests | Migration: 34 passed; current ROS 2 suite: 53 passed; Noetic camera IO checks: 8 passed |
| Camera viewer | Hidden Tk startup without frames, one active camera, rendering, stale-image clearing and shutdown passed; two simulated image streams bridged at 10 FPS with matching pixels |
| Physical cameras (read-only) | Hand: 28 frames in 6 s, 4.754 FPS, 752×480 mono8; head inactive. `--require-both` correctly failed with `PARTIAL`. No camera controls sent |
| Camera controls and concurrent video | Head/hand selection, camera-only acknowledgements, stop-all and stop-on-exit passed. Both physical feeds reached ROS 2 concurrently: head 1280×800 bgra8 at 5.96 FPS; hand 752×480 mono8 at 4.77 FPS. Initial stream state restored |
| Isolated transport | 200/200 JointCommands with identical fields; zero loss; JointStates at 100.005 Hz |
| Simulated control | Relative/absolute movement, limit recovery/rejection, duration rejection and collision abort passed |
| Startup failures | Missing-master preflight and invalid-config launch fail correctly |
| Physical read-only checks | Seven arm joints with finite positions/velocities at 99.994 Hz; 851 matched ROS 1/ROS 2 samples; extra telemetry and FK/IK passed |
| Physical guarded dry run | Correctly rejected out-of-limit `right_j6`; no commands sent |
| Physical movement | **Not run; explicit operator approval required** |

Evidence is in [docs/results/conda/](docs/results/conda/). The guarded application
uses the Conda proof in `docs/results/local_transport.json`. Historical outputs
were archived in `.runtime/legacy-results/` and removed from this branch;
original results also remain on `main`.
Camera commands and measured specifications are in [docs/CAMERAS.md](docs/CAMERAS.md);
latest evidence is [camera_options_validation.json](docs/results/conda/camera_options_validation.json).

## Limitations and resolved issues

- Last reported `right_j6`: **−4.7138359375 rad**, below live lower limit
  **−4.7124 rad**. Resolve through the operator's normal recovery procedure;
  rerun readiness checks before approving the documented 0.01 rad / 5 s test.
- ROS action definitions build, but action protocol bridging remains unsupported.
- Camera FPS/resolution settings are known; model-specific hardware maximums
  are not exposed by this controller and have not been verified. Concurrent
  streaming passed bounded tests, rather than prolonged workload testing.
- Fresh YAML solutions have not had runtime validation; use the tested locks.
- Conda base OpenSSL/plugin errors on deactivation were resolved by clearing
  `PYTHONPATH`. The README includes this step.
- Initial sandbox DNS and local-master timeout failures passed on retry.
  A temporary application-symlink edit was reverted and a Humble launch import
  incompatibility fixed before successful validation.
- Legacy CMake/CMP0148 and compileall “Can't list” diagnostics remain in logs;
  modules, binaries and conversions passed independent checks.

No robot enable, homing, calibration or physical motion command was sent during
migration. Use the README's approved-motion procedure only after readiness passes.
