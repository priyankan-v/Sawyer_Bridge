# System-Humble Conda package migration

Branch: **conda-pkg**. Validation date: **2026-10-13**.

## Result

The repository builds a local **sawyer-ros2-bridge 0.2.0** Conda artifact for
Ubuntu 22.04 x86_64 with existing system ROS 2 Humble. It packages the selective
bridge, Intera interfaces and applications, plus private ROS 1 clients. Users do
not create/source Noetic or Humble Conda environments. Only a Sawyer overlay is
sourced above `/opt/ros/humble`.

The previous `conda` branch remains at `b5be93d8b304929be795c2a139dc5f05cc9f1d41`;
`main` remains at `37d2c20e073069b416bb8571a461b1345792aa80`. The worktree was clean
before branching. Existing environments, build outputs and onboard software were
preserved. No physical actuation was performed.

## Implementation

- Native build uses system GCC, Boost 1.74 and Python 3.10 to match system Humble.
- SHA256-pinned ROS 1 archives are selected individually; no ROS distribution
  metapackage or desktop is installed. Private modules include the existing
  preflight/camera/configuration helpers' dependencies.
- Bridge discovery is restricted to builtin, standard, geometry, sensor and
  Intera interfaces. Existing bridge conversions and all Intera definitions are
  preserved. `scripts/move_joint.py` and its motion logic are unchanged.
- Installed-file discovery replaces checkout/environment paths in launch and
  camera helpers. User validation files live outside the package.
- Relative ELF paths support installation in a different prefix. Conda installs
  no Python, ROS 1 or ROS 2 dependency package for this artifact.
- An explicit self-test uses a private localhost master and DDS domain, then
  shuts down its owned processes. That master is not started during normal use.

## Validation

| Test | Result |
|---|---|
| Native ROS 1 clients + Intera build | PASS |
| System-Humble Intera/application/bridge build | PASS |
| Required JointState/JointCommand/HeadPanCommand mappings | PASS |
| Python unit tests | 57 PASS |
| Private camera IO unit checks | 8 PASS |
| Native C++ conversions, empty/bool arrays, exact ROS 1 serialization | PASS |
| Local arm/head controls, duration/limit/no-op checks, collision abort | PASS on synthetic localhost fixtures |
| Camera pixels, selection/start/stop, two concurrent read-only viewers | PASS on synthetic localhost fixtures |
| Conda archive creation and built-in import/CLI tests | PASS |
| Clean-prefix install and relocated ELF linkage | PASS; private ROS 1 + /opt/ros/humble, no original Conda ROS paths |
| Installed transport self-test | PASS for JointState, JointCommand, HeadPanCommand and Image |
| Installed bridge → physical Sawyer telemetry | PASS: 703 samples in six seconds, all seven arm positions/velocities and live limits received; aggregate topic rate approximately 120 Hz |
| Physical joint/gripper/head actuation | NOT RUN; requires explicit operator approval |
| Fresh-PC installation | NOT RUN; release prerequisite before public distribution |
| Public upload | NOT RUN; local artifact/channel preparation only |

Historical results under `docs/results/conda/` belong to the previous architecture.
Current package evidence is under `docs/results/conda-pkg/`.

Conda's linkage audit reports externally supplied Humble/Ubuntu libraries as
missing Conda dependencies or potential overlinking. This is intentional for this
platform-specific artifact. Relocated `ldd` checks and runtime tests verify their
actual resolution. The package must not be advertised as a generic Linux or
cross-distro Conda binary.

## Files

**Created:** `PACKAGING.md`, `packaging/{ros1-sources.json,build_package.py,
relocate.py,selftest.py,test_fixture.py,test_client.py,hardware_readonly.py,
recipe/meta.yaml}`, application `runtime.py`, package/runtime tests, and evidence.

**Modified:** README and architecture/camera/command-path guides, bridge package
selection and private interface discovery, launch, application entry points,
camera helper lookup, guarded validation-report lookup, developer environment
helper, integration/native tests and `.gitignore`.

**Removed from this branch:** the two ROS environment YAMLs and their explicit
locks, obsolete `scripts/python_noetic.sh`, and unused vendored EmPy 3 compatibility
files. System EmPy 3 is used during the native build. Existing environment
installations were not deleted.

The only maintained shell helper is `scripts/env.sh`, retained for development
and old regression runners. It selects system Humble or private test clients;
it is not shipped in the Conda package or required for user operation.
Generated ROS shell setup files remain standard build outputs.

## Commands and remaining checks

[README](README.md) contains user install, networking, bridge, camera, dry-run and
shutdown commands. [PACKAGING.md](PACKAGING.md) contains exact native build,
Conda creation, relocation, test and local-channel commands.

Public release still requires fresh-PC validation, an agreed channel and operator
approved hardware command verification. The existing passive-head issue remains
unresolved; a successful package build or simulated head test does not establish
physical head actuation. Package validation never bypasses robot safety mechanisms.
