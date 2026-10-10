# Sawyer ROS 1 ↔ ROS 2 workstation bridge

Sawyer's controller remains ROS 1. This project uses the existing Pixi Noetic
and Humble installations, original Sawyer message sources, and `ros1_bridge` to
expose selected telemetry and guarded joint-position commands to ROS 2.
No Docker, custom transport gateway, ROS reinstallation, or onboard change is
required. The previously working bridge installation and joint-state-only
configuration remain available.

```text
Sawyer ROS 1 controller
    ↕ ROS 1 Ethernet
Pixi Noetic + original Intera interfaces in ros1_ws
    ↕ ros1_bridge (Pixi Humble runtime; selected directions)
Pixi Humble + ROS 2 Intera interfaces in ros2_ws
    ↕ ROS 2 DDS
sawyer_ros2_control (monitoring and guarded POSITION_MODE commands)
```

## Environment and dependencies

Both Pixi environments use Python **3.12**. System Humble in `/opt/ros/humble`
uses Python **3.10**. The project wrappers sanitize inherited Python, ROS,
compiler, and library paths before starting a locked Pixi runtime. Use the
wrappers rather than sourcing both installations into one shell.

```bash
cd ~/sawyer_rs
pixi install --locked                    # only needed on a new workstation
./scripts/env.sh ros1 python --version
./scripts/env.sh ros2 python --version
./scripts/env.sh ros1 rosversion -d
./scripts/env.sh ros2 ros2 --help
```

The locked EmPy 4 package is incompatible with legacy ROS 1 generators.
`tools/empy3` supplies the unmodified pure-Python EmPy 3.3.4 module, with source
and license, under Python 3.12. The catkin Python wrapper ensures its precedence
even inside catkin's cached generator environment. No Python 3.10 packages are
imported by Pixi interpreters. Installed Pixi packages and `pixi.lock` are intact.

For system Humble CLI diagnostics only:

```bash
./scripts/env.sh system-ros2 /usr/bin/python3 /opt/ros/humble/bin/ros2 --help
```

This mode uses only system Python 3.10 paths; project custom Python interfaces
were built for Pixi Python 3.12, so use the Pixi ROS 2 wrapper for them. System
Humble is not used to compile or run the project's bridge.

## Build

The copied source repositories must exist at `intera_common`, `intera_sdk`, and
`sawyer_robot`. Only the two original interface packages are included in catkin.
SDK code and robot description packages are inspected, not unnecessarily built.

```bash
cd ~/sawyer_rs
./scripts/build.sh ros1
./scripts/build.sh ros2
./scripts/build.sh bridge
# Equivalent full sequence, after the original repositories are present:
./scripts/build.sh all
```

`prepare_interfaces.py` makes relative links in `ros1_ws/src` to unchanged
original ROS 1 sources and creates separate ROS 2 `ament_cmake`/`rosidl` packages
in `ros2_ws/src`. It records source hashes and field renames in
`docs/interface_manifest.json`. ROS 2 messages, services and actions are all
ported; interface generation alone does not establish action interoperability.

ROS 1 uses `catkin_make`; ROS 2 uses `colcon build`. The bridge uses the existing
checkout and Conda compiler, with ROS 1 package discovery through pkg-config
and ROS 2 discovery through ament. ROS 2 CMake prefixes precede Noetic to prevent
same-name package conflicts. ROS 1 interface source paths and generated Python
paths are supplied explicitly to the bridge generator.

The rebuilt bridge goes to `bridge_ws/build_sawyer`, `install_sawyer`, and
`log_sawyer`. The original `build`, `install`, and `log` directories are retained.
`patches/ros1_bridge_sawyer.patch` records the small source extensions needed for
directional topics and complete service conversion; the build script checks
that this patch is applied. Its base commit is recorded beside the patch.

```bash
SAWYER_BUILD_JOBS=8 ./scripts/build.sh bridge  # optional, sufficient RAM required
./scripts/env.sh ros1 rosmsg show intera_core_msgs/JointCommand
./scripts/env.sh ros1 rosmsg show intera_core_msgs/RobotAssemblyState
./scripts/env.sh ros1 rosmsg show intera_core_msgs/EndpointState
./scripts/env.sh ros2 ros2 interface show intera_core_msgs/msg/JointCommand
./scripts/env.sh ros2 ros2 interface show intera_core_msgs/msg/RobotAssemblyState
./scripts/env.sh ros2 ros2 interface show intera_core_msgs/msg/EndpointState
./scripts/env.sh ros1 python scripts/verify_ros1.py
./scripts/env.sh ros2 python scripts/verify_interfaces.py
./scripts/env.sh bridge ros2 run ros1_bridge dynamic_bridge -- --print-pairs > docs/results/bridge_pairs.txt
/usr/bin/python3 scripts/check_pairs.py docs/results/bridge_pairs.txt
```

Build logs are in `docs/results/build_*.log`. CMake deprecation and CMP0148
developer warnings come from older upstream catkin/Humble CMake modules; they
are retained in the logs. The policy compatibility argument accommodates the
installed CMake without modifying original packages. Compiler failures are
treated as failures and corrected before runtime validation.

## Network and startup after reboot

Previously verified lab addresses are controller `169.254.121.3`, workstation
`169.254.121.2`, and ROS master `http://169.254.121.3:11311`. The live wired
interface inspected during this work is **enp0s31f6**. Verify your current network
before using these values. Scripts do not change network configuration.

```bash
cd ~/sawyer_rs
ip -br addr
cp config/network.env.example config/network.env   # once; inspect addresses first
source config/network.env
./scripts/sawyer.sh
```

`sawyer.sh` reads the controller's actual topic types, graph, safety state and
joint limits, and checks the command subscriber's own XML-RPC subscriptions.
It never publishes commands or enables the robot. On each reboot, power and
connect Sawyer using normal lab procedures, then use these terminals:

Terminal 1 — selected read-only bridge:

```bash
cd ~/sawyer_rs
pixi run bridge-start
```

Terminal 2 — ROS 2 monitoring:

```bash
cd ~/sawyer_rs
pixi run ros2-shell
# Run these commands inside the shell that opens:
ros2 topic echo /robot/joint_states --once --qos-reliability best_effort
ros2 topic echo /robot/state --once
ros2 topic echo /robot/limb/right/endpoint_state --once
ros2 topic hz /robot/joint_states
ros2 run sawyer_ros2_control command read
```

Terminal 3 — optional ROS 1 diagnostics:

```bash
cd ~/sawyer_rs
pixi run ros1-shell
# Run these commands inside the shell that opens:
rostopic info /robot/limb/right/joint_command
rostopic echo -n 1 /robot/state
```

Enter the appropriate shell once per terminal, then run ordinary ROS commands
without prefixing each command with `scripts/env.sh`. Shell tasks automatically
load `config/network.env` and workspace overlays. Type `exit` to return to the
original terminal. The prompt identifies the selected environment. Do not source
`/opt/ros/humble` inside these shells. The environment wrapper remains available
for scripts and isolated one-shot commands.

The new bridge uses the ROS 1 node name `/sawyer_selective_bridge`, so it does
not replace the existing `/ros_bridge` process. Use one selected bridge instance
per ROS graph. `config/bridge.yaml` and the legacy instructions in
`0_DAILY_STARTUP.md` remain available for the original joint-state bridge.

## Selected topics and QoS

`config/sawyer_bridge.yaml` enables only joint states, assembly state, endpoint
state, collision state, joint limits, and head state. Command topics and optional controls
are disabled by default. Edit `enabled` on individual entries to select them;
restart the bridge after configuration changes. No dynamic all-topics bridge is
started by these scripts.

The command direction is strictly ROS 2 → ROS 1; telemetry is ROS 1 → ROS 2.
Sensor telemetry uses best-effort keep-last queues. State and collision use
reliable depth 1. Limits use reliable transient-local publication to preserve
the ROS 1 latched information. Commands use reliable volatile depth 1 and a
200 ms DDS lifespan to limit old queued setpoints. The extension applies command
QoS to the ROS 2 subscriber. DDS and ordinary Python scheduling do not provide
hard real-time guarantees.

The three `parameter_bridge` positional arguments name ROS **1 parameters**.
The startup script loads its configuration under `/sawyer_bridge`, preserving
the original root `/topics` configuration. See [docs/INTERFACES.md](docs/INTERFACES.md)
for exact syntax, service direction conventions, endpoint types, and mappings.

## Local validation before commanding Sawyer

```bash
cd ~/sawyer_rs
./scripts/test_local.sh
./scripts/env.sh ros2 python -m unittest discover -s tests -v
./scripts/test_native.sh
```

The transport test owns a separate localhost ROS 1 master on port **11421**,
DDS domain **87**, and `ROS_LOCALHOST_ONLY=1`. It sends 200 fully populated ROS 2
`JointCommand` messages to `/sawyer_bridge_test/joint_command` and checks exact
ROS 1 field equality, including timestamp and frame_id. It never connects that
topic to Sawyer. It also tests telemetry, QoS, one-way graph direction, FK/IK/IO
mock services, and the command application's publication-free dry run.
It refuses to use an existing master on its test port and cleans up only its
own process groups. A successful run records `docs/results/local_transport.json`.
The native C++ regression check verifies exact ROS 1 wire bytes and round trips
for empty arrays, boolean arrays and JointCommand. It reuses the built bridge's
link command to preserve the established ROS 1/2 library selections.

Read-only physical verification:

```bash
source config/network.env
./scripts/test_hardware_readonly.sh
```

This temporary selected bridge verifies actual telemetry, head/gripper status
and FK/IK queries. It never creates a JointCommand, head-command, enable or
gripper-command publisher. Do not run it concurrently with another
`/sawyer_selective_bridge` instance; stop your selected bridge first. The original
legacy bridge can remain running. This test cleans up its temporary bridge.

## Operator-approved position and trajectory control

For a single arm joint or the head, use `scripts/move_joint.py`, described in
[docs/SINGLE_JOINT_CONTROL.md](docs/SINGLE_JOINT_CONTROL.md). It accepts a signed
relative distance or an absolute joint-state angle and a trajectory time. In a
`pixi run ros2-shell` terminal, preview an arm or head move:

```bash
python scripts/move_joint.py --joint right_j0 --distance 0.01 --time 5
python scripts/move_joint.py --joint head_pan --distance -0.01 --time 5
```

Both commands are publication-free previews. Connect the required command paths
after a successful readiness check with `pixi run bridge-start --commands` for
the arm or `pixi run bridge-start --head-commands` for the head. Physical execution
requires `--execute --operator-approved`. Head state is part of the default
read-only bridge configuration. Both arm and head moves retain the all-arm
readiness/limit checks; an out-of-limit arm configuration also blocks head motion.

Physical commands were **not executed** during development. A first movement
requires explicit operator approval and a verified safe workspace. The app
never enables Sawyer automatically. Normal robot safety mechanisms remain active.

Before allowing motion, inspect current readiness and limits:

```bash
source config/network.env
./scripts/sawyer.sh --require-ready
```

At the initial inspection `right_j6` was measured slightly below the controller's
lower limit. The application correctly refuses motion when the starting
configuration is outside verified limits. Resolve and recheck that condition
using normal operator procedures; no limit widening is implemented.

After local tests pass and the operator has confirmed readiness, replace the
read-only selected bridge in Terminal 1 with:

```bash
source config/network.env
./scripts/start_bridge.sh --commands
```

This connects only the approved-direction JointCommand, timeout and speed-ratio
topics in addition to telemetry. Starting the bridge sends no joint commands.
It requires the isolated test result and a live successful robot preflight.

Terminal 2 — read current values and plan a small change without publication:

```bash
source config/network.env
./scripts/control.sh read
# Replace TARGET with a verified absolute joint angle near the current value:
./scripts/control.sh one --joint right_j0 --position TARGET --duration 5
```

Only after explicit operator approval, execute that reviewed target:

```bash
./scripts/control.sh one --joint right_j0 --position TARGET --duration 5 \
  --execute --operator-approved --report docs/results/first_motion.json
```

For a full configuration, create a target file from actual current telemetry
and change only the intended values before reviewing the dry run:

```bash
./scripts/control.sh read > .runtime/target.json
./scripts/control.sh configuration --target .runtime/target.json --duration 5
./scripts/control.sh quintic --target .runtime/target.json --duration 5
# Approved execution of the reviewed trajectory:
./scripts/control.sh quintic --target .runtime/target.json --duration 5 \
  --execute --operator-approved --report docs/results/trajectory.json
```

All motion operations use a quintic trajectory with zero initial/final velocity
and acceleration, streamed as **POSITION_MODE** at 100 Hz. `one` holds all other
arm joints at their starting positions; full configurations require exactly
`right_j0`–`right_j6`. Initial displacement is capped at **0.02 rad per joint**,
setpoint speed at **0.1 rad/s**, acceleration at **0.2 rad/s²**, and position-mode
speed ratio at **0.05**. Duration must satisfy analytic derivative extrema;
increase duration when planning is rejected. Bounds are the intersection of
physical URDF limits, live controller limits and application caps. Live
acceleration limits are required, because the URDF does not provide them.

The trajectory is `q=q0+(q1-q0)(10u³−15u⁴+6u⁵)`, with derivatives computed
analytically. Velocity and acceleration are validated and available in dry-run
output; physical POSITION_MODE messages carry positions only. This avoids the
SDK's TRAJECTORY_MODE behavior that bypasses collision-avoidance modifications.

Each command run performs a fresh ROS 1 preflight and verifies its master
address, subscriber connectivity, prior isolated test, fresh ROS 2 telemetry,
stationary starting arm, homing/enable/readiness, e-stop/error state, collision
state, joint bounds and command bridge discovery. It aborts on stale feedback,
changed limits, excessive tracking error, controller velocity-limit violation,
missed scheduling deadlines or failure to settle. A ROS 1 subscription on the
actual command topic is verified read-only; no test command is sent there.

Press **Ctrl+C** in the command terminal to cease publication. On completion or
interruption the app attempts one measured-position hold only when current
safety data remains valid. With stale/colliding/invalid feedback it sends no
further commands. POSITION_MODE and the SDK timeout are not an emergency-stop
guarantee: a previously issued position setpoint may remain active. Use the
robot's normal stop controls for an emergency. The conservative speed ratio
persists on the controller, matching SDK behavior. It is not raised automatically.

## Supported interfaces and validation results

The full [interface inventory](docs/INTERFACES.md) distinguishes converters from
working functionality. Current evidence is stored under `docs/results`:

| Work | Status |
|---|---|
| Original ROS 1 messages/services/actions | Built; all 51 source interfaces and generated wrappers load/serialize; source hashes unchanged |
| ROS 2 messages/services/actions | Built; all 51 interfaces load C type support and serialize; C/C++ libraries generated |
| Directional bridge and Sawyer converters | Built and tested: all 46 ordinary Sawyer messages and 3 services, plus standard JointState; `bridge_pairs.txt` |
| Isolated ROS 2 → ROS 1 JointCommand | Tested: 200/200 messages, identical fields, zero observed loss; `local_transport.json` |
| Actual Sawyer telemetry, head/gripper monitoring, FK/IK | Hardware read-only tested: 100.02 Hz ROS 2 arm states; valid FK and IK responses; `hardware_readonly.json` |
| Joint-control safety and planning | Eleven unit tests, simulated arm/head execution, native C++ serializer checks and localhost dry run; no hardware motion |
| Physical small joint move and trajectory tracking | Pending explicit operator approval and valid starting configuration |
| Head quintic command reference | Implemented; relative/absolute execution tested against an isolated ROS 1 fixture through ros1_bridge; physical motion pending |
| Enable/disable, gripper actuation | Types/configuration available; hardware actuation not tested or automated |
| Cartesian motion application | Not implemented |
| Intera and FollowJointTrajectory ROS 1 actions | Action protocol unsupported by this ros1_bridge; ROS 1 SDK server required for FollowJointTrajectory |

Local performance measures separate publish/receive frequency, sequence gaps,
inter-arrival jitter, and actual timestamped forwarding/ack timing. The local
publisher and receiver share the workstation clock. Physical read-only tests
correlate ROS 1 and ROS 2 timestamp/name identities in a common window and report
receiver frequencies. Controller-to-workstation latency is **not** inferred
from frequency or reported without verified clock synchronization. These are
observed samples, not real-time guarantees or proof of end-to-end controller loss.

During the physical test, ROS 1 arm reception was 100.00 Hz and ROS 2 reception
was 100.02 Hz. All 856 unique arm timestamp/name identities in the common window
matched, with zero observed mismatches. ROS 2 inter-arrival standard deviation
was 0.184 ms. On the isolated workstation test, command forwarding averaged
0.458 ms (p95 0.743 ms); command/ack round-trip averaged 0.900 ms. These timing
values apply to this local test, not physical robot command latency. See
[docs/VALIDATION.md](docs/VALIDATION.md) for the milestone record and limitations.

## Troubleshooting

* Python/NumPy import errors: use `scripts/env.sh`; do not combine `/opt/ros/humble`
  Python 3.10 paths with Pixi Python 3.12. `system-ros2` exists for isolated system
  CLI diagnostics. Wrapper shells skip user startup files and user site packages.
* `em.RAW_OPT` missing: build through `scripts/build.sh`; catkin must use
  `scripts/python_noetic.sh`, which selects project-local EmPy 3.
* Custom type missing from bridge: verify both workspaces, run `--print-pairs`,
  then rebuild only `ros1_bridge`. A sourced overlay at runtime cannot add a
  converter to an already compiled bridge.
* `cpp_common`/`xmlrpcpp` CMake failure: use the mixed bridge wrapper, which supplies
  Noetic CMake dependencies after ROS 2 prefixes and supplies ROS 1 pkg-config paths.
* CMake rejects old minimum version: the build script passes
  `CMAKE_POLICY_VERSION_MINIMUM=3.5`; retained upstream deprecation warnings are expected.
* ROS 2 receives no data: check `ROS_DOMAIN_ID` in both terminals, network routes,
  ROS 1 publisher types, enabled configuration entries and QoS. Use explicit
  sensor-data QoS for custom subscribers to best-effort telemetry.
* ROS 1 discovery works but subscriptions fail: verify controller-to-workstation
  connectivity to the advertised `ROS_IP`; unset conflicting `ROS_HOSTNAME`.
* Motion refused: inspect the printed reason. Fix readiness, stale data, measured
  limits or reviewed duration before retrying. Do not bypass checks or widen limits.
* FK/IK service appears but fails: inspect server availability and returned
  `is_valid`/`in_collision`/`result_type`. A converter or service advertisement
  alone does not prove a usable solution.
* Actions: native ROS 2 action definitions do not convert ROS 1 actionlib's
  protocol. Use the guarded position trajectory example for initial control.

Upstream bridge reference: [ros2/ros1_bridge](https://github.com/ros2/ros1_bridge).
