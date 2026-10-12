# Sawyer ROS 1 ↔ ROS 2 workstation bridge

Sawyer's controller remains ROS 1. This project uses the existing Pixi Noetic
and Humble installations, original Sawyer message sources, and `ros1_bridge` to
expose selected telemetry and joint-position commands to ROS 2.

For an explanation of where each component runs and how commands and feedback
travel between environments, read [How our setup works](docs/HOW_THE_SETUP_WORKS.md).

No Docker, custom transport gateway, ROS reinstallation, or onboard change is
required. The previously working bridge installation and joint-state-only
configuration remain available.

```text
Sawyer's original ROS 1 controller and master
    ↕ ROS 1 Ethernet (direct connection to the bridge)
C++ ros1_bridge (Pixi Humble + Noetic libraries + both interface workspaces)
    ↕ ROS 2 DDS
Python sawyer_ros2_control / move_joint.py (Pixi Humble)

Optional ROS 1 diagnostic tools run separately in Pixi Noetic.
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

## Move one joint (arm or head)

Running `scripts/move_joint.py` starts the movement immediately. Select a joint,
a relative distance or absolute position, and a trajectory time. Angles are
**radians** and time is **seconds**.

### Terminal 1: start the bridge

```bash
cd ~/sawyer_rs
pixi run bridge-start --commands --head-commands
```

Leave it running. If the selected bridge is already running, stop it with
Ctrl+C before restarting with these command paths. For arm-only control use
`--commands`; for head-only control use `--head-commands`.

### Terminal 2: enter ROS 2 once

```bash
cd ~/sawyer_rs
pixi run ros2-shell
```

Inside this shell, read current joint positions if needed:

```bash
ros2 topic echo /robot/joint_states --once --qos-reliability best_effort
```

### Move by a relative distance

Choose `right_j0`, `right_j1`, `right_j2`, `right_j3`, `right_j4`, `right_j5`,
`right_j6`, or `head_pan`:

```bash
JOINT=right_j0
DISTANCE=0.01
DURATION=5
python scripts/move_joint.py --joint "$JOINT" --distance "$DISTANCE" --time "$DURATION"
```

This commands `right_j0` by +0.01 rad over a five-second reference. Negative
distance moves in the opposite joint-coordinate direction. Each run calculates
its target from the current position, so repeating it adds another displacement.

For the head:

```bash
python scripts/move_joint.py --joint head_pan --distance -0.01 --time 5
```

### Move to an absolute position

Replace `0.10` with your desired joint-state angle:

```bash
python scripts/move_joint.py --joint right_j0 --position 0.10 --time 5
```

Use `--position` or `--distance`, exactly one at a time. `--duration` is an alias
for `--time`.

Both angle arguments accept `pi` and arithmetic expressions in radians:

```bash
python scripts/move_joint.py --joint right_j0 --position "pi/2" --time 5
python scripts/move_joint.py --joint right_j0 --position -pi/2 --time 5
python scripts/move_joint.py --joint right_j0 --distance "pi/12" --time 5
```

Both `--position -pi/2` and `--position=-pi/2` work. Quote expressions containing
spaces, for example `--position "-(pi / 2)"`. Numbers, `pi`, parentheses and `+ - * /` are
supported; functions and Python code are not accepted. Joint limits and minimum
trajectory time are checked after converting the expression to radians.

To move to the selected joint's limits:

```bash
# Lower joint-position limit.
python scripts/move_joint.py --position -end --time 5 --joint right_j5

# Upper joint-position limit.
python scripts/move_joint.py --position +end --time 5 --joint right_j5
```

`-end` selects the lower limit and `+end` the upper limit from the intersection
of current controller and URDF limits. The resolved radians and limit range are
printed before execution. These options apply to `--position`; `--distance`
remains a relative angle. Five seconds must still meet the calculated minimum
time; if it does not, the script prints the required duration before commands
are sent. Add `--dry-run` to preview the limit target and minimum time.

Small measured overshoot at a limit is handled separately from command limits.
Feedback up to **0.008726646 rad (0.5°)** outside a boundary is accepted; larger
excursions still stop the application. The raw measured angle is retained in
reports. For example, `right_j6 = -4.714042` with lower limit `-4.7124` counts
as already at `-end`, and a command to `+end` can move back through the allowed
range. Its quintic reference starts at the boundary; every published setpoint
and requested target remains strictly within limits. Reports include
`reference_start_rad` and `limit_overshoot_rad`.

For absolute zero:

```bash
python scripts/move_joint.py --joint right_j0 --position 0.0 --time 5
```

The absolute target remains **0.0 rad**; the quintic reference moves from the
measured starting angle to that target. The script prints progress once per
second and finishes at the requested trajectory time, with no extra settling
wait by default. Fresh measured feedback within **0.008726646 rad (0.5°)** counts
as arrival, matching the original Intera SDK's arm tolerance. A residual such as
0.004249 rad therefore succeeds and is still reported accurately. Repeating an
absolute command already within tolerance sends no commands. A larger residual
still returns `status: not_reached` at the deadline.

Optional `--tolerance` changes the arrival tolerance, and `--settle-timeout`
explicitly allows extra settling time. For example, to request a stricter 0.002
rad tolerance and allow up to two extra seconds:

```bash
python scripts/move_joint.py --joint right_j1 --position 0.0 --time 5 --tolerance 0.002 --settle-timeout 2
```

### Minimum trajectory time

For either an absolute target or a relative distance, the script calculates the
minimum feasible time for its quintic curve using the selected joint's live
velocity/acceleration limits. Velocity is also bounded by the URDF value.

```text
distance = abs(target - measured_start)
T_velocity = 1.875 * distance / velocity_limit
T_acceleration = sqrt((10 / sqrt(3)) * distance / acceleration_limit)
T_minimum = max(T_velocity, T_acceleration)
```

The entered `--time` must be **at least the calculated minimum**. A shorter time
prints the minimum and a suggested `--time`, then exits without sending commands.
It does not automatically change your requested duration. If live acceleration
data is unavailable, it uses the velocity bound and explicitly labels the result
as velocity-only.

To calculate and inspect a plan without movement:

```bash
python scripts/move_joint.py --joint right_j1 --position 0.0 --time 5 --dry-run
```

The same check applies to `--distance`. Accepted plans print
`Minimum quintic time: ... s`; saved reports include `minimum_time_seconds`.

### Optional preview and result file

Add `--dry-run` only when you want a preview without movement:

```bash
python scripts/move_joint.py --joint right_j0 --distance 0.01 --time 5 --dry-run
```

To save the measured result while executing:

```bash
python scripts/move_joint.py --joint right_j0 --distance 0.01 --time 5 \
  --report docs/results/joint_motion.json
```

### What the script does

The single Python script reads the selected joint position and its live limits,
intersects the position range with the Sawyer URDF range, and checks the starting
position, target and each published setpoint. Out-of-limit positions are rejected.
It generates a quintic position reference at 100 Hz with zero starting/ending
velocity and acceleration. Arm commands use `JointCommand.POSITION_MODE` and
contain only the selected joint, following the original SDK's partial joint
command behavior. Head commands use `HeadPanCommand.SET_ACTIVE_MODE` with speed
ratio 1.0. Arm execution also sets the SDK position-control speed ratio to **1.0**
before the reference starts, instead of inheriting an earlier slow ratio such as
0.05. This setting persists on the controller. Physical movement remains subject
to the robot's normal velocity limits and controller behavior.

The application checks the selected joint's position range and whether the
requested duration satisfies the quintic velocity/acceleration bounds. It has no
extra displacement, readiness, collision or tracking gates,
and it makes no ROS 1 preflight or Bash subprocess calls. Other joints do not
block the selected joint's limit check. Sawyer's onboard safety mechanisms remain
active; the robot must be enabled through its normal operator interface. Measured
arrival is checked at the reference deadline using fresh selected-joint feedback,
or during optional extra settling. Duplicate or older timestamped feedback is ignored.

Press **Ctrl+C** to stop publishing. The controller may retain the last position
target; use the robot's normal stop controls for an emergency. The optional
`--dry-run` is the only preview flag needed. For all arguments:

```bash
python scripts/move_joint.py --help
```

For head commands, the result also includes `head_state` with `pan_mode`,
`is_turning`, `is_blocked` and feedback age. A failed move with fresh
`pan_mode: 0` means ACTIVE_MODE was not confirmed by the controller. To inspect
head mode while the command application is running, use another ROS 2 shell:

```bash
ros2 topic echo /robot/head/head_state --once
```

The [Intera Head API](https://rethinkrobotics.github.io/intera_sdk_docs/5.1.0/intera_interface/html/intera_interface.head.Head-class.html)
defines mode 0 as passive and mode 1 as active position control. Successful
message forwarding alone does not establish that the controller activated its
head motor.

If the head stays passive, inspect what reaches ROS 1 in another terminal:

```bash
cd ~/sawyer_rs
pixi run ros1-shell
rostopic echo /robot/head/command_head_pan
```

To capture one command and one head-state message while the command publisher
keeps running, use ROS 1's `-n 1` option:

```bash
rostopic echo -n 1 /robot/head/command_head_pan
rostopic echo -n 1 /robot/head/head_state
```

This command only observes messages. During an operator-run head command,
expect changing `target`, `speed_ratio: 1.0` and `pan_mode: 1`. No output means
the command path needs checking. Correct incoming fields with fresh passive
head feedback require controller-side diagnosis; increasing trajectory time
alone does not resolve mode acceptance.

The older full-arm `command` application remains separate and retains its guarded
execution workflow. Details of the simplified single-joint script are in
[docs/SINGLE_JOINT_CONTROL.md](docs/SINGLE_JOINT_CONTROL.md).

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
| Joint-control tests | Nineteen unit tests covering head-mode diagnostics, minimum-time feasibility, direct single-joint motion, measured arrival and the separate guarded full-arm application |
| Arm quintic command reference | Direct relative/absolute execution through ros1_bridge; only the selected joint is commanded; position-limit rejection tested locally |
| Physical position movement | Operator reported movement with a large remaining error in the previous script; fixed arrival behavior awaits hardware validation |
| Head quintic command reference | Implemented; relative/absolute execution tested against an isolated ROS 1 fixture through ros1_bridge; physical motion pending |
| Enable/disable, gripper actuation | Types/configuration available; hardware actuation not tested or automated |
| Cartesian motion application | Not implemented |
| Intera and FollowJointTrajectory ROS 1 actions | Action protocol unsupported by this ros1_bridge; ROS 1 SDK server required for FollowJointTrajectory |

Latest validation (2026-10-12): local tests passed. The read-only physical
preflight could not reach `169.254.121.3:11311` (`No route to host`), so live
readiness and physical motion remain unverified. Earlier hardware observations
above refer to the 2026-10-09 session. See [the progress checkpoint](docs/PROGRESS.md).

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
* Single-joint motion error: check the selected joint's position bounds, current
  feedback and enabled command bridge. The separate full-arm application also
  checks readiness, feedback freshness and derivative limits.
* FK/IK service appears but fails: inspect server availability and returned
  `is_valid`/`in_collision`/`result_type`. A converter or service advertisement
  alone does not prove a usable solution.
* Actions: native ROS 2 action definitions do not convert ROS 1 actionlib's
  protocol. Use the position-reference application for joint-space control.

Upstream bridge reference: [ros2/ros1_bridge](https://github.com/ros2/ros1_bridge).
