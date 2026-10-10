# Validation record — 2026-10-09

| Milestone | Outcome and evidence |
|---|---|
| 1. ROS 1 custom interfaces | PASS: original core and motion packages built with catkin; all 51 source interfaces plus generated action wrappers serialize; original definition SHA256 values unchanged |
| 2. ROS 2 custom interfaces | PASS: both interface packages built with colcon; all 51 messages/services/actions load compiled Python type support and round-trip serialization; generated C and C++ libraries built |
| 3. Sawyer bridge converters | PASS: executable `--print-pairs` includes all 46 ordinary Sawyer messages and three services; original standard JointState mapping retained |
| 4. Selective bridging | PASS: one bridge process per graph, state only 1→2 and isolated commands only 2→1; graph checks confirm no reverse topic endpoints; disabled entry creates no endpoints |
| 5. Harmless command forwarding | PASS: 200 of 200 fully populated JointCommand messages received in ROS 1 with identical mode, joint names, all four numeric arrays, stamp and frame_id |
| 6. Guarded application | PASS for local planning and checks: publication-free dry run, approval-gate rejection, six safety tests, hardware dry-run rejection on out-of-limit measured joint |
| 7. Physical motion readiness | BLOCKED: operator approval absent and current right_j6 measurement is outside verified limits; no physical commands published |
| 8. Trajectory and other interfaces | Quintic planning and derivatives tested locally; actual ROS 2 state, endpoint, collision, limits, head, gripper configuration/status, FK and IK verified; physical trajectory tracking and additional actuation pending |

The hardware is reachable at `169.254.121.3` through workstation interface
`enp0s31f6`, address `169.254.121.2/16`. The robot's JointCommand subscriber is
`/realtime_loop`, with node URI `http://169.254.121.3:60599/` at inspection.
Its own ROS 1 subscription list confirms `intera_core_msgs/JointCommand`.
These runtime port numbers are observations and are not hardcoded into scripts.

Actual ROS 2 telemetry verified:

* `/robot/joint_states`
* `/robot/state`
* `/robot/limb/right/endpoint_state`
* `/robot/limb/right/collision_detection_state`
* `/robot/joint_limits`
* `/robot/head/head_state`
* `/io/end_effector/config`, identifying `right_gripper`
* `/io/end_effector/right_gripper/state`

ROS 2 FK returned `is_valid=[true]`, `in_collision=[false]`, one pose. ROS 2 IK
on the current FK pose returned `result_type=[1]` and one solution. The
physical controller provides `/motion/motion_command/goal` with
`intera_motion_msgs/MotionCommandActionGoal`. A ROS 2 action server was not
created: this ros1_bridge checkout does not implement action protocol bridging.
The SDK supplies a ROS 1 FollowJointTrajectory server; it was not launched.

The robot reported homed/enabled/ready, no e-stop, no error and no collision.
However, measured `right_j6=-4.714662109375` rad is outside its live and physical
URDF lower bound `-4.7124` rad by approximately 0.002262 rad. The command
application refused to plan or publish from that configuration. No limit
tolerance, widening or safety override was added. A normal operator procedure
must resolve the starting configuration and the operator must approve the first
physical movement before hardware command/tracking tests.

## Measurements

| Quantity | Observed result |
|---|---|
| Local ROS 1 joint source | 100.003 Hz |
| Local ROS 2 joint reception | 100.066 Hz |
| Local joint sequence gaps | 0 in 221 received messages |
| Local ROS 2 inter-arrival jitter stddev | 0.973 ms |
| Local command messages received | 200/200, identical fields |
| Local command forwarding time | mean 0.458 ms, p95 0.743 ms, max 1.388 ms |
| Local command/ack round-trip | mean 0.900 ms, p95 1.388 ms, max 3.435 ms |
| Physical ROS 1 arm reception | 100.000 Hz |
| Physical ROS 2 arm reception | 100.022 Hz |
| Physical ROS 2 arm inter-arrival jitter stddev | 0.184 ms |
| Common-window unique arm sample identities | 856 in each receiver, zero observed mismatches |

Local forwarding timing uses source and receiver timestamps from the same
workstation CLOCK_REALTIME; it includes serialization and scheduling overhead.
Round-trip timing includes ROS 1 acknowledgment generation and the return
bridge. Hardware frequency uses each receiver's monotonic clock. Hardware
sample comparison matches header stamp plus joint-name identities within the
common timestamp window; it is receiver-to-receiver correlation, not a proof of
controller-to-bridge loss. No workstation/controller clock synchronization was
established, so no physical one-way command or telemetry latency is reported.
No physical command round trip or trajectory tracking error was measured.

## Environment and build findings

The Noetic and Humble Pixi interpreters are both Python 3.12.15 and import NumPy
from their respective environment prefixes. The system wrapper uses
`/usr/bin/python3` (3.10), system NumPy, and `/opt/ros/humble` rclpy. Tests with a
deliberately inherited Pixi PYTHONPATH confirmed the system wrapper removes
all Pixi/Python 3.12 paths; Pixi wrappers contain no `/opt/ros`/Python 3.10 paths.
`ros2 --help` succeeds in both modes.

The locked EmPy 4.2.1 cannot generate Noetic C++ messages (`em.RAW_OPT` missing).
A Pixi dependency-pin attempt failed because package-index DNS was unavailable.
The project-local unmodified EmPy 3.3.4 source resolves this under Python 3.12
without changing installed environments or `pixi.lock`. ROS 2 interface parsing
also required removing leading indentation from ROS 1 constant declarations and
renaming invalid field names with explicit bridge mappings.

Bridge compiler failures for `vector<bool>` and FK response arrays were corrected
in the template; service-field rename rules and builtin time conversion were
implemented in the generator. These were validated by successful converter
compilation, FK bool-array responses, IO time round trips and exact command
forwarding. The standalone native C++ regression check also compares actual
converted ROS 1 wire bytes with native ROS 1 serialization and round-trips
JointCommand and InteractionControlCommand with empty/nonempty boolean and
numeric arrays. It passed. The harness reuses the bridge's validated linker
command because several ROS 1/2 libraries share sonames; independently resolving
the exported dependencies can select the wrong TF/class-loader libraries.
All source changes are preserved in the reproducible bridge patch.

Retained non-fatal build diagnostics:

* CMake deprecation warnings from upstream ROS 1 minimum versions and bridge CMake.
* CMP0148 developer warnings from old catkin/Humble Python CMake discovery.
* The upstream ament Python bytecode-install step prints `Can't list
  'lib/python3.12/site-packages/ros1_bridge'` because of its relative path. The
  installed `ros1_bridge` Python module imports successfully, converter
  generation completes, and the compiled executables pass integration tests;
  this diagnostic affects optional bytecode precompilation.

The original installed bridge and `config/bridge.yaml` remain intact. A final
read-only XML-RPC check confirmed `/ros_bridge` was still alive at
`http://169.254.121.2:40215/`, PID 370597, and no temporary
`/sawyer_selective_bridge` node remained after test cleanup.

Evidence files: `results/build_ros1.log`, `results/build_ros2.log`,
`results/build_bridge.log`, `results/bridge_pairs.txt`,
`results/ros2_interfaces.json`, `results/unit_tests.log`,
`results/native_tests.log`,
`results/local_transport.json`, `results/local_test_console.log`,
`results/hardware_preflight.json`, `results/hardware_readonly.json`,
`results/hardware_command_dry_run.log`, and raw receiver samples under `results`.
