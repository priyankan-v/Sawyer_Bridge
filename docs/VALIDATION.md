# Validation record

## Small measured overshoot at joint limits — 2026-10-12

The operator moved right_j6 from 1.567699 toward its lower limit -4.7124 rad.
The old script aborted when feedback reached -4.712583 and subsequently blocked
both `-end` and `+end` from measured -4.7140419921875 rad. The latter exceeds the
configured lower bound by only 0.0016419921875 rad.

Measured feedback now has a fixed 0.008726646 rad (0.5 degree) boundary allowance,
independent of the configurable arrival tolerance. Commands and requested targets
still obey exact limits. An allowed out-of-range starting measurement uses the
in-range boundary as the quintic reference start. Relative target calculation
still uses raw measured position; measured feedback/error is never clamped or
rewritten. Reports include `reference_start_rad` and `limit_overshoot_rad`.
Larger measured excursions, out-of-range targets and stale/malformed feedback
still fail. Repeating an already reached `-end` sends no commands.

Thirty-four unit tests passed, including the operator's exact overshoot,
completion at a limit, repeated endpoint no-op, recovery in both directions,
strict command limits, independent allowance and relative recovery semantics.
Evidence: `docs/results/limit_feedback_unit_tests.log`. No physical robot
commands were sent by the assistant.

The full isolated bridge suite passed with an injected -0.0016419921875 rad
sensor offset. The private fixture's lower target -1 rad completed with measured
-1.0016419913 rad, repeating `-end` produced zero commands, and `+end` recovered
from measured -1.0016419922 rad using reference start -1 rad. Every ROS 1 command
remained in [-1, 1]. Existing exact-field forwarding, minimum-time/limit rejection
and guarded collision tests remained passing. Evidence:
`docs/results/limit_feedback_local_tests.log`, `local_arm_lower_limit_overshoot.json`,
`local_repeated_limit_overshoot.json` and `local_arm_limit_recovery.json`.

## Joint-limit endpoint targets — 2026-10-12

`--position -end` selects the joint's lower controller/URDF limit and
`--position +end` selects its upper limit. Both space-separated and equals-sign
forms work. Targets resolve after live limits arrive, and are re-evaluated from
current limits before trajectory execution. Relative distance remains numeric
or a pi expression. Minimum quintic duration is still enforced before commands.

The quintic returns the exact final endpoint and keeps interpolation inside its
start/target range. Head endpoints round inward to a representable float32 value
when necessary so serialization cannot push a limit target outside bounds.

Thirty unit tests passed: argument forms, distance-marker rejection, controller/
URDF intersections, changed limits, previews without publication, both endpoint
executions, repeated-endpoint no-op, too-short duration rejection, and inward
head rounding. Evidence: `docs/results/joint_end_unit_tests.log`. These tests
use simulated feedback and do not establish hardware tracking at a joint limit.

The isolated bridge suite also passed. `-end` reached the fixture's lower limit
(-1 rad) in a five-second reference, and `+end` reached its upper limit (+1 rad)
in an eight-second reference from -1 rad. ROS 1 captured exact final endpoint
commands, only the selected joint was commanded, and every setpoint stayed
within the fixture limits. Existing transport, minimum-time rejection, repeated
zero and collision checks remained passing. Evidence:
`docs/results/joint_end_local_tests.log`, `local_arm_lower_limit.json` and
`local_arm_upper_limit.json`. No physical robot commands were sent.

## Pi expressions for joint angles — 2026-10-12

`--position` and `--distance` now accept numbers, `pi`, parentheses and basic
arithmetic (`+ - * /`). Expressions are parsed using a restricted AST interpreter;
function calls, attribute access and arbitrary Python execution are unsupported.
Negative expressions work in both forms: `--position -pi/2` or
`--position=-pi/2`, and similarly for `--distance`. Angle option values are
attached before argparse parsing so leading minus signs are accepted; genuine
options remain options. Joint limits and minimum quintic time are
checked against the evaluated radians.

Twenty-four unit tests passed, including absolute/relative pi expressions,
rejection of invalid arithmetic/code and out-of-limit pi targets before any
command publisher is created. Evidence: `docs/results/pi_angle_unit_tests.log`.
No physical commands were sent; the change affects argument parsing only.

After the operator reported rejection of `--position -pi/2` (quoted or unquoted),
space-separated negative expressions were added. Twenty-six unit tests passed,
including the exact CLI argument order, real sys.argv parsing, negative relative
values, invalid arithmetic and preserved options. A parsing-only command in the
ROS 2 environment also verified right_j2, position -1.5707963267948966 rad and
duration 5 seconds without initializing a node or publishing commands.
Evidence: `docs/results/negative_pi_cli_unit_tests.log`.

## Finish at requested time and accept SDK arrival tolerance — 2026-10-12

The operator's right_j1 command started at -0.085778 rad, targeted zero over five
seconds, and ended near -0.004249 rad. The previous 0.002 rad arrival tolerance
kept the application publishing for 15 extra seconds and then reported failure.

The single-joint script now uses the original Intera SDK's arm arrival tolerance,
0.008726646 rad (0.5 degrees). Extra settling defaults to zero, so execution
finishes at the first publication tick at/after the requested trajectory duration.
It prints progress once per second. Reports retain the actual measured error
and include `tolerance_rad`. Repeated absolute targets already within tolerance
publish nothing. A larger missed target still fails at the deadline; optional
`--tolerance` and `--settle-timeout` allow stricter/longer settling when desired.
Position limits, minimum quintic time and stale-feedback checks are preserved.

Twenty-one unit tests passed. Regression cases reproduce the operator's exact
residual with five-second completion, verify repeated zero sends no commands,
and verify blocked/strict-tolerance failures have no default extra wait. The
existing slow-controller case now explicitly requests additional settling.
The isolated bridge regression suite also passed, including 200/200 exact-field
command forwarding, direct relative/absolute execution, minimum-time rejection,
repeated-zero no-op and limit rejection. A 1.1-second fixture trajectory completed
in 1.104857 seconds with measured error approximately 1.08e-7 rad. These are local
fixture results, not hardware tracking results.
Evidence: `docs/results/on_time_completion_unit_tests.log`,
`docs/results/on_time_completion_local_tests.log` and
`docs/results/local_absolute_zero.json`. No physical robot
commands were sent; head activation diagnosis remains deferred by the operator.

## Head not moving: read-only diagnosis — 2026-10-12

The operator's head command reported target zero, start/measured angle
-2.982996 rad, and `not_reached` after 20 seconds. Read-only inspection found:

- Robot assembly enabled, ready and homed, with no e-stop/error flags.
- `/realtime_loop` subscribes to the expected HeadPanCommand type; the TCPROS
  connection to the selected bridge is established.
- Bridge and controller connection counters agree on 4,794 received head command
  frames. No new head commands were sent during these inspections.
- ROS 1 HeadState and ROS 2 HeadState agree: pan approximately -2.982996,
  passive mode 0, not turning and not blocked. JointState agrees with this angle.
- An additional diagnostic warning reported a right_j1 collision. Its relationship
  to the head behavior has not been established.
- Head homing state is HOMED (0). Controller firmware is 5.3.1; the original
  SDK's compatibility table explicitly supports it with SDK 5.3.0.
- The operator's subsequent result captured fresh HeadState at failed arrival:
  pan_mode 0, unchanged angle, not turning and not blocked. Passive mode is
  therefore confirmed at failed arrival, rather than only after a separate check.

These observations do not prove the fields of ROS 1 commands during the
operator's run. The precise reason for not activating the head remains unknown.
The next diagnostic is a read-only ROS 1 echo of `/robot/head/command_head_pan`
while the operator runs the application: target should follow the trajectory,
speed_ratio should be 1.0, and pan_mode should be 1. If those fields arrive while
fresh head feedback stays passive, investigate controller acceptance rather
than changing the quintic duration or joint-state indexing.

A later five-second live read-only capture, after the operator posted passive
feedback from the direct ROS 1 test, observed enabled/ready/homed assembly state,
passive head feedback and zero command messages. Only the selective bridge was
registered as a command publisher; no direct `rostopic pub` node was present.
This capture cannot establish acceptance during that earlier test. Keep its
publisher running while observing both incoming command fields and head state.
Evidence: `docs/results/head_direct_ros1_readonly.json`.

The operator then captured a direct ROS 1 command with target -2.982996,
speed_ratio 0.1 and pan_mode 1 while feedback remained passive. A subsequent
five-second live read-only capture independently observed these command fields
from the `rostopic` publisher. Controller connection 120 was established and its
receive count increased from 633 to 685 (52 messages). No command from the
registered bridge publisher was observed during this capture. Assembly remained
enabled/ready/homed; no warnings or diagnostic entries were captured. This
confirms the direct ROS 1 command transport works while active mode acceptance
remains unsuccessful. It does not identify the internal controller cause.
Evidence: `docs/results/head_active_request_readonly.json`.
The script now includes fresh HeadState fields in result reports and prints a
specific passive-mode diagnostic at failed arrival. No new motion gate was added.
Nineteen unit tests passed, including a passive, unmoving head receiving mode-1
commands and reporting its mode correctly.

Evidence: `head_readonly_preflight.json`, `head_readonly_diagnosis.json`,
`head_readonly_connection_stats.json`, `head_ros2_graph.txt`, `head_ros2_state.txt`,
`head_readonly_faults.json`, `head_firmware_diagnosis.json` and
`head_mode_unit_tests.log` in `docs/results/`.

## Minimum quintic time — 2026-10-12

The user requested an analytic feasibility check before executing either a
relative distance or an absolute target. For displacement `d`, the script uses
`max(1.875*abs(d)/v_max, sqrt((10/sqrt(3))*abs(d)/a_max))`. Velocity is the smaller
of live controller and URDF limits; acceleration uses live controller data when
available. Without acceleration data, it explicitly reports a velocity-only
minimum. Entered durations at or above the minimum are accepted; shorter values
print the minimum and suggested time without starting movement. The requested
duration is never automatically increased.

Eighteen unit tests and the isolated bridge suite passed. The fixture rejected a
0.05 s command with minimum 1.074570 s and zero published commands, accepted a
1.1 s command and verified arrival, and preserved repeated absolute-zero no-op
behavior. No physical robot commands were sent. Evidence is in
`docs/results/minimum_time_unit_tests.log`, `minimum_time_local_tests.log`,
`local_minimum_time_rejection.json` and `local_absolute_zero.json`.

## Absolute-position completion fix — 2026-10-12

Operator logs showed an absolute target of zero but final measured angles of
-3.103655 and -2.771718 rad after successive five-second runs. The previous
application stopped at reference expiry without checking arrival. The observed
approximately 0.064 rad/s is consistent with the previously used 0.05 controller
speed ratio (right_j1 URDF velocity is 1.328 rad/s).

The fixed script explicitly sets SDK position-control speed ratio 1.0, streams
the unchanged final target after reference expiry, and requires fresh feedback
within 0.002 rad for 0.2 seconds before success. It reports failure after the
default 15 seconds of additional settling. Already-reached absolute targets
produce no commands. Duplicate/older timestamped feedback is ignored; stopping
feedback cannot produce a successful arrival result. No approval flags were added.

Fifteen unit tests and the isolated bridge regression suite passed. A rate-limited
actuator fixture reached zero after a 0.05 s reference with additional settling;
the next absolute-zero command sent no messages. This verifies the completion
logic and bridge path in simulation. No physical commands were sent by the
assistant, and fixed physical tracking remains unverified.

Evidence: `absolute_position_unit_tests.log`, `absolute_position_local_tests.log`,
`local_absolute_zero.json` and `local_repeated_absolute_zero.json` in `docs/results/`.

## Simplified single-joint execution — 2026-10-12

At the user's request, the single-joint command now executes directly and checks
only selected-joint position bounds plus valid input/communication availability.
It has an optional `--dry-run`, with no execution/approval gates or ROS 1 preflight
calls. Command bridge startup no longer runs a preflight gate.

The Python control package built successfully. Twelve unit tests and the isolated
transport suite passed. Relative/absolute arm and head commands crossed
ros1_bridge without approval flags. A 0.05 rad arm movement was accepted without
the former displacement cap. Out-of-limit targets were rejected before command
publication. Arm messages contain only the selected joint. The collision-abort
regression now tests the separate guarded full-arm application, not this simple
single-joint script. No physical robot commands were sent.

Evidence: `build_simple_joint.log`, `simple_joint_unit_tests.log`,
`simple_joint_local_tests.log` and the updated `local_arm_*.json` /
`local_head_motion.json` reports in `docs/results/`.

## Continuation — 2026-10-12

- All eleven unit tests passed.
- The existing 200-message exact-field JointCommand forwarding check was rerun.
- Relative and absolute single-joint arm/head trajectories were tested through
  ros1_bridge against an isolated ROS 1 fixture (localhost master port 11421,
  DDS domain 87, localhost-only transport). These simulate ideal feedback and do
  not establish physical robot tracking performance.
- Arm checks verify POSITION_MODE, empty derivative/effort arrays, all seven
  names, the six unchanged setpoints, speed ratio and command timeout.
- A simulated collision after 20 commands caused exit code 1; no further
  commands arrived after the application exited.
- Preflight now checks the master's read-only API before registering a node.
  An unavailable local master returns an error report promptly.
- The physical master `169.254.121.3:11311` returned `No route to host` during
  read-only preflight. Live readiness could not be established; no physical
  commands were published. Earlier joint-limit observations remain historical.

Evidence: `docs/results/resume_2026_10_12_local_tests.log`,
`local_arm_relative.json`, `local_arm_absolute.json`,
`local_arm_collision_abort.json`, `local_head_motion.json`,
`local_unavailable_master.json` and `resume_2026_10_12_robot_preflight.json`.

## Original validation — 2026-10-09

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
