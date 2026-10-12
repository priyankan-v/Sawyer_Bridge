# Progress checkpoint — 2026-10-12

## Completed

- Documented component locations, Pixi environments, direct ROS 1 and DDS connections, command/feedback paths and normal terminals in [HOW_THE_SETUP_WORKS.md](HOW_THE_SETUP_WORKS.md), linked from README and daily startup instructions.
- Fixed minor measured limit overshoot blocking completion and recovery. The single-joint app accepts measured excursions up to 0.008726646 rad outside a limit, independently of arrival tolerance, retains raw feedback in reports, and starts any recovery reference at the in-range boundary. Targets and all published commands remain strictly in range. Thirty-four unit tests include the operator's exact right_j6 measurement and recovery in both directions.
- Added `--position -end` and `--position +end` for the selected joint's lower/upper live controller/URDF limit. The script prints the numeric target and limit range, keeps the minimum-time check, preserves exact quintic endpoints, and rounds float32 head limit targets inward when required. No physical limit movement was performed by the assistant.
- Added radians expressions (`pi`, `pi/2`, `-pi/4`, basic arithmetic) to both single-joint angle arguments. Fixed space-separated negative expressions so `--position -pi/2` works, quoted or unquoted, as does `--position=-pi/2`. Joint-limit and minimum-time checks still apply; no Python code execution is allowed. Twenty-six unit tests pass, recorded in `docs/results/negative_pi_cli_unit_tests.log`.
- Preserved the original working joint-state bridge and its build.
- Built the original ROS 1 Sawyer interfaces in `ros1_ws` without changing their definitions.
- Built compatible ROS 2 interfaces in `ros2_ws`: 51 message/service/action definitions.
- Rebuilt only the necessary bridge components in a separate `build_sawyer` / `install_sawyer` overlay. Verified converters for 46 Sawyer messages and three services, plus standard JointState.
- Added selective, directional bridging with command topics disabled by default.
- Isolated Python environments and added Pixi shell/startup tasks so ordinary `python` and `ros2` commands work after entering `pixi run ros2-shell` once.
- Added readiness/preflight checks and ROS 2 position/quintic control applications.
- Simplified `scripts/move_joint.py` at the user's request: a complete standalone Python script selecting `right_j0`–`right_j6` or `head_pan`, relative distance or absolute position, and trajectory duration. Commands execute immediately; optional `--dry-run` previews. It checks the selected joint's controller/URDF position range and the requested quintic duration. It makes no preflight or shell subprocess calls.
- Command bridge flags now select paths without invoking preflight or requiring saved test artifacts. The older full-arm application retains its own guarded workflow.
- Added documentation, startup commands, reproducible builds and validation artifacts.
- Continued arm trajectory validation through the bridge and added an unavailable-master check before ROS 1 preflight node registration.
- Fixed absolute-position completion after operator logs showed the old script stopping after five seconds with several radians of remaining error. The script sets the SDK position speed ratio to 1.0, checks measured arrival, and treats an already-reached absolute target as a publication-free no-op. Failed arrival is reported as failure. Optional tolerance/settling controls do not add approval flags.
- Added the requested minimum-time algorithm: exact quintic velocity and acceleration extrema determine the minimum feasible duration for relative or absolute moves. Too-short inputs print a suggested duration before any command publisher is created. Missing acceleration data is explicitly identified as velocity-only calculation.
- Added read-only HeadState capture to head motion result reports. An unreached target with fresh passive-mode feedback now prints that the requested active mode was not confirmed; this diagnostic does not add a motion gate.
- Removed the default 15-second settling delay after the operator reported a right_j1 residual of 0.004249 rad. The single-joint script now finishes at the requested duration, uses the original SDK arm tolerance of 0.008726646 rad (0.5 degrees), and prints progress once per second. Reports retain the measured error. Additional settling is optional, default zero; larger missed targets still fail promptly. Head diagnosis is deferred at the operator's request.

## Verified

- ROS 1 and ROS 2 interface builds and generated type support.
- Native conversion/serialization checks, including boolean arrays.
- Isolated ROS 2 → ROS 1 JointCommand forwarding: 200/200 messages with identical fields.
- Selective topic direction and disabled-command behavior.
- Thirty-four unit tests, including measured-limit overshoot completion/recovery, live-limit endpoint selection/execution, inward float32 rounding, space-separated negative pi expression parsing/rejection, the operator's exact residual finishing on time, default deadline failures, passive head diagnostics, exact minimum-time extrema, duration rejection, simulated arm/head execution, repeated absolute zero, feedback ordering, optional settling and the separate guarded full-arm application.
- Direct relative and absolute head trajectories through the bridge against an isolated ROS 1 fixture; no arm-command publisher during head execution.
- Direct relative and absolute arm trajectories through the bridge against the isolated fixture; messages command only the selected joint, including a 0.05 rad move without the former displacement cap.
- An out-of-limit target produces no command messages.
- The isolated bridge suite passed with a -0.0016419921875 rad feedback offset: lower-limit arrival succeeded, repeated `-end` sent no commands, and `+end` recovered from slightly out-of-range measured feedback with all transmitted commands in bounds. Evidence: `docs/results/limit_feedback_local_tests.log`, `local_arm_lower_limit_overshoot.json` and `local_arm_limit_recovery.json`. Hardware behavior after this fix remains unverified.
- Both `-end` and `+end` executed through ros1_bridge against the private ROS 1 fixture. Exact endpoint messages arrived on ROS 1, only the selected joint was commanded, and all setpoints stayed within limits. The complete isolated suite passed; results are in `docs/results/joint_end_local_tests.log`, `local_arm_lower_limit.json` and `local_arm_upper_limit.json`.
- Isolated bridge regression after the completion change passed. A 1.1-second simulated trajectory completed in 1.104857 seconds; exact-field forwarding, limits/minimum-time rejection and repeated-zero no-op remain verified. Evidence: `docs/results/on_time_completion_local_tests.log` and `on_time_completion_unit_tests.log`.
- A rate-limited ROS 1 actuator fixture reaches absolute zero with continued final-target publication; repeating zero produces zero command messages. Measured zero arrival and no-op reports are saved separately.
- Isolated minimum-time regression: a 0.05 s reference was rejected with a computed minimum of 1.074570 s and zero command messages; a 1.1 s reference was accepted and reached its measured target.
- Injected collision feedback still aborts the separate guarded full-arm application; the simplified single-joint script has no collision gate.
- Unavailable-master preflight returns a structured error without waiting for node registration.
- Earlier physical read-only testing: joint/assembly/endpoint state, collisions, limits, head/gripper status and FK/IK service responses. These are recorded observations, not a claim about the robot's current state.

Evidence is in `docs/results/`, particularly `bridge_pairs.txt`, `local_transport.json`, `hardware_readonly.json`, `minimum_time_unit_tests.log`, `minimum_time_local_tests.log`, `local_minimum_time_rejection.json`, `local_absolute_zero.json` and `local_repeated_absolute_zero.json`. Simulated tracking results do not establish physical actuator performance.

## Remaining

- Direct ROS 1 head activation diagnosis now confirms target -2.982996, speed_ratio 0.1, pan_mode 1 arriving at the connected controller: its receive counter increased by 52 messages during a five-second read-only check. Head feedback stayed passive despite enabled/ready/homed assembly state. No competing bridge commands or head warning were observed. The internal controller reason remains unknown; this direct test does not depend on ROS 2 or the bridge. Evidence: `docs/results/head_active_request_readonly.json`.
- No physical arm or head motion has been performed by the assistant. Physical tracking and partial joint commands still require hardware validation. The single-joint CLI now executes directly when the operator runs it, as requested.
- The operator reported physical right_j1 movement from -3.4220 to -3.1037 rad and then from -3.0894 to -2.7717 rad while targeting zero. This confirms the earlier script exited before arrival. The measured speed is consistent with the earlier 0.05 SDK speed ratio; the fixed script has not been hardware-tested.
- The operator reported a head target of zero with measured angle unchanged at -2.982996 rad after 20 seconds. A subsequent report confirmed fresh passive head mode 0 at failed arrival. Live read-only checks found the robot enabled/ready/homed, a connected `/realtime_loop` head subscriber, matching ROS 1/ROS 2 head feedback, and firmware 5.3.1 supported by the original SDK. ROS connection counters showed 4,794 head messages received. Actual ROS 1 command fields during the failed run remain uncaptured, so the reason for not accepting active mode remains unconfirmed. A right_j1 collision diagnostic was also recorded; its relationship to the head behavior is unknown. No actuator commands or robot configuration changes were sent during diagnosis.
- An earlier read-only preflight on 2026-10-12 could not reach the physical master (`No route to host`). Later read-only diagnosis successfully reconnected and obtained the states recorded above.
- Earlier measured `right_j6 = -4.714662109375` was below its verified lower limit `-4.7124`. That approximately 0.002262 rad excursion is now within the single-joint app's measured-feedback allowance; command targets remain strictly within limits. The older full-arm application retains all-arm checks.
- Physical trajectory tracking, stop behavior and command timing remain untested.
- Enable/disable and gripper actuation are not automated. Cartesian control is not implemented. This ros1_bridge does not translate ROS 1 action protocols.

## Resume

1. Read `README.md` and `docs/SINGLE_JOINT_CONTROL.md`.
2. For read-only operation, run `pixi run bridge-start` in one terminal and `pixi run ros2-shell` in another.
3. For direct control, start `pixi run bridge-start --commands --head-commands`, then run `python scripts/move_joint.py --joint right_j0 --distance 0.01 --time 5` inside the ROS 2 shell, or select `head_pan`.
4. Add `--dry-run` only when a preview without movement is wanted. No execution or operator-approval flags are needed for this script.

The modified nested `bridge_ws/src/ros1_bridge` checkout is preserved by `patches/ros1_bridge_sawyer.patch` and its recorded base commit. Build outputs and the local `config/network.env` remain on this workstation and are intentionally Git-ignored. The example network configuration is versioned.
