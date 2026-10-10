# Sawyer interface inventory

Primary definitions are the copied `intera_common` repositories. SDK behavior is
from `intera_sdk/intera_interface/src`; limits are from the physical variant of
`sawyer_robot/sawyer_description/urdf/sawyer_base.urdf.xacro`.
The controller remains ROS 1. No SDK runtime or onboard software was migrated.

| Interface | ROS 1 endpoint and type | ROS 2 behavior |
|---|---|---|
| Joints | `/robot/joint_states`, `sensor_msgs/JointState` | Selected 1→2 monitoring, merge arm subsets from multiple publishers |
| Assembly state | `/robot/state`, `intera_core_msgs/RobotAssemblyState` | Selected 1→2 monitoring; `lowVoltage` maps to `low_voltage` |
| Endpoint | `/robot/limb/right/endpoint_state`, `intera_core_msgs/EndpointState` | Selected 1→2 monitoring |
| Arm commands | `/robot/limb/right/joint_command`, `intera_core_msgs/JointCommand` | Selected 2→1; disabled by default; POSITION_MODE application |
| Command timeout | `/robot/limb/right/joint_command_timeout`, `std_msgs/Float64` | Selected 2→1 with command bridge; SDK default 0.2 s |
| Position speed ratio | `/robot/limb/right/set_speed_ratio`, `std_msgs/Float64` | Selected 2→1 with command bridge; application uses 0.05 |
| Collision | `/robot/limb/right/collision_detection_state`, `intera_core_msgs/CollisionDetectionState` | Selected 1→2; abort on collision or stale feedback |
| Limits | `/robot/joint_limits`, `intera_core_msgs/JointLimits` | Selected 1→2 transient-local; live `accel` required for motion |
| Enable/disable | `/robot/set_super_enable`, `std_msgs/Bool` | Converter and optional 2→1 entry; disabled and untested on hardware; no automatic enable |
| Head pan | `/robot/head/command_head_pan`, `intera_core_msgs/HeadPanCommand` | Guarded single-joint quintic reference in ACTIVE_MODE; 2→1 entry enabled by `--head-commands`; physical motion untested |
| Head state | `/robot/head/head_state`, `intera_core_msgs/HeadState` | Default 1→2 monitoring for single-joint head feedback |
| Tip endpoints | `/robot/limb/right/tip_states`, `intera_core_msgs/EndpointStates` | Converter exists; no default subscription |
| End-effector config | `/io/end_effector/config`, `intera_core_msgs/IONodeConfiguration` | Read-only validation script; optional 1→2 transient-local monitoring |
| Gripper state | `/io/end_effector/right_gripper/state`, `intera_core_msgs/IODeviceStatus` | Read-only validation script; hardware-dependent availability |
| Gripper commands | `/io/end_effector/right_gripper/command`, `intera_core_msgs/IOComponentCommandSrv` | Service converter; no hardware commands implemented; operation/JSON semantics remain SDK-specific |
| FK | `/ExternalTools/right/PositionKinematicsNode/FKService`, `intera_core_msgs/SolvePositionFK` | Service converter; read-only test via ROS 2 |
| IK | `/ExternalTools/right/PositionKinematicsNode/IKService`, `intera_core_msgs/SolvePositionIK` | Service converter; read-only test via ROS 2; returned validity must be checked |
| Cartesian motion | SDK interaction and motion interfaces | Definitions ported; motion controller application not implemented |
| Intera trajectory | `/motion/motion_command`, `intera_motion_msgs/MotionCommandAction` | ROS 2 action definitions build; action protocol bridging unsupported by this bridge |
| Calibration | `intera_core_msgs/CalibrationCommandAction` | ROS 2 definition builds; action protocol bridging unsupported; no calibration sent |
| FollowJointTrajectory | `/robot/limb/right/follow_joint_trajectory`, `control_msgs/FollowJointTrajectoryAction` in SDK server | Requires ROS 1 SDK action-server node; not assumed to exist on robot; no ROS 2 action bridge implemented |

The SDK `limb.py:set_joint_trajectory` explicitly documents that TRAJECTORY_MODE
bypasses safety-system modifications including collision avoidance. Initial
control intentionally uses only POSITION_MODE. Velocity, acceleration and torque
fields remain empty in physical command messages; analytic velocity and
acceleration profiles are used to validate the position trajectory. The isolated
transport test populates every field on an unrelated topic to verify conversion.

The inspected live graph confirms `/motion/motion_command/goal` is subscribed
by `/motion_controller` on the robot. No FollowJointTrajectory endpoints were
present in that graph. FK and IK are existing controller services, and head and
right-gripper status were received through ROS 2 in the read-only hardware test.

All 35 core message definitions, 11 motion message definitions, three services,
and two actions are ported. See `interface_manifest.json` for each source SHA256
and required field renames. ROS 1 sources are unchanged. ROS 2 requires lowercase
snake_case fields, so camelCase and capitalized fields receive complete explicit
mapping rules. `Header` becomes `std_msgs/Header`; `time` and `duration` become
`builtin_interfaces/Time` and `builtin_interfaces/Duration`. Array lengths, field
order, numeric types and constants are preserved. ROS 2 Header has no ROS 1 `seq`;
the bridge's standard Header converter preserves stamp/frame_id.

The bridge patch adds complete top-level service field mappings (needed by FK),
time/duration conversion in services (needed by IO), primitive-array conversion
between ROS 1 byte-backed bool arrays and ROS 2 `vector<bool>`, and correct boolean
wire serialization. It also avoids taking `.front()` of empty primitive arrays.
Action definitions alone do not provide action goal/cancel/status interoperability.

Upstream `parameter_bridge` arguments are ROS **1 parameter names**, not topic
type specifications and not ROS 2 parameter YAML. In this checkout:

```text
parameter_bridge [topics_parameter_name] [services_1_to_2_parameter_name] [services_2_to_1_parameter_name]
```

The extension adds `direction: 1to2|2to1|bidirectional` and `enabled: true|false`
to topic entries. Missing direction retains upstream bidirectional behavior,
preserving the original `config/bridge.yaml`. QoS applies to the ROS 2 publisher
for 1→2 and ROS 2 subscriber for 2→1. The patched bridge fails when a configured
topic cannot be created. Upstream service naming is counterintuitive:
`services_2_to_1` creates a ROS 2 server that calls the existing ROS 1 server.
Entries require the legacy `package: ''` key in this checkout; `type` is
`intera_core_msgs/SolvePositionIK` (without `/srv/`).

The local patch and base commit are stored in `patches/`. No release branch or
unrelated bridge sources were replaced. The old installed bridge is retained.
