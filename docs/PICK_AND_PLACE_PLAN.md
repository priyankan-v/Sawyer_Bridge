# Cartesian pick and place with obstacle avoidance

The task will use taught pick/place poses and a collision-aware planner that
can choose feasible approach directions. There is no fixed assumption that
every pick is approached vertically. A requested pose can still be unreachable
or blocked; that must produce a failed plan rather than an unchecked move.

The current requested scope is a taught-pose motion/gripper demonstration,
deferring object and obstacle dimensions. The new planner checks robot
self-collision, but no table, fixtures or carried-object geometry is included.
Workspace obstacle avoidance remains a later stage. The initial sequence has
a configurable approach direction (`--approach X Y Z`, expressed in `base`)
and clearance; it defaults to 5 cm along positive base Z. Automatic approach
selection around workspace obstacles still requires a measured planning scene.

## Work plan and current progress

| Step | Work | Acceptance check | Status |
|---|---|---|---|
| 1 | Bridge FK/IK; capture and save Cartesian tool poses; check saved goals with seeded IK and FK | Capture the actual gripper tip in `base`; validate frame/tool conventions, all seven joints, response errors and self-collision; publish no motion commands | Implemented; pick/place files captured |
| 2 | Configure Sawyer for MoveIt 2: controller robot/gripper collision geometry, planning group, SRDF, kinematics, live joint state and TF | FK matches the controller captures; limits are intersected with live limits; known self-collisions and a synthetic blocker are detected | Implemented and checked; physical/RViz visual review remains |
| 3 | Build the workspace planning scene with measured table, fixtures, obstacles and the object to grasp | Verify object positions, dimensions and padding in `base`; a deliberately blocking obstacle causes a failed plan or a collision-free detour | Deferred by user for the initial demonstration |
| 4 | Implement plan-only pick/place sequencing | Generate all six arm moves and three gripper stages, reject partial Cartesian paths and failed plans | Implemented for a configurable approach direction; full sequence tested with a private hypothetical start. Automatic approach selection and carried-object collision checks remain |
| 5 | Implement guarded position-mode execution through the existing joint-command bridge | Retime the planned polyline with bounded smooth segments; recheck the exact executed path and start state; monitor readiness, limits, tracking, head state and scheduling | Implemented; complete simulated task passed |
| 6 | Connect gripper actions and verify grasp/release transitions | Confirm acknowledgement and grasp before transport; monitor grasp loss; preserve gripper holding on failures | Implemented and simulated; failed/lost-grasp guards covered |
| 7 | Validate the full task with fixtures, then a reviewed physical setup | Test blocked goals, no IK, invalid starts, stale/offline plans, trajectory faults and grasp failures; then verify conservative physical motions | Automated checks passed; physical start currently rejected for wrist limit violation |

The current milestone implements capture, a MoveIt planning model, a plan-only
task coordinator and an explicit guarded task executor. It has executed against
a simulated controller, but has not moved the physical robot. An IK solution
is not a motion plan. Moving between IK endpoints
using joint-space interpolation does not guarantee a straight Cartesian path
or obstacle clearance.

## Findings from the workstation and robot

- The controller exposes `/ExternalTools/right/PositionKinematicsNode/FKService`
  and `/ExternalTools/right/PositionKinematicsNode/IKService`.
- Live FK accepts both `right_hand` and `right_gripper_tip`. They have different
  positions; capture defaults to `right_gripper_tip` because the goal concerns
  the gripper. The installed fingertip/tool geometry must still be verified.
- The robot reference frame returned by FK is `base`.
- `/opt/ros/humble` contains MoveIt 2, OMPL, RViz and robot state publisher.
  The current control/bridge applications use Conda Humble and Python 3.12.
  MoveIt should run in a separate system ROS shell/process on the same DDS
  domain. Do not source system ROS into the Conda runtime. Standard ROS 2
  messages/services will connect the planner and the control application.
- `planning/prepare_model.py` now generates a Sawyer SRDF, local mesh references
  and intersected planning limits from the controller's exported URDF and limits.
- `planning/moveit.launch.py` runs the MoveIt planning services and robot state
  publisher in the system runtime; trajectory execution and motion actions are
  disabled. It starts a telemetry relay for merged, locally timestamped states.
- The robot ROS graph advertises 2D camera images and CameraInfo, with no depth
  image or PointCloud2 feed found during inspection. Automatic 3D obstacle
  mapping is therefore not available from the current setup.

Start obstacle modelling with measured boxes/cylinders or meshes for the table,
fixtures, obstacles and target object. We need their dimensions and poses in
`base`, including an uncertainty margin. Unknown space is not certified clear
by the camera viewer. An empty scene cannot demonstrate obstacle avoidance.
Dynamic obstacle avoidance needs additional sensing, calibration and scene
updates; it is not part of the initial static-scene milestone.

The intended planning stack is MoveIt 2 with its Planning Scene and OMPL,
followed by a multi-stage pick/place coordinator. MoveIt Task Constructor is a
candidate for evaluating grasp and approach alternatives; it is not installed
in the system ROS environment and needs a separate build/install and Sawyer
integration step. The official Humble examples cover
[planning around collision objects](https://moveit.picknik.ai/humble/doc/tutorials/planning_around_objects/planning_around_objects.html)
and [pick/place stages with object attachment](https://moveit.picknik.ai/humble/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html).

## Step 1: build and enable read-only kinematics

Build the new application entry point once:

```bash
./scripts/env.sh ros2 colcon --log-base ros2_ws/log_conda build \
  --base-paths ros2_ws/src --build-base ros2_ws/build_conda \
  --install-base ros2_ws/install_conda --symlink-install \
  --packages-select sawyer_ros2_control
```

Stop the existing bridge with Ctrl-C in its terminal. Restart it with
`kinematics:=true`. The read-only teaching/checking workflow needs no command
transport:

```bash
source config/network.env
./scripts/env.sh ros2 ros2 launch sawyer_ros2_control bridge.launch.py kinematics:=true
```

If other existing applications require their command/image transports,
preserve their launch options and add `kinematics:=true` to that launch instead.
Run one main bridge per robot/domain. The kinematics option exposes services
that calculate poses and joint solutions; it does not command motion.

## Capture pick and place poses

In another terminal, enter the existing ROS 2 application shell:

```bash
./scripts/env.sh ros2
ros2 run sawyer_ros2_control cartesian read
```

The output identifies the reference and tool frames and includes:

- `frame_id`: `base`.
- `tip_name`: `right_gripper_tip` by default.
- `position`: `[x, y, z]` in metres.
- `orientation_xyzw`: quaternion `[x, y, z, w]`.
- `joints`: the seven arm joint positions used for FK.

Using the robot operator's normal teaching procedure, position the gripper at
the desired grasp pose. Keep it stationary, then capture:

```bash
ros2 run sawyer_ros2_control cartesian capture --output "$SAWYER_ROOT/config/poses/pick.json"
```

Teach the desired placement pose, keep the arm stationary, then capture:

```bash
ros2 run sawyer_ros2_control cartesian capture --output "$SAWYER_ROOT/config/poses/place.json"
```

These are tool poses, not object centres. They must include the intended tool
orientation and grasp/placement offsets. No software here moves or enables the
arm for teaching. Existing files are not overwritten; choose a new filename
when revising a pose. Use `--tip right_hand` only when intentionally teaching
that tool link, and keep the same tool frame throughout the task.

## Check a captured goal

```bash
ros2 run sawyer_ros2_control cartesian ik --pose "$SAWYER_ROOT/config/poses/pick.json"
ros2 run sawyer_ros2_control cartesian ik --pose "$SAWYER_ROOT/config/poses/place.json"
```

The node seeds IK from current arm feedback, rejects failed/self-colliding
solutions, validates the returned seven joints against live and model limits,
and checks that FK matches the requested goal. `ik_valid` means only that the
goal passed those checks. It does not check table/obstacle clearance, choose an
approach, certify the intervening path, or execute anything.

`cartesian` and `planning/plan_pick_place.py` create no robot command publishers.
The separate `pick_place` executable defaults to preview; `--execute` explicitly
starts guarded arm/gripper execution after all preflight checks pass.

## Step 2: prepare the planning model

Keep the existing main bridge running with kinematics and telemetry. From the
repository root, export the controller's description, live limits and measured
head/arm/gripper state, then prepare the local model:

```bash
source config/network.env
./scripts/env.sh ros1 python scripts/export_moveit_model.py
/usr/bin/python3 planning/prepare_model.py
```

Generated files live under `.runtime/moveit_model/`. They are local artifacts,
not replacements for the controller's robot description. The URDF keeps the
controller's collision primitives, finger variant, connector plate and TCP
offsets; mesh URIs resolve to files already in this repository. Arm bounds are
the intersection of URDF and controller bounds. Planning velocity and
acceleration caps are 0.15 rad/s and 0.3 rad/s² before request scaling.

The SRDF excludes rigid internal contacts, direct neighbours, and eight
specific manufacturer-documented shoulder/opposing-finger pairs. The source
links are recorded in `planning/collision_exclusions.json`; it does not disable
other pairs just because a small random sample did not collide.

The SDK publishes full gripper opening under `right_gripper_l_finger_joint`.
The URDF represents each finger's half-opening travel. The export/relay divides
that signal by two; arm angles remain measured values, including any limit
violations. The relay also merges partial JointState messages and uses local
receipt time because controller and workstation timestamps have different
epochs. Stale arm/head feedback stops derived state publication.

In a separate terminal, launch system MoveIt through the isolation wrapper:

```bash
source config/network.env
./scripts/env.sh system-ros2 ros2 launch planning/moveit.launch.py
```

Do not source `/opt/ros/humble/setup.bash` into the existing Conda shell.
Planning services are under `/sawyer_planning`. The launcher pins the installed
KDL plugin in the MoveIt process to avoid an observed Humble 2.5.9 plugin-unload
segmentation fault during shutdown. The class-loader warning may remain; clean
shutdown is verified by the isolated integration runner.

## Step 3: generate a demonstration plan

With the planning launch running, use another terminal from the repository root:

```bash
source config/network.env
./scripts/env.sh system-ros2 python3 planning/plan_pick_place.py
```

This reads the captured `config/poses/pick.json` and `place.json`, and requests:

1. Open before picking (a planned gripper action).
2. Move to pre-pick using collision-checked joint-space planning.
3. Descend to the taught pick pose using a complete Cartesian path.
4. Grasp (a planned gripper action).
5. Lift along the reverse approach direction.
6. Transfer to pre-place using joint-space planning.
7. Descend to the taught place pose using a complete Cartesian path.
8. Release (a planned gripper action).
9. Retreat along the reverse approach direction.

The output is `.runtime/pick_place_plan.json`, containing the trajectories and
gripper action descriptions. `--clearance 0.03` changes the clearance;
`--approach 1 0 0` chooses positive base X instead of base Z. The planned
closed-hand planning stages assume zero aperture. The executor confirms the
actual grasp width and rechecks each subsequent arm path with the measured
finger geometry before transporting. No carried-object geometry is checked.

The code requires an in-bounds, self-collision-free start, validates goal
solutions and rejects incomplete Cartesian paths. It does not execute any
trajectory or open/close the real gripper.

## Step 4: preview and explicitly execute a valid live plan

The `pick_place` executable was built in this workspace. Rebuild
`sawyer_ros2_control` when installing these changes elsewhere. In the Conda ROS 2
application shell, preview the freshly generated plan:

```bash
ros2 run sawyer_ros2_control pick_place --plan "$SAWYER_ROOT/.runtime/pick_place_plan.json"
```

The preview validates every arm waypoint and prints stage durations. The
executor retimes the planned polyline into quintic segments with zero velocity
and acceleration at each waypoint. This is deliberately conservative for the
initial demonstration: default velocity/acceleration caps are 0.015 rad/s and
0.03 rad/s². It does not reuse unvalidated smoothing between waypoints.

When the live start/goal checks pass and the operator has prepared the clear
workspace, explicit execution uses:

```bash
ros2 run sawyer_ros2_control pick_place \
  --plan "$SAWYER_ROOT/.runtime/pick_place_plan.json" --execute
```

The main bridge needs `commands:=true gripper_commands:=true`; the MoveIt
planning launch also remains running. The executor never enables the robot or
calibrates the gripper automatically. It requires a plan generated from live
feedback less than 120 seconds ago, a matching model, an in-bounds stationary
arm, fresh readiness/collision feedback and a calibrated error-free gripper.
Offline snapshot plans are rejected before execution.

Before each arm stage it checks the retimed polyline with MoveIt's current
head and measured gripper geometry, at no more than 0.005 rad between checked
joint samples. A small settled arrival residual (up to the SDK's half-degree
joint tolerance) is connected back to the planned first waypoint by a separately
checked segment. This never expands the joint bounds. Commands stream at 100 Hz
in `POSITION_MODE` with a 0.2 s timeout and 0.05 controller speed ratio.

The executor checks limits, readiness, collisions, tracking, scheduling and
head movement during arm stages. A close must report `is_gripping` before lift;
grasp loss aborts transport. An error or Ctrl-C ceases the task, sends a measured
position hold only when feedback is healthy, and does not issue a gripper-open
command during failure handling. An accepted gripper operation may continue on
the device; use its normal stop controls if needed. An empty-hand sequence stops
at the grasp check rather than pretending an object was picked.

### Current captured-pose diagnostics (2026-10-16)

MoveIt FK agrees with both controller captures to numerical precision, with
no self-collision contacts after applying documented model exclusions. Both
recorded configurations and the measured start contain:

```text
right_j6 = -4.714662109375 rad
lower limit = -4.7124 rad
undershoot = 0.002262109375 rad (approximately 0.13 degrees)
```

The default controller IK check also failed for the pick pose when seeded from
the current configuration. This is a solver/seed result, not proof that no
alternative valid configuration exists for that Cartesian goal.

The current start is rejected before task planning. Resolve the wrist state
through the robot operator's normal procedure, then refresh the model snapshot
and rerun goal/plan checks. Do not edit a saved measured angle to pretend the
robot has moved. If a goal still cannot obtain a valid IK solution, teach that
goal again with the wrist away from its limit and save it to a new capture.

A read-only recovery preview to an inward wrist target was checked successfully:

```bash
ros2 run sawyer_ros2_control move_joint --joint right_j6 --position -4.70 --time 3 --dry-run
```

This sends no motion commands. It reports the measured start and the in-limit
reference used by the existing joint tool. Any physical recovery still belongs
to the operator's normal robot-control procedure.

## Verification

```bash
./scripts/env.sh ros2 python -m unittest discover -s tests -v
env -u PYTHONPATH -u PYTHONHOME PYTHONNOUSERSITE=1 /usr/bin/python3 tests/run_cartesian_local.py
./scripts/env.sh system-ros2 python3 tests/planning_system_checks.py -v
./scripts/env.sh system-ros2 cmake -S tests/moveit -B .runtime/moveit_native_build
./scripts/env.sh system-ros2 cmake --build .runtime/moveit_native_build --parallel 2
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=91 ./scripts/env.sh system-ros2 \
  .runtime/moveit_native_build/model_checks .runtime/moveit_model \
  .runtime/moveit_model/model_checks.json config/poses/pick.json config/poses/place.json
/usr/bin/python3 tests/run_moveit_local.py
/usr/bin/python3 tests/run_pick_place_local.py
```

The local Cartesian suite uses its own localhost master and DDS domain. It
verifies default-disabled kinematics, FK/IK transport, capture of an explicitly
named tool, pose-file preservation, and failure handling without robot command
publishers.

Milestone verification on 2026-10-16: the application build and all 85 unit tests
passed, as did the isolated Cartesian integration suite. A temporary service-only
bridge was also used to capture the physical `right_gripper_tip` pose and run a
seeded IK/FK round trip with zero reported position/orientation error. The
temporary bridge and parameters were removed afterward. No robot motion or
gripper commands were published during these hardware checks. This verifies the
capture/kinematics milestone, not obstacle avoidance or task execution.

The planning/execution milestone passes 98 application/model unit tests, five
system-ROS planning/relay unit checks, native FK/self-collision/blocker checks,
an isolated complete plan-sequence/blocker-rejection integration test, and a
complete execution against a simulated ROS 2 arm/gripper controller. The latter
observed about 1,400 simulated arm commands and exactly three gripper commands
on private localhost DDS domain 92. The complete sequences use separate,
explicitly hypothetical in-bounds test states and goals; they do not validate
the out-of-bounds physical start. Hardware checks were read-only, and the
physical task preflight still reports the wrist limit violation. No command
was published to the physical robot during this work.
