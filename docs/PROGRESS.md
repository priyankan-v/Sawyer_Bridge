# Progress checkpoint — 2026-10-09

## Completed

- Preserved the original working joint-state bridge and its build.
- Built the original ROS 1 Sawyer interfaces in `ros1_ws` without changing their definitions.
- Built compatible ROS 2 interfaces in `ros2_ws`: 51 message/service/action definitions.
- Rebuilt only the necessary bridge components in a separate `build_sawyer` / `install_sawyer` overlay. Verified converters for 46 Sawyer messages and three services, plus standard JointState.
- Added selective, directional bridging with command topics disabled by default.
- Isolated Python environments and added Pixi shell/startup tasks so ordinary `python` and `ros2` commands work after entering `pixi run ros2-shell` once.
- Added readiness/preflight checks and ROS 2 position/quintic control applications.
- Added `scripts/move_joint.py`: selects `right_j0`–`right_j6` or `head_pan`, accepts relative distance or absolute position and trajectory duration. Preview publishes no commands; execution requires `--execute --operator-approved`.
- Added documentation, startup commands, reproducible builds and validation artifacts.

## Verified

- ROS 1 and ROS 2 interface builds and generated type support.
- Native conversion/serialization checks, including boolean arrays.
- Isolated ROS 2 → ROS 1 JointCommand forwarding: 200/200 messages with identical fields.
- Selective topic direction and disabled-command behavior.
- Eleven unit tests, including simulated arm/head quintic execution and safety checks.
- Relative and absolute head trajectories through the bridge against an isolated ROS 1 fixture: 400 head commands observed; no arm-command publisher during head execution.
- Earlier physical read-only testing: joint/assembly/endpoint state, collisions, limits, head/gripper status and FK/IK service responses. These are recorded observations, not a claim about the robot's current state.

Evidence is in `docs/results/`, particularly `bridge_pairs.txt`, `local_transport.json`, `hardware_readonly.json`, `single_joint_unit_tests.log`, `single_joint_local_tests.log` and `local_head_motion.json`. Simulated tracking results do not establish physical actuator performance.

## Remaining

- No physical arm or head motion has been performed. First motion requires explicit operator approval and fresh readiness checks.
- Earlier measured `right_j6 = -4.714662109375` was below its verified lower limit `-4.7124`. Execution remains blocked if this condition persists; limits and safety checks have not been bypassed.
- Physical trajectory tracking, stop behavior and command timing remain untested.
- Enable/disable and gripper actuation are not automated. Cartesian control is not implemented. This ros1_bridge does not translate ROS 1 action protocols.

## Resume

1. Read `README.md` and `docs/SINGLE_JOINT_CONTROL.md`.
2. For read-only operation, run `pixi run bridge-start` in one terminal and `pixi run ros2-shell` in another.
3. Preview with `python scripts/move_joint.py --joint right_j0 --distance 0.01 --time 5` or select `head_pan`.
4. Review live readiness and obtain operator approval before enabling command bridging or executing physical movement.

The modified nested `bridge_ws/src/ros1_bridge` checkout is preserved by `patches/ros1_bridge_sawyer.patch` and its recorded base commit. Build outputs and the local `config/network.env` remain on this workstation and are intentionally Git-ignored. The example network configuration is versioned.
