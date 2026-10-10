# Move one Sawyer joint with a quintic reference

`scripts/move_joint.py` selects one of `right_j0`–`right_j6` or `head_pan`, reads
its current joint-state position, and constructs a quintic reference. Specify
exactly one of `--distance` (signed relative displacement) and `--position`
(absolute joint-state angle). All angles are **radians**, and `--time` is seconds.
`--duration` is an alias for `--time`.

The script defaults to a publication-free preview. Physical execution requires
both `--execute` and `--operator-approved`, a successful isolated transport test,
fresh ROS 1 subscriber/readiness verification and fresh ROS 2 feedback. It does
not enable Sawyer or change safety mechanisms.

## Preview

Terminal 1 — stop an existing selected bridge before replacing it:

```bash
cd ~/sawyer_rs
pixi run bridge-start
```

Terminal 2 — enter the ROS 2 shell once:

```bash
cd ~/sawyer_rs
pixi run ros2-shell
```

Then run ordinary Python commands:

```bash
# Move right_j0 by +0.01 rad over a five-second reference:
python scripts/move_joint.py --joint right_j0 --distance 0.01 --time 5

# Move head_pan by -0.01 rad over a five-second reference:
python scripts/move_joint.py --joint head_pan --distance -0.01 --time 5

# Absolute target: choose a value near the current reported joint-state position.
python scripts/move_joint.py --joint head_pan --position -2.45 --time 5
```

The plan prints the measured starting angle, target angle, displacement,
duration, peak reference velocity/acceleration and `execute: false`.
An absolute target example is illustrative; the script rejects targets outside
the displacement cap or verified limits. Use a fresh preview immediately before
execution because a relative target is calculated from the then-current position.

The same application is available as:

```bash
ros2 run sawyer_ros2_control move_joint --joint right_j1 --distance 0.01 --time 5
```

## Operator-approved execution

The first physical movement remains pending operator approval. At the earlier
inspection, measured `right_j6` was slightly outside its lower limit. If that
condition persists, both arm and head execution are blocked by the retained
all-arm safety checks. Resolve readiness through normal operator procedures;
the script does not widen limits or provide a bypass.

After reviewing the workspace, current feedback and target, replace Terminal 1's
read-only bridge with the required command path:

```bash
# Arm commands only:
pixi run bridge-start --commands

# Or head commands only:
pixi run bridge-start --head-commands

# Or both paths, if both are needed:
pixi run bridge-start --commands --head-commands
```

Choose one bridge instance. Starting it advertises the selected command topic
without sending a motion message. It requires the prior isolated test and a
successful live readiness preflight.

Only after explicit approval, in Terminal 2's ROS 2 shell:

```bash
python scripts/move_joint.py --joint right_j0 --distance 0.01 --time 5 \
  --execute --operator-approved --report docs/results/single_arm_motion.json

# Or, with the head command bridge enabled:
python scripts/move_joint.py --joint head_pan --distance -0.01 --time 5 \
  --execute --operator-approved --report docs/results/single_head_motion.json
```

The default displacement cap is **0.02 rad**. A larger reviewed movement can set
`--max-displacement`, while all controller/URDF bounds and derivative checks
still apply. For example, a preview of a 0.1 rad move uses
`--distance 0.1 --max-displacement 0.1 --time 10`. Commands run at 100 Hz by default,
with a 0.05 speed ratio, 0.1 rad/s reference-speed cap and 0.2 rad/s²
reference-acceleration cap. The speed cap also accounts for the speed ratio and
the selected joint's actual velocity limit. An insufficient duration is rejected
with a suggested minimum duration.

## Trajectory and command semantics

For `u=t/T` clamped to `[0,1]`:

```text
q(t) = q0 + (q1-q0) * (10u³ - 15u⁴ + 6u⁵)
v(t) = (q1-q0)/T * (30u² - 60u³ + 30u⁴)
a(t) = (q1-q0)/T² * (60u - 180u² + 120u³)
```

Starting/ending reference velocity and acceleration are zero. Exact peak
derivatives are checked over the entire trajectory, including between samples.
Arm references use `JointCommand.POSITION_MODE`; the other six arm joints retain
their starting setpoints. Torque, velocity and acceleration command arrays remain
empty. Head references use `HeadPanCommand.SET_ACTIVE_MODE` on the separate
head topic, with the target expressed as a head joint angle rather than a
base-frame cancellation angle. The physical head bounds in the copied URDF are
`[-5.0952, 0.9064]` rad; execution intersects them with live limits. Actual
float32 head targets are validated before publication.

Head ACTIVE_MODE and the conservative speed ratio persist after the command,
matching the original SDK's behavior. Other arm joints are monitored during head
execution, and no arm command publisher is created for a head-only move.

The quintic curve specifies the **command reference**. The robot's position
controllers and safety behavior can modify actual motion, so exact physical
duration or quintic tracking is not guaranteed. Execution checks feedback,
tracking error, scheduling, limits, collision/readiness and settling. The final
report records measured angle, target, tracking error and elapsed time; settling
may add up to three seconds after the requested reference duration.

Press Ctrl+C to stop publishing. A measured-position hold is attempted only
after commands have actually started and safety feedback remains healthy.
Stopping publication is not a hardware emergency stop; the controller can retain
its last position target. Use the robot's normal stop controls for an emergency.

## Local verification

Only `sawyer_ros2_control` needs rebuilding; existing message converters already
include HeadPanCommand and HeadState:

```bash
./scripts/env.sh ros2 bash -c 'cd "$SAWYER_ROOT/ros2_ws"; colcon build --symlink-install --packages-select sawyer_ros2_control'
./scripts/env.sh ros2 python -m unittest discover -s tests -v
./scripts/test_local.sh
```

Unit tests exercise both arm and head execution loops with simulated feedback,
quintic extrema, relative/absolute argument selection, approval gates, unselected
arm setpoints, head message fields and head faults. The localhost integration
test also checks head relative/absolute commands across ros1_bridge using its
own master on port 11421 and DDS domain 87 with localhost-only transport.
Local simulation tests do not establish physical actuator performance.

The implementation follows the original `intera_sdk` head interface and the
[Rethink Head API](https://rethinkrobotics.github.io/intera_sdk_docs/5.1.0/intera_interface/html/intera_interface.head.Head-class.html)
and [HeadPanCommand definition](https://rethinkrobotics.github.io/intera_sdk_docs/5.1.0/intera_core_msgs/html/msg/HeadPanCommand.html).
