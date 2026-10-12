# Move one Sawyer joint

`scripts/move_joint.py` is the complete Python script. It executes immediately
when you run it. The ROS 2 package entry point uses the same source file.

Start one bridge in Terminal 1:

```bash
cd ~/sawyer_rs
pixi run bridge-start --commands --head-commands
```

In Terminal 2, enter the ROS 2 environment once:

```bash
cd ~/sawyer_rs
pixi run ros2-shell
```

Then run ordinary Python commands:

```bash
# Relative arm movement: +0.01 radians over 5 seconds.
python scripts/move_joint.py --joint right_j0 --distance 0.01 --time 5

# Relative head movement.
python scripts/move_joint.py --joint head_pan --distance -0.01 --time 5

# Absolute target: replace 0.10 with the desired joint-state angle.
python scripts/move_joint.py --joint right_j0 --position 0.10 --time 5

# Optional preview, without sending a command.
python scripts/move_joint.py --joint right_j0 --distance 0.01 --time 5 --dry-run
```

Valid names are `right_j0`–`right_j6` and `head_pan`. Angles/distances are radians;
time is seconds. Specify exactly one of `--distance` and `--position`.
Both accept expressions such as `pi`, `pi/2`, `2*pi/3` and `(pi-0.1)/2`.
Negative expressions work with spaces or equals signs: `--position -pi/2`,
`--position=-pi/2` or `--distance -pi/12`. Quote expressions that contain spaces.
The parser supports numbers, `pi`, parentheses and basic
`+ - * /` arithmetic only. The resulting radians still undergo the same joint
limit and minimum-time checks.

For the selected joint's lower or upper position limit:

```bash
python scripts/move_joint.py --position -end --time 5 --joint right_j5
python scripts/move_joint.py --position +end --time 5 --joint right_j5
```

The endpoints use current controller/URDF limit intersections, rather than fixed
angles. The script prints the selected range and numeric target. These markers
are supported only for `--position`; minimum trajectory time is still enforced.
For float32 head commands, the nearest representable value inside the limit is
used if rounding the exact endpoint would cross it. `--dry-run` previews without
publishing commands.
`--duration` is an alias for `--time`. Optional `--report FILE.json` saves measured
position and tracking results. The package entry point also works:

```bash
ros2 run sawyer_ros2_control move_joint --joint right_j0 --distance 0.01 --time 5
```

The script reads current feedback and live joint-position limits. Its position
check uses the intersection of controller and URDF bounds for the selected joint.
Targets and published setpoints remain strictly within those limits, including
the actual float32 value of head targets. Measured feedback may exceed a limit
by up to 0.008726646 rad (0.5 degrees), independently of `--tolerance`. This
allows minor tracking/calibration overshoot at an endpoint without blocking a
return move. Larger excursions still stop the application. Measurements remain
unmodified in reports; an out-of-range starting measurement within this allowance
uses the boundary as its quintic reference start, keeping all commands in range.
`reference_start_rad` and `limit_overshoot_rad` make this visible in reports.
Missing feedback or a disconnected command
bridge produces an error; malformed/non-finite inputs are rejected.

For either `--position` or `--distance`, the minimum feasible quintic time is:

```text
d = abs(target - measured_start)
T_min = max(1.875*d / velocity_limit,
            sqrt((10/sqrt(3))*d / acceleration_limit))
```

Velocity uses the smaller of controller and URDF limits; acceleration uses the
live controller limit when supplied. If acceleration data is absent, the result
is explicitly velocity-only. A duration below the minimum prints the required
time and rejects the move before commands are sent. A duration at or above it is
accepted. `--dry-run` performs the same calculation without movement, and reports
include `minimum_time_seconds`. The minimum is recalculated from fresh start
feedback before execution.

The quintic reference is:

```text
u = clamp(t/T, 0, 1)
q(t) = q0 + (q1-q0) * (10u^3 - 15u^4 + 6u^5)
```

Starting/ending reference velocity and acceleration are zero. It publishes at
100 Hz for the requested reference duration, with progress printed once per
second. At the deadline, it publishes the final target and checks fresh feedback.
The default arrival tolerance is 0.008726646 rad (0.5 degrees), matching the
original Intera SDK's arm tolerance. A repeated absolute command already within
that tolerance sends no commands. There is no additional settling wait by
default: `--settle-timeout` defaults to zero. A larger remaining error still
returns `status: not_reached`; the reported measured angle/error is never altered.
Optional `--tolerance` selects a stricter tolerance and `--settle-timeout` allows
extra final-target publication if desired. Feedback older than one second cannot
confirm arrival, and duplicate/older nonzero message timestamps are ignored.

Arm messages contain only the
selected joint in POSITION_MODE, following the original SDK's partial position
commands. Head messages use SET_ACTIVE_MODE and speed ratio 1.0. Existing arm
position-control speed settings are replaced with SDK speed ratio 1.0 before
motion, so an earlier persistent ratio such as 0.05 cannot silently slow the run.
This setting persists. Normal onboard velocity limits still apply.

The application checks position bounds and quintic duration feasibility. There
are no extra displacement, readiness, collision or tracking gates. It does not call ROS 1
preflight or shell subprocesses. Sawyer retains its normal onboard safety and
controller behavior; the robot must already be enabled. Actual tracking can
differ from the reference. The separate full-arm application retains its older
guarded behavior.

Ctrl+C stops publishing; the controller can keep its last position target. Use
Sawyer's normal stop controls for emergencies.

Local tests run against an isolated ROS 1 master on localhost port 11421 and
DDS domain 87. They verify direct relative/absolute arm/head commands, optional
preview, single-joint message fields and rejection of an out-of-limit target.
They also use rate-limited feedback to check measured arrival at absolute zero
and that repeating zero publishes no commands. A too-short duration is rejected
without publishing commands; a feasible duration passes. No physical actuator movement is
performed by those tests.
