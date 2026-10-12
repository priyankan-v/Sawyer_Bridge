#!/usr/bin/env python3
"""Move one Sawyer arm joint or head_pan using a quintic position reference."""
import argparse
import ast
import json
import math
import struct
import sys
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from intera_core_msgs.msg import HeadPanCommand, HeadState, JointCommand, JointLimits

JOINTS = [f'right_j{i}' for i in range(7)] + ['head_pan']
JOINT_ANGLE_TOLERANCE = 0.008726646  # Original Intera SDK: half a degree.


def parse_angle(text):
    """Parse radians using numbers, pi, parentheses and basic arithmetic only."""
    def calculate(node):
        if isinstance(node, ast.Constant) and type(node.value) in (int, float):
            return float(node.value)
        if isinstance(node, ast.Name) and node.id == 'pi':
            return math.pi
        if isinstance(node, ast.UnaryOp):
            if isinstance(node.op, ast.UAdd):
                return calculate(node.operand)
            if isinstance(node.op, ast.USub):
                return -calculate(node.operand)
        if isinstance(node, ast.BinOp):
            left, right = calculate(node.left), calculate(node.right)
            if isinstance(node.op, ast.Add):
                return left + right
            if isinstance(node.op, ast.Sub):
                return left - right
            if isinstance(node.op, ast.Mult):
                return left * right
            if isinstance(node.op, ast.Div):
                return left / right
        raise ValueError('Unsupported angle expression')

    try:
        if len(text) > 128:
            raise ValueError('Angle expression is too long')
        tree = ast.parse(text.strip(), mode='eval')
        if sum(1 for _ in ast.walk(tree)) > 64:
            raise ValueError('Angle expression is too complex')
        value = calculate(tree.body)
        if not math.isfinite(value):
            raise ValueError('Non-finite angle')
        return value
    except (SyntaxError, ValueError, ArithmeticError) as error:
        raise argparse.ArgumentTypeError(
            'Use a finite angle in radians, such as 0.1, pi, pi/2 or -pi/4; '
            'only numbers, pi, parentheses and + - * / are supported') from error


def parse_position(text):
    if text.strip() in ('-end', '+end'):
        return text.strip()
    return parse_angle(text)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--joint', required=True, choices=JOINTS)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument('--distance', type=parse_angle, help='Relative displacement in radians (e.g. 0.1, pi/4)')
    target.add_argument('--position', type=parse_position,
                        help='Absolute radians (e.g. pi/2), -end for lower limit, +end for upper limit')
    parser.add_argument('--time', '--duration', dest='duration', type=float, required=True, help='Trajectory time in seconds')
    parser.add_argument('--dry-run', action='store_true', help='Print the target without sending commands')
    # Same arrival tolerance as intera_interface/settings.py (half a degree).
    parser.add_argument('--tolerance', type=float, default=JOINT_ANGLE_TOLERANCE,
                        help='Position arrival tolerance in radians (default: 0.008726646, 0.5 degrees)')
    parser.add_argument('--settle-timeout', type=float, default=0.,
                        help='Optional extra seconds to reach the final target (default: 0, no extra wait)')
    parser.add_argument('--report', type=Path, help='Optional JSON result file')
    # argparse mistakes a space-separated -pi/2 for an option. Attach angle
    # values to their flags before parsing, keeping genuine options intact.
    arguments = list(sys.argv[1:] if argv is None else argv)
    normalized = []
    index = 0
    while index < len(arguments):
        argument = arguments[index]
        if argument in ('--position', '--distance') and index + 1 < len(arguments):
            value = arguments[index + 1]
            if not value.startswith('--') and value != '-h':
                normalized.append(argument + '=' + value)
                index += 2
                continue
        normalized.append(argument)
        index += 1
    args = parser.parse_args(normalized)
    value = args.distance if args.distance is not None else args.position
    if ((not isinstance(value, str) and not math.isfinite(value)) or any(not math.isfinite(v) or v <= 0
                                      for v in (args.duration, args.tolerance))
            or not math.isfinite(args.settle_timeout) or args.settle_timeout < 0):
        parser.error('Position/distance must be finite; time and tolerance must be positive; settle timeout must be nonnegative')
    return args


def quintic_position(start, target, elapsed, duration):
    if elapsed >= duration:
        return target  # Preserve the exact endpoint, including at joint limits.
    u = min(1.0, max(0.0, elapsed / duration))
    position = start + (target - start) * (10*u**3 - 15*u**4 + 6*u**5)
    return min(max(start, target), max(min(start, target), position))


def minimum_duration(distance, velocity, acceleration=None):
    """Exact peak velocity/acceleration bounds for this zero-endpoint quintic."""
    if not math.isfinite(distance) or not math.isfinite(velocity) or velocity <= 0:
        raise ValueError('Distance must be finite and the velocity limit must be positive')
    distance = abs(distance)
    duration = 1.875 * distance / velocity
    if acceleration is not None:
        if not math.isfinite(acceleration) or acceleration <= 0:
            raise ValueError('The acceleration limit must be finite and positive')
        duration = max(duration, math.sqrt((10 / math.sqrt(3)) * distance / acceleration))
    return duration


class JointMover(Node):
    def __init__(self, args):
        super().__init__('sawyer_move_joint')
        self.args = args
        self.position = None
        self.position_received = None
        self.position_stamp = 0
        self.position_sequence = 0
        self.bounds = None
        self.velocity_limit = None
        self.acceleration_limit = None
        self.minimum_time = 0.
        self.head_state = None
        self.head_state_received = None
        self.error = None
        self.pub = None
        path = Path(get_package_share_directory('sawyer_ros2_control')) / 'joint_limits.json'
        self.model = json.loads(path.read_text())['joints'][args.joint]
        self.create_subscription(JointState, '/robot/joint_states', self.on_position, qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(JointLimits, '/robot/joint_limits', self.on_limits, qos)
        if args.joint == 'head_pan':
            self.create_subscription(HeadState, '/robot/head/head_state', self.on_head_state, 1)

    def on_head_state(self, msg):
        self.head_state = msg
        self.head_state_received = time.monotonic()

    def on_position(self, msg):
        if self.args.joint not in msg.name:
            return
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            self.error = 'Malformed joint-state positions'
            return
        value = msg.position[msg.name.index(self.args.joint)]
        if not math.isfinite(value):
            self.error = 'Non-finite joint-state position'
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp and self.position_stamp and stamp <= self.position_stamp:
            return  # Ignore duplicate/older feedback from overlapping bridges.
        self.position = value
        self.position_received = time.monotonic()
        self.position_stamp = stamp
        self.position_sequence += 1

    def on_limits(self, msg):
        if self.args.joint not in msg.joint_names:
            return
        count = len(msg.joint_names)
        if len(set(msg.joint_names)) != count or len(msg.position_lower) != count or len(msg.position_upper) != count:
            self.error = 'Malformed joint-position limits'
            return
        i = msg.joint_names.index(self.args.joint)
        lower, upper = msg.position_lower[i], msg.position_upper[i]
        if not math.isfinite(lower) or not math.isfinite(upper):
            self.error = 'Non-finite joint-position limits'
            return
        self.bounds = max(lower, self.model['lower']), min(upper, self.model['upper'])
        if self.bounds[0] >= self.bounds[1]:
            self.error = 'Controller and URDF joint limits do not overlap'
            return
        if len(msg.velocity) != count:
            self.error = 'Missing or malformed joint velocity limits'
            return
        velocity = msg.velocity[i]
        if not math.isfinite(velocity) or velocity <= 0:
            self.error = 'Invalid joint velocity limit'
            return
        self.velocity_limit = min(velocity, self.model['velocity'])
        self.acceleration_limit = None
        if len(msg.accel):
            if len(msg.accel) != count or not math.isfinite(msg.accel[i]) or msg.accel[i] <= 0:
                self.error = 'Invalid joint acceleration limits'
                return
            self.acceleration_limit = msg.accel[i]

    def check_position(self, position):
        if self.error:
            raise RuntimeError(self.error)
        lower, upper = self.bounds
        if not math.isfinite(position) or not lower <= position <= upper:
            raise ValueError(f'{self.args.joint}: {position} rad is outside joint limits [{lower}, {upper}]')
        return position

    def wire_position(self, position):
        if self.args.joint == 'head_pan':
            position = struct.unpack('f', struct.pack('f', position))[0]
        return self.check_position(position)

    def resolve_target(self, start):
        if self.args.distance is not None:
            return start + self.args.distance
        if self.args.position not in ('-end', '+end'):
            return self.args.position
        lower, upper = self.bounds
        target = lower if self.args.position == '-end' else upper
        return self.boundary_position(target)

    def boundary_position(self, target):
        """Represent an in-range boundary/start without rounding past a limit."""
        lower, upper = self.bounds
        if self.args.joint == 'head_pan':
            # Head commands use float32; select the nearest representable value
            # INSIDE the limit when rounding the endpoint would cross it.
            target = struct.unpack('f', struct.pack('f', target))[0]
            if target < lower or target > upper:
                direction = 1 if target < lower else -1
                bits = struct.unpack('I', struct.pack('f', target))[0]
                bits += direction if math.copysign(1., target) > 0 else -direction
                target = struct.unpack('f', struct.pack('I', bits))[0]
        return self.wire_position(target)

    def measured_position(self):
        if self.position_received is None or time.monotonic() - self.position_received > 1.:
            raise RuntimeError('Joint feedback stopped; cannot confirm the measured position')
        if self.error:
            raise RuntimeError(self.error)
        if not math.isfinite(self.position):
            raise ValueError('Non-finite joint-state position')
        lower, upper = self.bounds
        overshoot = max(lower-self.position, self.position-upper, 0.)
        if overshoot > JOINT_ANGLE_TOLERANCE:
            raise ValueError(f'{self.args.joint}: measured {self.position} rad is outside joint limits '
                             f'[{lower}, {upper}] by {overshoot:.6f} rad; '
                             f'feedback allowance is {JOINT_ANGLE_TOLERANCE:.6f} rad')
        return self.position  # Preserve raw feedback; command limits remain strict.

    def check_duration(self, start, target):
        self.minimum_time = minimum_duration(self.wire_position(target)-start,
                                             self.velocity_limit, self.acceleration_limit)
        if self.args.duration < self.minimum_time and not math.isclose(
                self.args.duration, self.minimum_time, rel_tol=1e-9, abs_tol=1e-9):
            suggested = math.ceil(self.minimum_time*1000)/1000
            message = (f'Time {self.args.duration:g} s is too short for {self.args.joint}; '
                       f'minimum quintic time is {self.minimum_time:.6f} s. '
                       f'Use --time {suggested:.3f} or longer.')
            if self.pub is None:
                message += ' No commands sent.'
            raise ValueError(message)

    def print_plan(self, start, target):
        print(f'{self.args.joint}: {start:.6f} -> {target:.6f} rad in {self.args.duration:g} s', flush=True)
        if self.reference_start != start:
            print(f'Quintic reference starts at the in-limit position {self.reference_start:.6f} rad', flush=True)
        if self.args.position in ('-end', '+end'):
            print(f'Joint limits: {self.bounds[0]:.6f} to {self.bounds[1]:.6f} rad '
                  f'(selected {self.args.position})', flush=True)
        detail = '' if self.acceleration_limit is not None else ' (velocity only; acceleration limit unavailable)'
        print(f'Minimum quintic time: {self.minimum_time:.6f} s{detail}', flush=True)

    def already_at_target(self, target):
        if abs(self.measured_position() - target) > self.args.tolerance:
            return False
        begin = time.monotonic()
        sequence = self.position_sequence
        while rclpy.ok() and time.monotonic() - begin < .2:
            rclpy.spin_once(self, timeout_sec=.01)
            if abs(self.measured_position() - target) > self.args.tolerance:
                return False
        return rclpy.ok() and self.position_sequence > sequence

    def save_report(self, start, target, samples, elapsed, errors, status):
        measured = self.measured_position()
        report = dict(joint=self.args.joint, status=status, start_rad=start, target_rad=target,
                      measured_rad=measured, final_error_rad=abs(measured-target), samples=samples,
                      tolerance_rad=self.args.tolerance,
                      reference_start_rad=self.reference_start,
                      limit_overshoot_rad=max(self.bounds[0]-measured, measured-self.bounds[1], 0.),
                      trajectory_seconds=self.args.duration, elapsed_seconds=elapsed,
                      minimum_time_seconds=self.minimum_time,
                      settle_seconds=max(0., elapsed-self.args.duration) if samples else 0.,
                      maximum_tracking_error_rad=max(errors, default=abs(measured-target)),
                      rms_tracking_error_rad=math.sqrt(sum(e*e for e in errors)/len(errors)) if errors else 0.)
        if self.args.joint == 'head_pan':
            report['head_state'] = None
            if self.head_state is not None:
                state = self.head_state
                age = time.monotonic() - self.head_state_received
                report['head_state'] = dict(pan=state.pan, pan_mode=state.pan_mode,
                                           is_turning=state.is_turning, is_blocked=state.is_blocked,
                                           age_seconds=age, fresh=age <= 1.)
                if status == 'not_reached' and age <= 1. and state.pan_mode == HeadState.PASSIVE_MODE:
                    print('Head reports PASSIVE_MODE (0); requested ACTIVE_MODE (1) was not confirmed.', flush=True)
        print(json.dumps(report, indent=2), flush=True)
        if self.args.report:
            self.args.report.write_text(json.dumps(report, indent=2)+'\n')

    def publish_position(self, position):
        position = self.wire_position(position)
        if self.args.joint == 'head_pan':
            msg = HeadPanCommand(target=position, speed_ratio=1.0, pan_mode=HeadPanCommand.SET_ACTIVE_MODE)
        else:
            msg = JointCommand()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.mode = JointCommand.POSITION_MODE
            msg.names = [self.args.joint]
            msg.position = [position]
        self.pub.publish(msg)

    def run(self):
        deadline = time.monotonic() + 10
        while rclpy.ok() and (self.position is None or self.bounds is None or self.velocity_limit is None):
            if self.error:
                raise RuntimeError(self.error)
            if time.monotonic() >= deadline:
                raise RuntimeError('Missing joint position or position/velocity limits; start the Sawyer bridge first')
            rclpy.spin_once(self, timeout_sec=0.05)
        if not rclpy.ok():
            raise RuntimeError('ROS stopped before joint feedback arrived')
        start = self.measured_position()
        self.reference_start = self.boundary_position(min(self.bounds[1], max(self.bounds[0], start)))
        target = self.resolve_target(start)
        self.wire_position(target)
        self.check_duration(start, target)
        if self.args.dry_run:
            self.print_plan(start, target)
            print('Dry run: no commands sent.')
            return
        if self.args.position is not None and self.already_at_target(target):
            print(f'{self.args.joint} is already at {target:.6f} rad; no commands sent.')
            self.save_report(start, target, 0, 0., [], 'already_at_target')
            return
        head = self.args.joint == 'head_pan'
        topic = '/robot/head/command_head_pan' if head else '/robot/limb/right/joint_command'
        self.pub = self.create_publisher(HeadPanCommand if head else JointCommand, topic, 1)
        publishers = [self.pub]
        if not head:
            self.speed_pub = self.create_publisher(Float64, '/robot/limb/right/set_speed_ratio', 1)
            publishers.append(self.speed_pub)
        deadline = time.monotonic() + 5
        while rclpy.ok() and not all(p.get_subscription_count() for p in publishers):
            if time.monotonic() >= deadline:
                raise RuntimeError(f'No bridge subscriber on {topic}; enable its command path')
            rclpy.spin_once(self, timeout_sec=0.02)
        if not rclpy.ok():
            raise RuntimeError('ROS stopped before the command bridge connected')
        start = self.measured_position()
        target = self.resolve_target(start)
        self.check_duration(start, target)
        if not head:
            # POSITION_MODE is rate-limited by this persistent SDK setting. Set
            # it explicitly so a previous 0.05 ratio cannot silently slow this run.
            self.speed_pub.publish(Float64(data=1.0))
            deadline = time.monotonic() + .2
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.01)
        # Use feedback after discovery/settings, never an earlier printed angle.
        start = self.measured_position()
        self.reference_start = self.boundary_position(min(self.bounds[1], max(self.bounds[0], start)))
        target = self.resolve_target(start)
        self.wire_position(target)
        self.check_duration(start, target)
        self.print_plan(start, target)
        begin = next_tick = time.monotonic()
        next_progress = begin + 1.
        samples = 0
        errors = []
        reached = False
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=max(0., min(0.005, next_tick - time.monotonic())))
            now = time.monotonic()
            if now < next_tick:
                continue
            measured = self.measured_position()
            self.wire_position(target)
            self.check_duration(start, target)
            position = quintic_position(self.reference_start, target, now - begin, self.args.duration)
            errors.append(abs(measured - position))
            self.publish_position(position)
            samples += 1
            next_tick = now + 0.01  # 100 Hz position reference.
            if now >= next_progress:
                elapsed = now - begin
                stage = (f'{min(elapsed, self.args.duration):.1f}/{self.args.duration:g} s'
                         if elapsed <= self.args.duration or self.args.settle_timeout == 0 else
                         f'settling {elapsed-self.args.duration:.1f}/{self.args.settle_timeout:g} s')
                print(f'{self.args.joint}: {stage}, measured {measured:.6f} rad, '
                      f'target error {abs(measured-target):.6f} rad', flush=True)
                next_progress = now + 1.
            if now - begin >= self.args.duration:
                # Fresh measured arrival, with no extra wait by default. A user
                # may opt into holding the SAME target for additional settling.
                if abs(measured - target) <= self.args.tolerance:
                    reached = True
                    break
                if now - begin >= self.args.duration + self.args.settle_timeout:
                    break
        else:
            raise RuntimeError('ROS stopped during the trajectory')
        self.save_report(start, target, samples, time.monotonic()-begin, errors,
                         'reached' if reached else 'not_reached')
        if not reached:
            raise RuntimeError(f'Target {target:.6f} rad was not reached; measured {self.position:.6f} rad')


def main(argv=None):
    args = parse_args(argv)
    node = None
    try:
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = JointMover(args)
        node.run()
        return 0
    except KeyboardInterrupt:
        print('Stopped publishing commands.')
        return 130
    except (ValueError, RuntimeError, OSError) as error:
        print(f'Error: {error}')
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
