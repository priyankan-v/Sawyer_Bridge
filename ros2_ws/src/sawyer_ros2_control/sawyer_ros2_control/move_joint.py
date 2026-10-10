"""Move one arm joint or head_pan along a quintic position reference (radians)."""
import argparse
import json
import math
import os
import struct
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.signals import SignalHandlerOptions
from intera_core_msgs.msg import HeadPanCommand, HeadState, JointCommand
from std_msgs.msg import Float64

from .command import Controller
from .safety import JOINTS, SELECTABLE_JOINTS, JointQuintic, finite, single_joint_limit


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--joint', required=True, choices=SELECTABLE_JOINTS)
    destination = parser.add_mutually_exclusive_group(required=True)
    destination.add_argument('--distance', type=float, help='Signed displacement from current joint angle, in radians')
    destination.add_argument('--position', type=float, help='Absolute target joint angle, in radians')
    parser.add_argument('--time', '--duration', dest='duration', type=float, required=True, help='Quintic reference duration, seconds')
    parser.add_argument('--execute', action='store_true', help='Publish commands; otherwise only validate and display the plan')
    parser.add_argument('--operator-approved', action='store_true', help='Confirm explicit operator approval for this physical movement')
    parser.add_argument('--max-displacement', type=float, default=0.02, help='Reviewed per-move displacement cap, default 0.02 rad')
    parser.add_argument('--max-velocity', type=float, default=0.1)
    parser.add_argument('--max-acceleration', type=float, default=0.2)
    parser.add_argument('--speed-ratio', type=float, default=0.05)
    parser.add_argument('--rate', type=float, default=100)
    parser.add_argument('--wait', type=float, default=10)
    parser.add_argument('--feedback-timeout', type=float, default=1)
    parser.add_argument('--command-timeout', type=float, default=0.2)
    parser.add_argument('--max-tracking-error', type=float, default=0.05)
    parser.add_argument('--goal-tolerance', type=float, default=0.002)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args(argv)
    if args.execute and not args.operator_approved:
        parser.error('Physical motion requires both --execute and --operator-approved')
    try:
        for name in ('duration', 'max_displacement', 'max_velocity', 'max_acceleration', 'speed_ratio',
                     'rate', 'wait', 'feedback_timeout', 'command_timeout', 'max_tracking_error', 'goal_tolerance'):
            if finite(getattr(args, name)) <= 0:
                raise ValueError(f'{name} must be positive')
        finite(args.distance if args.distance is not None else args.position)
        if not 50 <= args.rate <= 200 or args.speed_ratio > 0.1:
            raise ValueError('Rate must be 50–200 Hz; speed ratio must be at most 0.1')
        if not 3 / args.rate <= args.command_timeout <= 0.2:
            raise ValueError('Command timeout must cover at least three periods and be at most 0.2 s')
    except ValueError as error:
        parser.error(str(error))
    args.operation = 'one'  # Reuse the existing controller's safety-feedback requirements.
    return args


def execution_preflight(args):
    root = Path(os.environ['SAWYER_ROOT'])
    result = json.loads((root / 'docs/results/local_transport.json').read_text())
    if result.get('status') != 'PASS':
        raise RuntimeError('Run pixi run test-local successfully before physical execution')
    subprocess.run([str(root / 'scripts/env.sh'), 'ros1', 'python', str(root / 'scripts/preflight.py'),
                    '--require-ready', '--joint', args.joint, '--output', str(root / '.runtime/preflight.json')],
                   check=True, timeout=20)
    preflight = json.loads((root / '.runtime/preflight.json').read_text())
    if not preflight.get('ready') or preflight.get('errors') or not preflight.get('verified_command_subscribers') \
            or preflight['master'] != os.environ.get('ROS_MASTER_URI') \
            or preflight.get('joint') != args.joint or not 0 <= time.time() - preflight['wall_time'] <= 30:
        raise RuntimeError('Robot preflight is not valid for this joint/master')


class SingleJointController(Controller):
    def __init__(self, args):
        # Publisher creation is deferred until the plan and all safety checks pass.
        base_args = argparse.Namespace(**vars(args))
        base_args.execute = False
        super().__init__(base_args)
        self.args = args
        self.head = args.joint == 'head_pan'
        self.head_position = self.head_velocity = None
        self.started_at = None
        self.held_arm = None
        if self.head:
            self.create_subscription(HeadState, '/robot/head/head_state', lambda m: self.receive('head', m), 1)

    def on_joints(self, msg):
        super().on_joints(msg)
        if getattr(self, 'head', False) and 'head_pan' in msg.name:
            try:
                if len(msg.name) != len(set(msg.name)) or len(msg.name) != len(msg.position) or len(msg.velocity) != len(msg.name):
                    raise ValueError('Head joint feedback requires unique names, positions and velocities')
                index = msg.name.index('head_pan')
                self.head_position = finite(msg.position[index])
                self.head_velocity = finite(msg.velocity[index])
                self.received['head_joint'] = time.monotonic()
            except ValueError as error:
                self.fault = str(error)

    def wait_for_feedback(self):
        deadline = time.monotonic() + self.args.wait
        required = ('joints', 'state', 'collision', 'limits') + (('head', 'head_joint') if self.head else ())
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if all(key in self.data if key != 'head_joint' else key in self.received for key in required):
                return
        raise RuntimeError(f'Missing feedback for {self.args.joint}: {required}')

    def selected_position(self):
        return self.head_position if self.head else self.positions[self.args.joint]

    def selected_limit(self):
        return single_joint_limit(self.data['limits'], self.args.joint, self.urdf[self.args.joint],
                                  self.args.max_velocity, self.args.max_acceleration, self.args.speed_ratio)

    def check(self):
        arm_limits = super().check()
        if self.head:
            now = time.monotonic()
            for key in ('head', 'head_joint'):
                if now - self.received.get(key, -math.inf) > self.args.feedback_timeout:
                    raise RuntimeError(f'Missing or stale {key} feedback')
            state = self.data['head']
            if state.is_blocked:
                raise RuntimeError('Head controller reports blocked movement')
            if abs(finite(state.pan) - self.head_position) > 0.01:
                raise RuntimeError('HeadState and joint-state angles disagree')
            limit = self.selected_limit()
            if not limit.lower <= self.head_position <= limit.upper:
                raise RuntimeError('Measured head_pan outside verified limits')
            index = list(self.data['limits'].joint_names).index('head_pan')
            if abs(self.head_velocity) > min(self.data['limits'].velocity[index], self.urdf['head_pan']['velocity']):
                raise RuntimeError('Measured head velocity exceeds controller/URDF limits')
            if self.started_at is not None and now - self.started_at > 0.3 and state.pan_mode != HeadState.ACTIVE_MODE:
                raise RuntimeError('Head controller failed to enter or remain in ACTIVE_MODE')
        return arm_limits, self.selected_limit()

    def ensure_stationary(self):
        if max(abs(v) for v in self.velocities.values()) > 0.02:
            raise RuntimeError('Arm must be stationary before a single-joint movement')
        if self.head and (abs(self.head_velocity) > 0.02 or self.data['head'].is_turning):
            raise RuntimeError('Head must be stationary before planning')

    def make_publishers(self):
        qos = QoSProfile(depth=1, lifespan=Duration(seconds=self.args.command_timeout))
        if self.head:
            self.pub = self.create_publisher(HeadPanCommand, '/robot/head/command_head_pan', qos)
            publishers = [self.pub]
        else:
            self.pub = self.create_publisher(JointCommand, '/robot/limb/right/joint_command', qos)
            self.timeout_pub = self.create_publisher(Float64, '/robot/limb/right/joint_command_timeout', 1)
            self.speed_pub = self.create_publisher(Float64, '/robot/limb/right/set_speed_ratio', 1)
            publishers = [self.pub, self.timeout_pub, self.speed_pub]
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            self.check()
            if all(p.get_subscription_count() for p in publishers):
                return
        raise RuntimeError('Selected command bridge is not connected; start it with --commands or --head-commands')

    def publish_sample(self, position):
        limit = self.selected_limit()
        if not limit.lower <= finite(position) <= limit.upper:
            raise RuntimeError('Setpoint outside verified limits')
        if self.head:
            # The head API stores its target as float32; validate the actual wire value.
            wire_position = struct.unpack('f', struct.pack('f', position))[0]
            if not limit.lower <= wire_position <= limit.upper:
                raise RuntimeError('Rounded head target outside verified limits')
            self.pub.publish(HeadPanCommand(target=wire_position, speed_ratio=self.args.speed_ratio,
                                            pan_mode=HeadPanCommand.SET_ACTIVE_MODE))
        else:
            positions = dict(self.held_arm)
            positions[self.args.joint] = position
            self.publish_positions(positions)

    def safe_hold(self):
        if self.pub is None or self.started_at is None or not rclpy.ok():
            return
        try:
            self.check()
            if not self.head:
                self.held_arm = dict(self.positions)
            self.publish_sample(self.selected_position())
            rclpy.spin_once(self, timeout_sec=0.02)
        except (ValueError, RuntimeError, KeyError):
            pass

    def run(self):
        self.wait_for_feedback()
        limits = self.check()
        self.ensure_stationary()
        start = self.selected_position()
        target = start + self.args.distance if self.args.distance is not None else self.args.position
        trajectory = JointQuintic(start, target, self.args.duration, limits[1], self.args.max_displacement)
        if self.head:
            for value in (start, target):
                wire_value = struct.unpack('f', struct.pack('f', value))[0]
                if not limits[1].lower <= wire_value <= limits[1].upper:
                    raise ValueError('Head endpoint rounds outside verified bounds in float32')
        self.held_arm = dict(self.positions)
        print(json.dumps({'joint': self.args.joint, 'start_rad': start, 'target_rad': target,
                          'distance_rad': trajectory.delta, 'trajectory_seconds': trajectory.duration,
                          'peak_velocity_rad_s': trajectory.peak_velocity,
                          'peak_acceleration_rad_s2': trajectory.peak_acceleration,
                          'mode': 'head ACTIVE_MODE' if self.head else 'arm POSITION_MODE',
                          'execute': self.args.execute}, indent=2), flush=True)
        if not self.args.execute:
            return
        if self.get_parameter('use_sim_time').value:
            raise RuntimeError('Physical execution rejects simulated time')
        self.make_publishers()
        if not self.head:
            self.timeout_pub.publish(Float64(data=self.args.command_timeout))
            self.speed_pub.publish(Float64(data=self.args.speed_ratio))
            deadline = time.monotonic() + 0.3
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.01)
                self.check()
        self.ensure_stationary()
        if abs(self.selected_position() - start) > 0.002 or any(abs(self.positions[j] - self.held_arm[j]) > 0.002 for j in JOINTS):
            raise RuntimeError('Robot moved after planning; replan')
        begin = previous = next_tick = time.monotonic()
        period = 1 / self.args.rate
        errors, samples = [], 0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=max(0., min(0.005, next_tick - time.monotonic())))
            now = time.monotonic()
            if now < next_tick:
                continue
            if now - previous > self.args.command_timeout:
                raise RuntimeError('Command scheduling deadline missed')
            if self.check() != limits:
                raise RuntimeError('Joint limits changed during execution')
            for joint in JOINTS:
                if joint != self.args.joint and abs(self.positions[joint] - self.held_arm[joint]) > self.args.max_tracking_error:
                    raise RuntimeError(f'Unselected arm joint {joint} moved unexpectedly')
            position, _, _ = trajectory.sample(now - begin)
            error = abs(self.selected_position() - position)
            if error > self.args.max_tracking_error:
                raise RuntimeError('Selected joint tracking error exceeds limit')
            self.publish_sample(position)
            if self.started_at is None:
                self.started_at = now
            samples += 1
            errors.append(error)
            previous, next_tick = now, now + period
            if now - begin >= trajectory.duration:
                velocity = self.head_velocity if self.head else self.velocities[self.args.joint]
                if abs(self.selected_position() - target) <= self.args.goal_tolerance and abs(velocity) < 0.01:
                    break
                if now - begin > trajectory.duration + 3:
                    raise RuntimeError('Target did not settle within three seconds of the trajectory end')
        else:
            raise RuntimeError('ROS context stopped during trajectory execution')
        report = {'joint': self.args.joint, 'target_rad': target, 'measured_rad': self.selected_position(),
                  'final_error_rad': abs(self.selected_position() - target), 'samples': samples,
                  'elapsed_seconds': time.monotonic() - begin, 'maximum_tracking_error_rad': max(errors),
                  'rms_tracking_error_rad': math.sqrt(sum(e*e for e in errors) / len(errors))}
        print(json.dumps(report, indent=2), flush=True)
        if self.args.report:
            self.args.report.write_text(json.dumps(report, indent=2) + '\n')


def main(argv=None):
    args = parse_args(argv)
    node = None
    try:
        if args.execute:
            execution_preflight(args)
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = SingleJointController(args)
        node.run()
        return 0
    except KeyboardInterrupt:
        print('Interrupted; ceasing publication.')
        return 130
    except (ValueError, RuntimeError, KeyError, OSError, subprocess.SubprocessError) as error:
        print(f'Aborted: {error}')
        return 1
    finally:
        if node is not None:
            node.safe_hold()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
