"""Read current joints, plan one/all joints, and explicitly approve position-mode streaming."""
import argparse
import json
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from intera_core_msgs.msg import JointCommand, RobotAssemblyState, CollisionDetectionState, JointLimits
from .safety import JOINTS, Quintic, finite, live_limits, ready, validate_positions


class Controller(Node):
    def __init__(self, args):
        super().__init__('sawyer_guarded_position_control')
        self.args = args
        self.data, self.received = {}, {}
        self.positions, self.velocities = {}, {}
        self.joint_received = {}
        self.fault = None
        self.create_subscription(JointState, '/robot/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(RobotAssemblyState, '/robot/state', lambda m: self.receive('state', m), 1)
        self.create_subscription(CollisionDetectionState, '/robot/limb/right/collision_detection_state',
                                 lambda m: self.receive('collision', m), 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(JointLimits, '/robot/joint_limits', lambda m: self.receive('limits', m), qos)
        model = Path(get_package_share_directory('sawyer_ros2_control')) / 'joint_limits.json'
        self.urdf = json.loads(model.read_text())['joints']
        self.pub = None
        if args.execute:
            # Created only after explicit command-line approval has been checked.
            command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.VOLATILE,
                                     lifespan=Duration(seconds=args.command_timeout))
            self.pub = self.create_publisher(JointCommand, '/robot/limb/right/joint_command', command_qos)
            self.timeout_pub = self.create_publisher(Float64, '/robot/limb/right/joint_command_timeout', 1)
            self.speed_pub = self.create_publisher(Float64, '/robot/limb/right/set_speed_ratio', 1)

    def receive(self, key, msg):
        self.data[key] = msg
        self.received[key] = time.monotonic()

    def on_joints(self, msg):
        try:
            # Sawyer has multiple publishers on JointState (arm, head, end effector).
            # Merge arm subsets while tracking freshness separately for each joint.
            arm_names = set(msg.name).intersection(JOINTS)
            if not arm_names:
                return
            if len(msg.name) != len(set(msg.name)) or len(msg.name) != len(msg.position):
                raise ValueError('Malformed or duplicate joint states')
            if len(msg.velocity) != len(msg.name):
                raise ValueError('Joint velocities required for safety checking')
            positions = dict(zip(msg.name, msg.position))
            velocities = dict(zip(msg.name, msg.velocity))
            for j in arm_names:
                self.positions[j] = finite(positions[j])
                self.velocities[j] = finite(velocities[j])
                self.joint_received[j] = time.monotonic()
            if set(self.positions) == set(JOINTS):
                self.data['joints'] = msg
                self.received['joints'] = min(self.joint_received.values())
        except (ValueError, KeyError) as error:
            self.fault = str(error)

    def limits(self):
        return live_limits(self.data['limits'], self.urdf, self.args.max_velocity, self.args.max_acceleration)

    def check(self):
        if self.fault:
            raise RuntimeError(self.fault)
        now = time.monotonic()
        for key in ('joints', 'state', 'collision'):
            if now - self.received.get(key, -math.inf) > self.args.feedback_timeout:
                raise RuntimeError(f'Missing or stale {key} feedback')
        if not ready(self.data.get('state'), self.data.get('collision')):
            raise RuntimeError('Robot readiness or collision check failed')
        limits = self.limits()
        validate_positions(self.positions, limits)
        for joint in JOINTS:
            # The application cap bounds setpoints; controller limits bound measured motion.
            index = list(self.data['limits'].joint_names).index(joint)
            hard_velocity = min(self.data['limits'].velocity[index], self.urdf[joint]['velocity'])
            if abs(self.velocities[joint]) > hard_velocity:
                raise RuntimeError(f'{joint}: measured velocity exceeds limit')
        return limits

    def wait_for_feedback(self):
        deadline = time.monotonic() + self.args.wait
        required = ('joints',) if self.args.operation == 'read' else ('joints', 'state', 'collision', 'limits')
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(key in self.data for key in required):
                if self.fault:
                    raise RuntimeError(self.fault)
                return
        raise RuntimeError(f'Feedback not available: {required}')

    def publish_positions(self, positions):
        validate_positions(positions, self.limits())
        msg = JointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.names = list(JOINTS)
        msg.mode = JointCommand.POSITION_MODE
        msg.position = [positions[j] for j in JOINTS]
        # velocity and acceleration are intentionally empty in POSITION_MODE.
        self.pub.publish(msg)

    def safe_hold(self):
        """One measured-position hold only with healthy feedback; then cease publishing."""
        if self.pub is None or not rclpy.ok():
            return
        try:
            self.check()
            self.publish_positions(self.positions)
            rclpy.spin_once(self, timeout_sec=0.02)
        except (ValueError, RuntimeError, KeyError):
            pass  # No further commands when safety data is invalid.

    def run(self):
        self.wait_for_feedback()
        if self.args.operation == 'read':
            print(json.dumps(self.positions, indent=2))
            return
        limits = self.check()
        if max(abs(v) for v in self.velocities.values()) > 0.02:
            raise RuntimeError('Arm must be stationary before planning')
        start = dict(self.positions)
        target = dict(start)
        if self.args.operation == 'one':
            if self.args.joint not in JOINTS or self.args.position is None:
                raise ValueError('one requires --joint right_jN and --position RADIANS')
            target[self.args.joint] = finite(self.args.position)
        else:
            if self.args.target is None:
                raise ValueError('configuration/quintic requires --target JSON_FILE')
            target = json.loads(self.args.target.read_text())
        trajectory = Quintic(start, target, self.args.duration, limits, self.args.max_displacement)
        print(json.dumps({'start': start, 'target': target, 'duration': self.args.duration,
                          'mode': 'POSITION_MODE', 'execute': self.args.execute,
                          'midpoint_position_velocity_acceleration': trajectory.sample(self.args.duration / 2)}, indent=2))
        if not self.args.execute:
            return
        if self.get_parameter('use_sim_time').value:
            raise RuntimeError('Hardware execution rejects simulated time')
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if all(p.get_subscription_count() for p in (self.pub, self.timeout_pub, self.speed_pub)):
                break
        else:
            raise RuntimeError('Command/timeout/speed bridges must all be connected')
        self.check()
        if max(abs(start[j] - self.positions[j]) for j in JOINTS) > 0.002:
            raise RuntimeError('Arm moved after trajectory was planned; replan')
        self.timeout_pub.publish(Float64(data=self.args.command_timeout))
        self.speed_pub.publish(Float64(data=self.args.speed_ratio))
        # Give settings time to reach the robot before the first setpoint.
        setting_deadline = time.monotonic() + 0.3
        while time.monotonic() < setting_deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            self.check()
        if max(abs(start[j] - self.positions[j]) for j in JOINTS) > 0.002 \
                or max(abs(v) for v in self.velocities.values()) > 0.02:
            raise RuntimeError('Arm moved while applying controller settings; replan')
        begin = previous = time.monotonic()
        period = 1 / self.args.rate
        next_tick = begin
        errors, count = [], 0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=max(0.0, min(0.005, next_tick - time.monotonic())))
            now = time.monotonic()
            if now < next_tick:
                continue
            if now - previous > self.args.command_timeout:
                raise RuntimeError('Command scheduling deadline missed')
            if self.check() != limits:
                raise RuntimeError('Joint limits changed during execution')
            positions, velocity, acceleration = trajectory.sample(now - begin)
            error = max(abs(self.positions[j] - positions[j]) for j in JOINTS)
            if error > self.args.max_tracking_error:
                raise RuntimeError(f'Tracking error {error:.6f} rad exceeds limit')
            self.publish_positions(positions)
            errors.append(error)
            count += 1
            previous = now
            next_tick = now + period
            if now - begin >= self.args.duration:
                goal_error = max(abs(self.positions[j] - target[j]) for j in JOINTS)
                if goal_error <= self.args.goal_tolerance and max(abs(v) for v in self.velocities.values()) < 0.01:
                    break
                if now - begin > self.args.duration + 3:
                    raise RuntimeError(f'Goal did not settle: {goal_error:.6f} rad')
        report = {'samples': count, 'elapsed_seconds': time.monotonic() - begin,
                  'maximum_tracking_error_rad': max(errors),
                  'rms_tracking_error_rad': math.sqrt(sum(e*e for e in errors)/len(errors)),
                  'final_positions': self.positions}
        print(json.dumps(report, indent=2))
        if self.args.report:
            self.args.report.write_text(json.dumps(report, indent=2) + '\n')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('read', 'one', 'configuration', 'quintic'))
    parser.add_argument('--joint')
    parser.add_argument('--position', type=float)
    parser.add_argument('--target', type=Path)
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--operator-approved', action='store_true')
    parser.add_argument('--rate', type=float, default=100)
    parser.add_argument('--wait', type=float, default=10)
    parser.add_argument('--feedback-timeout', type=float, default=1.0)
    parser.add_argument('--command-timeout', type=float, default=0.2)
    parser.add_argument('--max-velocity', type=float, default=0.1)
    parser.add_argument('--max-acceleration', type=float, default=0.2)
    parser.add_argument('--max-displacement', type=float, default=0.02)
    parser.add_argument('--max-tracking-error', type=float, default=0.05)
    parser.add_argument('--goal-tolerance', type=float, default=0.002)
    parser.add_argument('--speed-ratio', type=float, default=0.05)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args(argv)
    if args.execute and (not args.operator_approved or args.operation == 'read'):
        parser.error('Physical motion requires --execute --operator-approved with a motion operation')
    for name in ('duration', 'rate', 'wait', 'feedback_timeout', 'command_timeout', 'max_velocity',
                 'max_acceleration', 'max_displacement', 'max_tracking_error', 'goal_tolerance', 'speed_ratio'):
        if finite(getattr(args, name)) <= 0:
            parser.error(f'{name} must be positive and finite')
    if not 50 <= args.rate <= 200 or not 0 < args.speed_ratio <= 0.1:
        parser.error('Rate must be 50–200 Hz and initial speed ratio at most 0.1')
    if args.command_timeout > 0.2 or args.command_timeout < 3 / args.rate:
        parser.error('Command timeout must cover at least three periods and be at most 0.2 seconds')
    if args.execute:
        from sawyer_ros2_control.runtime import state_directory, transport_report
        preflight = json.loads((state_directory() / 'preflight.json').read_text())
        transport = json.loads(transport_report().read_text())
        age = time.time() - preflight['wall_time']
        if not 0 <= age <= 30 or not preflight.get('ready') or preflight['errors'] \
                or preflight['master'] != os.environ.get('ROS_MASTER_URI') \
                or not preflight['verified_command_subscribers'] or transport.get('status') != 'PASS':
            parser.error('A fresh successful robot preflight and isolated forwarding test are required')
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Controller(args)
    result = 0
    try:
        node.run()
    except KeyboardInterrupt:
        print('Interrupted; ceasing command publication.')
        result = 130
    except (ValueError, RuntimeError, KeyError) as error:
        print(f'Aborted: {error}')
        result = 1
    finally:
        node.safe_hold()
        node.destroy_node()
        rclpy.try_shutdown()
    return result
