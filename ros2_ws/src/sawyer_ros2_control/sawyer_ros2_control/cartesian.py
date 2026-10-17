"""Read/capture tool poses and validate IK without publishing robot commands."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from intera_core_msgs.msg import JointLimits
from intera_core_msgs.srv import SolvePositionFK, SolvePositionIK
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from .safety import JOINTS, finite, live_limits, validate_positions

SERVICE_ROOT = '/ExternalTools/right/PositionKinematicsNode/'
DEFAULT_TIP = 'right_gripper_tip'


def checked_joints(message):
    names, positions = list(message.name), list(message.position)
    if len(names) != len(set(names)) or len(names) != len(positions):
        raise ValueError('Malformed or duplicate joint positions')
    if set(names) != set(JOINTS):
        raise ValueError('IK must return exactly right_j0 through right_j6')
    return {name: finite(value) for name, value in zip(names, positions)}


def checked_pose(data, tip=None):
    if not isinstance(data, dict) or data.get('frame_id') != 'base':
        raise ValueError('Pose frame_id must be base; transforms from other frames are not implemented')
    name = data.get('tip_name')
    if not isinstance(name, str) or not name.strip():
        raise ValueError('Pose requires an explicit tip_name')
    if tip is not None and name != tip:
        raise ValueError(f'Pose tip_name {name} does not match requested tip {tip}')
    position, orientation = data.get('position'), data.get('orientation_xyzw')
    if not isinstance(position, (list, tuple)) or len(position) != 3:
        raise ValueError('Pose position must contain x, y, z in metres')
    if not isinstance(orientation, (list, tuple)) or len(orientation) != 4:
        raise ValueError('Pose orientation_xyzw must contain quaternion x, y, z, w')
    position = [finite(value) for value in position]
    orientation = [finite(value) for value in orientation]
    norm = math.hypot(*orientation)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Pose quaternion must be finite and nonzero')
    return dict(frame_id='base', tip_name=name, position=position,
                orientation_xyzw=[value / norm for value in orientation])


def pose_message(data):
    pose = PoseStamped()
    pose.header.frame_id = data['frame_id']
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = data['position']
    (pose.pose.orientation.x, pose.pose.orientation.y,
     pose.pose.orientation.z, pose.pose.orientation.w) = data['orientation_xyzw']
    return pose


def pose_data(pose, tip):
    return checked_pose(dict(frame_id=pose.header.frame_id, tip_name=tip,
        position=[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
        orientation_xyzw=[pose.pose.orientation.x, pose.pose.orientation.y,
                          pose.pose.orientation.z, pose.pose.orientation.w]))


def pose_error(first, second):
    distance = math.dist(first['position'], second['position'])
    # q and -q describe the same rotation.
    dot = abs(sum(a * b for a, b in zip(first['orientation_xyzw'], second['orientation_xyzw'])))
    return distance, 2 * math.acos(min(1., max(0., dot)))


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    # Captured pick/place poses should survive an accidental repeated command.
    with path.open('x') as stream:
        stream.write(json.dumps(data, indent=2, allow_nan=False) + '\n')


class Cartesian(Node):
    def __init__(self, args):
        super().__init__('sawyer_cartesian')
        self.args = args
        self.positions, self.velocities, self.received = {}, {}, {}
        self.limits_message = None
        self.fault = None
        self.create_subscription(JointState, '/robot/joint_states', self.receive_joints, qos_profile_sensor_data)
        self.create_subscription(JointLimits, '/robot/joint_limits', self.receive_limits,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.fk_client = self.create_client(SolvePositionFK, SERVICE_ROOT + 'FKService')
        self.ik_client = self.create_client(SolvePositionIK, SERVICE_ROOT + 'IKService')

    def receive_joints(self, message):
        if not set(message.name).intersection(JOINTS):
            return
        try:
            if len(message.name) != len(set(message.name)) or len(message.name) != len(message.position):
                raise ValueError('Malformed arm joint feedback')
            if len(message.velocity) != len(message.name):
                raise ValueError('Arm joint velocities are required for pose capture')
            now = time.monotonic()
            for name, position, velocity in zip(message.name, message.position, message.velocity):
                if name in JOINTS:
                    self.positions[name], self.velocities[name] = finite(position), finite(velocity)
                    self.received[name] = now
        except ValueError as error:
            self.fault = str(error)

    def receive_limits(self, message):
        self.limits_message = message

    def wait_until(self, predicate, description):
        deadline = time.monotonic() + self.args.timeout
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.fault:
                raise RuntimeError(self.fault)
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(f'Timed out waiting for {description}')
        raise RuntimeError('ROS shut down')

    def snapshot(self):
        self.wait_until(lambda: set(self.positions) == set(JOINTS), 'all seven arm joints')
        if time.monotonic() - min(self.received.values()) > 1.:
            raise RuntimeError('Arm joint feedback is stale')
        if max(abs(value) for value in self.velocities.values()) > 0.02:
            raise RuntimeError('Keep the arm stationary while capturing a pose or checking IK')
        return dict(self.positions)

    def service(self, client, request):
        if not client.wait_for_service(timeout_sec=self.args.timeout):
            raise RuntimeError(f'Service {client.srv_name} unavailable; restart the bridge with kinematics:=true')
        future = client.call_async(request)
        try:
            self.wait_until(future.done, client.srv_name)
            response = future.result()
            if response is None:
                raise RuntimeError(f'{client.srv_name} returned no response')
            return response
        finally:
            if not future.done():
                client.remove_pending_request(future)
                future.cancel()

    def fk(self, joints, tip):
        request = SolvePositionFK.Request(
            configuration=[JointState(name=list(JOINTS), position=[joints[name] for name in JOINTS])],
            tip_names=[tip])
        response = self.service(self.fk_client, request)
        if len(response.pose_stamp) != 1 or len(response.is_valid) != 1 or len(response.in_collision) != 1:
            raise RuntimeError('Malformed FK response')
        if not response.is_valid[0]:
            raise RuntimeError(f'FK rejected tip {tip} or the joint configuration')
        if response.in_collision[0]:
            raise RuntimeError('FK reports self-collision')
        return pose_data(response.pose_stamp[0], tip)

    def ik(self, pose, seed):
        self.wait_until(lambda: self.limits_message is not None, 'live joint limits')
        model = json.loads((Path(get_package_share_directory('sawyer_ros2_control')) / 'joint_limits.json').read_text())['joints']
        limits = live_limits(self.limits_message, model)
        request = SolvePositionIK.Request(pose_stamp=[pose_message(pose)], tip_names=[pose['tip_name']],
            seed_mode=SolvePositionIK.Request.SEED_USER,
            seed_angles=[JointState(name=list(JOINTS), position=[seed[name] for name in JOINTS])])
        request.pose_stamp[0].header.stamp = self.get_clock().now().to_msg()
        response = self.service(self.ik_client, request)
        if len(response.result_type) != 1:
            raise RuntimeError('Malformed IK result_type')
        code = response.result_type[0]
        if code <= 0:
            reason = {-1: 'unreachable pose', -2: 'self-collision', -3: 'unknown tool frame'}.get(code, 'invalid result')
            raise RuntimeError(f'IK rejected the pose: {reason} ({code})')
        if len(response.joints) != 1:
            raise RuntimeError('Malformed IK joints')
        joints = checked_joints(response.joints[0])
        validate_positions(joints, limits)
        achieved = self.fk(joints, pose['tip_name'])
        distance, angle = pose_error(pose, achieved)
        if distance > 0.005 or angle > 0.02:
            raise RuntimeError(f'FK does not match requested IK pose: {distance:.6f} m, {angle:.6f} rad')
        return dict(status='ik_valid', pose=pose, joints=joints,
                    position_error_m=distance, orientation_error_rad=angle,
                    validation_scope='Kinematics and controller self-collision only; no workspace obstacle check',
                    robot_commands_published=False)

    def run(self):
        start = self.snapshot()
        if self.args.operation == 'ik':
            pose = checked_pose(json.loads(self.args.pose.read_text()), self.args.tip)
            result = self.ik(pose, start)
        else:
            result = dict(schema_version=1, **self.fk(start, self.args.tip or DEFAULT_TIP),
                          joints=start, captured_at_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                          robot_commands_published=False)
        current = self.snapshot()
        if max(abs(current[name] - start[name]) for name in JOINTS) > 0.002:
            raise RuntimeError('Arm moved during the request; keep it stationary and retry')
        if self.args.output is not None:
            write_json(self.args.output, result)
        return result


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', nargs='?', default='read', choices=('read', 'capture', 'ik'))
    parser.add_argument('--tip', help=f'Tool link (default for read/capture: {DEFAULT_TIP}; IK uses pose file)')
    parser.add_argument('--pose', type=Path, help='Captured pose JSON for IK validation')
    parser.add_argument('--output', type=Path, help='Write a new JSON file; existing files are never overwritten')
    parser.add_argument('--timeout', type=float, default=10.)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('timeout must be positive and finite')
    if (args.operation == 'ik') != (args.pose is not None):
        parser.error('ik requires --pose FILE; read/capture do not accept --pose')
    if args.operation == 'capture' and args.output is None:
        parser.error('capture requires --output FILE')
    if args.output is not None and args.output.exists():
        parser.error(f'Output already exists: {args.output}; choose a new filename')
    return args


def main(argv=None):
    args = parse_args(argv)
    node = None
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = Cartesian(args)
        print(json.dumps(node.run(), indent=2, allow_nan=False))
        return 0
    except KeyboardInterrupt:
        return 130
    except (ValueError, RuntimeError, KeyError, TypeError, OSError) as error:
        print(f'Cartesian: {error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
