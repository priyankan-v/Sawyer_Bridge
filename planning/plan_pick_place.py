#!/usr/bin/env python3
"""Plan a taught-pose pick/place sequence through MoveIt; never execute motion."""
import argparse
import copy
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionIK, GetStateValidity
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState

ROOT = Path(__file__).resolve().parents[1]
ARM = tuple(f'right_j{i}' for i in range(7))
NAMESPACE = '/sawyer_planning/'


def read_pose(path):
    data = json.loads(path.read_text())
    if data.get('frame_id') != 'base' or data.get('tip_name') != 'right_gripper_tip':
        raise ValueError('Use captured base/right_gripper_tip poses')
    p, q = data['position'], data['orientation_xyzw']
    if len(p) != 3 or len(q) != 4 or any(type(v) not in (int, float) or not math.isfinite(v) for v in [*p, *q]):
        raise ValueError('Pose must contain finite xyz and quaternion xyzw values')
    norm = math.hypot(*q)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Zero pose quaternion')
    message = PoseStamped()
    message.header.frame_id = 'base'
    message.pose.position.x, message.pose.position.y, message.pose.position.z = p
    (message.pose.orientation.x, message.pose.orientation.y,
     message.pose.orientation.z, message.pose.orientation.w) = [v / norm for v in q]
    return message


def robot_state(positions):
    return RobotState(joint_state=JointState(name=sorted(positions), position=[positions[name] for name in sorted(positions)]), is_diff=False)


class PickPlacePlanner(Node):
    def __init__(self, args):
        super().__init__('sawyer_pick_place_planner')
        self.args = args
        self.positions = None
        self.received = None
        self.state_source = 'unknown'
        model = ET.parse(args.model / 'sawyer.urdf').getroot()
        self.bounds = {}
        for joint in model.findall('joint'):
            if joint.get('type') in ('revolute', 'prismatic') and joint.find('mimic') is None:
                limit = joint.find('limit')
                self.bounds[joint.get('name')] = (float(limit.get('lower')), float(limit.get('upper')))
        self.create_subscription(JointState, NAMESPACE + 'joint_states', self.receive, 1)
        self.service_clients = {name: self.create_client(typ, NAMESPACE + name) for name, typ in (
            ('check_state_validity', GetStateValidity), ('compute_ik', GetPositionIK),
            ('plan_kinematic_path', GetMotionPlan), ('compute_cartesian_path', GetCartesianPath))}

    def receive(self, message):
        if len(message.name) != len(message.position) or len(message.name) != len(set(message.name)):
            self.positions = None
            return
        self.positions = dict(zip(message.name, message.position))
        self.received = time.monotonic()
        self.state_source = 'live_feedback' if message.header.frame_id == 'base' else 'offline_snapshot'

    def call(self, name, request):
        client = self.service_clients[name]
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError(f'Missing MoveIt service {name}; launch planning/moveit.launch.py in system-ros2')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.)
        if not future.done():
            client.remove_pending_request(future)
            future.cancel()
            raise RuntimeError(f'MoveIt service timed out: {name}')
        result = future.result()
        if result is None:
            raise RuntimeError(f'MoveIt service returned no response: {name}')
        return result

    def validate(self, positions):
        for name, (lower, upper) in self.bounds.items():
            value = positions.get(name)
            if value is None or not math.isfinite(value) or not lower <= value <= upper:
                raise RuntimeError(f'{name}: measured/planned {value} outside [{lower}, {upper}]; '
                                   'resolve the joint state through the operator procedure before planning')
        response = self.call('check_state_validity', GetStateValidity.Request(robot_state=robot_state(positions)))
        if not response.valid:
            pairs = sorted({(contact.contact_body_1, contact.contact_body_2) for contact in response.contacts})
            raise RuntimeError(f'State rejected by collision/constraint checker: {pairs}')

    def endpoint(self, trajectory, start):
        if not trajectory.joint_trajectory.points or set(trajectory.joint_trajectory.joint_names) != set(ARM):
            raise RuntimeError('Planner returned an empty trajectory or incorrect joint group')
        result = dict(start)
        point = trajectory.joint_trajectory.points[-1]
        if len(point.positions) != len(ARM):
            raise RuntimeError('Malformed planned joint endpoint')
        result.update(zip(trajectory.joint_trajectory.joint_names, point.positions))
        self.validate(result)
        return result

    def joint_move(self, name, start, goal):
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'right_arm'
        request.ik_request.ik_link_name = 'right_gripper_tip'
        request.ik_request.robot_state = robot_state(start)
        request.ik_request.pose_stamped = goal
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 1
        result = self.call('compute_ik', request)
        if result.error_code.val != 1:
            raise RuntimeError(f'{name}: no valid collision-free IK ({result.error_code.val})')
        target = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
        target = dict(start, **{joint: target[joint] for joint in ARM})
        self.validate(target)
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name, motion.pipeline_id, motion.planner_id = 'right_arm', 'ompl', 'RRTConnect'
        motion.start_state = robot_state(start)
        motion.allowed_planning_time = 5.
        motion.num_planning_attempts = 3
        motion.max_velocity_scaling_factor = .1
        motion.max_acceleration_scaling_factor = .1
        motion.workspace_parameters.header.frame_id = 'base'
        for point, value in ((motion.workspace_parameters.min_corner, -2.), (motion.workspace_parameters.max_corner, 2.)):
            point.x = point.y = point.z = value
        motion.goal_constraints = [Constraints(joint_constraints=[JointConstraint(joint_name=joint,
            position=target[joint], tolerance_above=.001, tolerance_below=.001, weight=1.) for joint in ARM])]
        response = self.call('plan_kinematic_path', request).motion_plan_response
        if response.error_code.val != 1:
            raise RuntimeError(f'{name}: motion planning failed ({response.error_code.val})')
        end = self.endpoint(response.trajectory, start)
        return dict(kind='arm', name=name, trajectory=message_to_ordereddict(response.trajectory)), end

    def cartesian_move(self, name, start, goal):
        request = GetCartesianPath.Request(header=goal.header, start_state=robot_state(start),
            group_name='right_arm', link_name='right_gripper_tip', waypoints=[goal.pose],
            max_step=.005, revolute_jump_threshold=.15, avoid_collisions=True)
        response = self.call('compute_cartesian_path', request)
        if response.error_code.val != 1 or response.fraction < .999999:
            raise RuntimeError(f'{name}: Cartesian path incomplete ({response.fraction:.3f}, code {response.error_code.val})')
        end = self.endpoint(response.solution, start)
        return dict(kind='arm', name=name, trajectory=message_to_ordereddict(response.solution)), end

    def run(self):
        pick, place = read_pose(self.args.pick), read_pose(self.args.place)
        deadline = time.monotonic() + 5.
        while rclpy.ok() and self.positions is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
        if self.positions is None or time.monotonic() - self.received > 1.:
            raise RuntimeError('No fresh planning joint state; check the joint-state relay')
        start = dict(self.positions)
        self.validate(start)
        stages = [dict(kind='gripper', name='open_before_pick', operation='open')]
        current = dict(start)
        # Plan with the open hand before approaching; actual grasp width and carried
        # object geometry must be incorporated before physical task execution.
        current['right_gripper_l_finger_joint'] = self.bounds['right_gripper_l_finger_joint'][1]
        self.validate(current)
        def above(pose):
            result = copy.deepcopy(pose)
            result.pose.position.x += self.args.clearance * self.args.approach[0]
            result.pose.position.y += self.args.clearance * self.args.approach[1]
            result.pose.position.z += self.args.clearance * self.args.approach[2]
            return result

        pre_pick, pre_place = above(pick), above(place)
        for name, goal, method in [('pre_pick', pre_pick, self.joint_move), ('pick', pick, self.cartesian_move),
                                   ('lift', pre_pick, self.cartesian_move), ('pre_place', pre_place, self.joint_move),
                                   ('place', place, self.cartesian_move), ('retreat', pre_place, self.cartesian_move)]:
            if name == 'lift':
                stages.append(dict(kind='gripper', name='grasp', operation='close', require_grasp=True))
                current['right_gripper_l_finger_joint'] = 0.
                self.validate(current)
            if name == 'retreat':
                stages.append(dict(kind='gripper', name='release', operation='open'))
                current['right_gripper_l_finger_joint'] = self.bounds['right_gripper_l_finger_joint'][1]
                self.validate(current)
            stage, current = method(name, current, goal)
            stages.append(stage)
        return dict(status='planned', planned_at_unix=time.time(), execution_enabled=False, state_source=self.state_source,
            model_source_sha256=json.loads((self.args.model / 'manifest.json').read_text())['source_sha256'],
            collision_scope='robot self-collision; workspace and carried object not modelled',
            start_state=start, approach_in_base=self.args.approach, clearance_m=self.args.clearance,
            pick_file=str(self.args.pick), place_file=str(self.args.place), stages=stages,
            note='Plan only. Grasp width and current state must be verified before execution.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pick', type=Path, default=ROOT / 'config/poses/pick.json')
    parser.add_argument('--place', type=Path, default=ROOT / 'config/poses/place.json')
    parser.add_argument('--model', type=Path, default=ROOT / '.runtime/moveit_model')
    parser.add_argument('--output', type=Path, default=ROOT / '.runtime/pick_place_plan.json')
    parser.add_argument('--clearance', type=float, default=.05)
    parser.add_argument('--approach', type=float, nargs=3, default=[0., 0., 1.], metavar=('X', 'Y', 'Z'))
    args = parser.parse_args()
    norm = math.hypot(*args.approach)
    if not math.isfinite(args.clearance) or not 0 < args.clearance <= .3 or not math.isfinite(norm) or norm < 1e-12:
        parser.error('Use finite clearance in (0, 0.3] metres and a nonzero finite approach vector')
    args.approach = [value / norm for value in args.approach]
    node = None
    rclpy.init(args=[])
    try:
        node = PickPlacePlanner(args)
        report = node.run()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print(json.dumps(dict(status=report['status'], output=str(args.output), stages=[s['name'] for s in report['stages']], execution_enabled=False)))
        return 0
    except (ValueError, RuntimeError, KeyError, OSError) as error:
        print(f'Pick/place plan: {error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
