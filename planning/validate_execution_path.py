#!/usr/bin/env python3
"""Validate the exact retimed joint polyline against current hand/head geometry."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity

ARM = tuple(f'right_j{i}' for i in range(7))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path', type=Path)
    parser.add_argument('--model', type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.path.read_text())
    limits = {}
    for joint in ET.parse(args.model / 'sawyer.urdf').getroot().findall('joint'):
        if joint.get('type') in ('revolute', 'prismatic') and joint.find('mimic') is None:
            limit = joint.find('limit')
            limits[joint.get('name')] = (float(limit.get('lower')), float(limit.get('upper')))
    rclpy.init(args=[])
    node = Node('sawyer_execution_path_check')
    latest = {}

    def receive(message):
        if len(message.name) == len(message.position) and len(set(message.name)) == len(message.name):
            latest.update(positions=dict(zip(message.name, message.position)), received=time.monotonic(),
                          live=message.header.frame_id == 'base')

    node.create_subscription(JointState, '/sawyer_planning/joint_states', receive, 1)
    client = node.create_client(GetStateValidity, '/sawyer_planning/check_state_validity')
    try:
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if latest and latest['live']:
                expected = data.get('expected_gripper_joint')
                if expected is None or abs(latest['positions'].get('right_gripper_l_finger_joint', math.inf) - expected) <= .001:
                    break
        else:
            raise RuntimeError('Fresh live planning state matching the gripper is required; offline replay cannot execute')
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('MoveIt state validity service unavailable')
        baseline = dict(latest['positions'])
        for name, (lower, upper) in limits.items():
            value = baseline.get(name)
            if value is None or not math.isfinite(value) or not lower <= value <= upper:
                raise RuntimeError(f'{name}: {value} outside [{lower}, {upper}]')
        points = data['waypoints']
        if not points or max(abs(points[0][name] - baseline[name]) for name in ARM) > .008726646:
            raise RuntimeError('Arm does not match the start of the planned stage; replan')
        # Connect the measured settled position to the planned first waypoint.
        # This connection is collision checked too; an arrival tolerance never
        # silently changes the geometric path that will be executed.
        actual = {name: baseline[name] for name in ARM}
        if max(abs(points[0][name] - actual[name]) for name in ARM) > 1e-10:
            points = [actual] + points
        checked = 0
        for first, second in zip(points, points[1:] or points):
            count = max(1, math.ceil(max(abs(second[name] - first[name]) for name in ARM) / .005))
            for index in range(count + 1):
                if time.monotonic() - latest['received'] > 1.:
                    raise RuntimeError('Live planning feedback disconnected during validation')
                positions = dict(baseline)
                positions.update({name: first[name] + (second[name] - first[name]) * index / count for name in ARM})
                for name in ARM:
                    value = positions[name]
                    if not math.isfinite(value) or not limits[name][0] <= value <= limits[name][1]:
                        raise RuntimeError(f'Invalid planned position for {name}')
                state = RobotState(joint_state=JointState(name=list(positions), position=list(positions.values())), is_diff=False)
                future = client.call_async(GetStateValidity.Request(robot_state=state))
                rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
                if not future.done() or future.result() is None or not future.result().valid:
                    raise RuntimeError('Execution polyline failed collision/constraint validation')
                if abs(latest['positions']['head_pan'] - baseline['head_pan']) > .01:
                    raise RuntimeError('Head moved during collision validation; replan')
                checked += 1
        if max(abs(latest['positions'][name] - baseline[name]) for name in ARM) > .002:
            raise RuntimeError('Arm moved during path validation; replan')
        print(json.dumps(dict(status='valid', samples=checked, head_pan=baseline['head_pan'], waypoints=points)))
        return 0
    except (ValueError, RuntimeError, KeyError) as error:
        print(f'Execution path: {error}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
