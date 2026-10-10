#!/usr/bin/env python3
"""Read-only robot graph, subscriber-type and safety checks; never publish."""
import argparse
import json
import math
import os
import socket
import time
import threading
import xmlrpc.client
from pathlib import Path
import rosgraph
import rospy
from intera_core_msgs.msg import CollisionDetectionState, JointLimits, RobotAssemblyState
from sensor_msgs.msg import JointState


def wait_for_positions(required, timeout=5):
    """Merge partial JointState messages; unknown joints cannot satisfy readiness."""
    positions, errors = {}, []
    lock = threading.Lock()
    complete = threading.Event()

    def receive(msg):
        with lock:
            if len(msg.name) != len(set(msg.name)) or len(msg.name) != len(msg.position):
                errors.append('Malformed/duplicate JointState positions')
                complete.set()
                return
            for joint, position in zip(msg.name, msg.position):
                if joint in required:
                    if not math.isfinite(position):
                        errors.append(f'Non-finite position for {joint}')
                    positions[joint] = position
            if errors or set(required).issubset(positions):
                complete.set()

    subscription = rospy.Subscriber('/robot/joint_states', JointState, receive, queue_size=10)
    try:
        if not complete.wait(timeout):
            raise RuntimeError(f'Missing joint feedback: {set(required) - set(positions)}')
        if errors:
            raise RuntimeError('; '.join(errors))
        with lock:
            return dict(positions)
    finally:
        subscription.unregister()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--require-ready', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--joint', choices=[f'right_j{i}' for i in range(7)] + ['head_pan'])
    args = parser.parse_args()
    socket.setdefaulttimeout(4)
    rospy.init_node('sawyer_readonly_preflight', anonymous=True, disable_signals=True)
    master = rosgraph.Master(rospy.get_name())
    types = dict(master.getTopicTypes())
    pubs, subs, services = master.getSystemState()
    expected = {
        '/robot/joint_states': 'sensor_msgs/JointState',
        '/robot/state': 'intera_core_msgs/RobotAssemblyState',
        '/robot/limb/right/endpoint_state': 'intera_core_msgs/EndpointState',
        '/robot/limb/right/collision_detection_state': 'intera_core_msgs/CollisionDetectionState',
        '/robot/joint_limits': 'intera_core_msgs/JointLimits',
    }
    command_topic = '/robot/head/command_head_pan' if args.joint == 'head_pan' else '/robot/limb/right/joint_command'
    command_type = 'intera_core_msgs/HeadPanCommand' if args.joint == 'head_pan' else 'intera_core_msgs/JointCommand'
    if args.joint == 'head_pan':
        expected['/robot/head/head_state'] = 'intera_core_msgs/HeadState'
    result = {'wall_time': time.time(), 'master': os.environ.get('ROS_MASTER_URI'),
              'topic_types': types, 'publishers': pubs, 'subscribers': subs, 'services': services,
              'joint': args.joint, 'command_topic': command_topic, 'command_type': command_type}
    errors = []
    for topic, typ in expected.items():
        if types.get(topic) != typ:
            errors.append(f'{topic}: expected {typ}, discovered {types.get(topic)}')
    command_nodes = dict(subs).get(command_topic, [])
    verified = []
    for node in command_nodes:
        uri = master.lookupNode(node)
        code, _, subscriptions = xmlrpc.client.ServerProxy(uri).getSubscriptions(rospy.get_name())
        if code == 1 and [command_topic, command_type] in subscriptions:
            verified.append({'name': node, 'uri': uri})
    # Exclude bridge/application nodes when establishing a real robot subscriber.
    verified = [node for node in verified if 'bridge' not in node['name'] and 'test' not in node['name']]
    result['verified_command_subscribers'] = verified
    if not verified:
        errors.append(f'No verified robot {command_type} subscriber on {command_topic}')
    if not errors:
        state = rospy.wait_for_message('/robot/state', RobotAssemblyState, timeout=5)
        collision = rospy.wait_for_message('/robot/limb/right/collision_detection_state', CollisionDetectionState, timeout=5)
        required = [f'right_j{i}' for i in range(7)] + (['head_pan'] if args.joint == 'head_pan' else [])
        positions = wait_for_positions(required)
        limits = rospy.wait_for_message('/robot/joint_limits', JointLimits, timeout=5)
        result['state'] = {key: getattr(state, key) for key in state.__slots__}
        result['collision'] = collision.collision_state
        result['positions'] = positions
        result['limits'] = {key: list(getattr(limits, key)) for key in limits.__slots__}
        limit_errors = []
        if len(limits.joint_names) != len(set(limits.joint_names)) or any(len(getattr(limits, f)) != len(limits.joint_names) for f in ('position_lower','position_upper','velocity','accel')):
            raise RuntimeError('Malformed live joint limits')
        if not set(required).issubset(limits.joint_names):
            raise RuntimeError('Required live joint limits are missing')
        for i, joint in enumerate(limits.joint_names):
            if joint in required:
                position = result['positions'][joint]
                if not limits.position_lower[i] <= position <= limits.position_upper[i]:
                    limit_errors.append(f'{joint}: measured {position} outside live limits [{limits.position_lower[i]}, {limits.position_upper[i]}]')
        result['position_limit_errors'] = limit_errors
        result['ready'] = bool(not limit_errors and state.enabled and state.ready and state.homed and not state.stopped
                               and not state.error and not state.lowVoltage and state.estop_source == 0
                               and state.estop_button == 0 and not collision.collision_state)
        if args.require_ready and not result['ready']:
            errors.extend(limit_errors or ['Robot is not homed, enabled, ready and collision-free'])
    result['errors'] = errors
    print(json.dumps({key: result.get(key) for key in ('master', 'joint', 'command_topic', 'verified_command_subscribers', 'state', 'collision', 'positions', 'ready', 'position_limit_errors', 'errors')}, indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n')
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
