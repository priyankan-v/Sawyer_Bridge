#!/usr/bin/env python3
"""Export the controller model, limits and measured state for offline planning."""
import argparse
import json
from pathlib import Path
import time
import threading
import sys
import xml.etree.ElementTree as ET

import rospy
from intera_core_msgs.msg import JointLimits
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'planning'))
from state_adapter import model_positions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('.runtime/moveit_model'))
    args = parser.parse_args()
    rospy.init_node('sawyer_moveit_model_export', anonymous=True, disable_signals=True)
    xml = rospy.get_param('/robot_description')
    model = ET.fromstring(xml)
    required = {j.get('name') for j in model.findall('joint')
                if j.get('type') not in ('fixed', 'floating', 'planar') and j.find('mimic') is None}
    state, lock, ready = {}, threading.Lock(), threading.Event()

    def receive(message):
        if len(message.name) != len(message.position):
            return
        with lock:
            state.update({name: value for name, value in zip(message.name, message.position) if name in required})
            if required.issubset(state):
                ready.set()

    subscription = rospy.Subscriber('/robot/joint_states', JointState, receive, queue_size=10)
    try:
        if not ready.wait(5.):
            raise RuntimeError(f'Missing measured model joints: {sorted(required - set(state))}')
        limits = rospy.wait_for_message('/robot/joint_limits', JointLimits, timeout=5.)
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / 'source.urdf').write_text(xml)
        (args.output / 'live_limits.json').write_text(json.dumps(
            {key: list(getattr(limits, key)) for key in limits.__slots__}, indent=2) + '\n')
        with lock:
            snapshot = dict(joints=model_positions(state), sdk_joints=dict(state), exported_at_unix=time.time(),
                            source='measured ROS 1 joint feedback',
                            gripper_conversion='Full opening in SDK left-finger feedback divided by two for URDF travel')
        (args.output / 'state.json').write_text(json.dumps(snapshot, indent=2, allow_nan=False) + '\n')
        print(json.dumps(dict(output=str(args.output), measured_joints=sorted(state), robot_commands_published=False)))
    finally:
        subscription.unregister()


if __name__ == '__main__':
    main()
