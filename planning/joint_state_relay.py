#!/usr/bin/env python3
"""Merge Intera feedback for MoveIt; convert gripper travel and timestamp receipt."""
import argparse
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from state_adapter import model_positions


class JointStateRelay(Node):
    def __init__(self, directory, offline=None):
        super().__init__('sawyer_planning_state_relay')
        model = ET.parse(directory / 'sawyer.urdf').getroot()
        self.required = {joint.get('name') for joint in model.findall('joint')
            if joint.get('type') not in ('fixed', 'floating', 'planar') and joint.find('mimic') is None}
        self.positions, self.received = {}, {}
        self.offline = offline is not None
        self.publisher = self.create_publisher(JointState, '/sawyer_planning/joint_states', 1)
        if self.offline:
            data = json.loads(offline.read_text())
            self.positions = {name: float(data['joints'][name]) for name in self.required}
            self.get_logger().warn('OFFLINE snapshot replay; this state must not be used for hardware execution')
        else:
            self.create_subscription(JointState, '/robot/joint_states', self.receive, qos_profile_sensor_data)
        self.create_timer(.05, self.publish)

    def receive(self, message):
        if not set(message.name).intersection(self.required):
            return
        if len(message.name) != len(message.position) or len(message.name) != len(set(message.name)):
            self.get_logger().error('Malformed joint feedback; ceasing derived state until fresh valid feedback')
            self.received.clear()
            return
        try:
            values = model_positions(dict(zip(message.name, message.position)))
        except ValueError as error:
            self.received.clear()
            self.get_logger().error(str(error))
            return
        now = time.monotonic()
        for name, value in values.items():
            if name in self.required:
                self.positions[name], self.received[name] = value, now

    def publish(self):
        if not self.required.issubset(self.positions):
            return
        # Gripper state may be published only on change. Arm and head feedback
        # are continuous and must remain fresh. Never clamp measured positions.
        if not self.offline and any(time.monotonic() - self.received.get(name, -1e30) > 1.
            for name in self.required if name != 'right_gripper_l_finger_joint'):
            return
        message = JointState(name=sorted(self.required), position=[self.positions[name] for name in sorted(self.required)])
        # Controller timestamps use a different clock epoch; MoveIt freshness is
        # based on local receipt time, matching the existing arm control guards.
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'offline_snapshot' if self.offline else 'base'
        self.publisher.publish(message)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--offline-state', type=Path)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = JointStateRelay(args.model, args.offline_state)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
