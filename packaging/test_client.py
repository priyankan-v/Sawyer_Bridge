"""Verify exact message fields over the localhost package fixture."""
import json
import os
import time

assert os.environ['ROS_IP'] == '127.0.0.1'
assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
import rclpy
from rclpy.qos import qos_profile_sensor_data
from intera_core_msgs.msg import JointCommand, HeadPanCommand
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('sawyer_package_test_client')
received = {}
subscriptions = [node.create_subscription(JointState, '/sawyer_pkg_test/joints', lambda msg: received.update(joints=msg), qos_profile_sensor_data),
                 node.create_subscription(Image, '/sawyer_pkg_test/image', lambda msg: received.update(image=msg), qos_profile_sensor_data),
                 node.create_subscription(String, '/sawyer_pkg_test/ack', lambda msg: received.update(ack=json.loads(msg.data)), 5),
                 node.create_subscription(HeadPanCommand, '/sawyer_pkg_test/head_ack', lambda msg: received.update(head=msg), 5)]
publisher = node.create_publisher(JointCommand, '/sawyer_pkg_test/command', 5)
head = node.create_publisher(HeadPanCommand, '/sawyer_pkg_test/head', 5)
command = JointCommand(mode=1, names=['right_j5'], position=[0.1])
command.header.frame_id = 'package-test'
command.header.stamp.sec = 123
command.header.stamp.nanosec = 456
deadline = time.monotonic() + 20
try:
    while time.monotonic() < deadline and len(received) != 4:
        publisher.publish(command)
        head.publish(HeadPanCommand(target=0.125, speed_ratio=0.25, pan_mode=1))
        rclpy.spin_once(node, timeout_sec=0.05)
    assert len(received) == 4, list(received)
    ack = received['ack']
    assert ack == {'mode': 1, 'names': ['right_j5'], 'position': [0.1], 'velocity': [],
                   'acceleration': [], 'effort': [], 'frame_id': 'package-test', 'stamp': 123000000456}, ack
    state = received['joints']
    assert state.name == ['right_j5'] and list(state.position) == [0.25]
    assert list(state.velocity) == [0.125] and list(state.effort) == [1.5]
    assert bytes(received['image'].data) == bytes([255, 0, 0, 0, 255, 0])
    assert received['head'].target == 0.125 and received['head'].pan_mode == 1
    print('PASS: exact JointCommand fields/empty arrays/header, JointState feedback, HeadPanCommand and Image pixels')
finally:
    node.destroy_node()
    rclpy.shutdown()
