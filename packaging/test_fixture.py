"""Private localhost fixtures for the installed package's transport self-test."""
import json
import os
import sys
import time

assert os.environ['ROS_IP'] == '127.0.0.1'
assert os.environ['ROS_MASTER_URI'].startswith('http://127.0.0.1:')
if sys.argv[1] == 'master':
    from rosmaster.main import rosmaster_main
    rosmaster_main(['rosmaster', '-p', sys.argv[2]])
    raise SystemExit(0)

import rospy
from intera_core_msgs.msg import JointCommand, HeadPanCommand
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String

rospy.init_node('sawyer_package_test_fixture')
ack = rospy.Publisher('/sawyer_pkg_test/ack', String, queue_size=5)

def receive(message):
    fields = ('mode', 'names', 'position', 'velocity', 'acceleration', 'effort')
    data = {field: getattr(message, field) for field in fields}
    data['frame_id'] = message.header.frame_id
    data['stamp'] = message.header.stamp.to_nsec()
    ack.publish(String(data=json.dumps(data)))

rospy.Subscriber('/sawyer_pkg_test/command', JointCommand, receive, queue_size=5)
head_ack = rospy.Publisher('/sawyer_pkg_test/head_ack', HeadPanCommand, queue_size=5)
rospy.Subscriber('/sawyer_pkg_test/head', HeadPanCommand, head_ack.publish, queue_size=5)
joints = rospy.Publisher('/sawyer_pkg_test/joints', JointState, queue_size=5)
image = rospy.Publisher('/sawyer_pkg_test/image', Image, queue_size=1)
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    state = JointState(name=['right_j5'], position=[0.25], velocity=[0.125], effort=[1.5])
    state.header.stamp = rospy.Time.now()
    joints.publish(state)
    frame = Image(height=1, width=2, encoding='rgb8', step=6, data=bytes([255, 0, 0, 0, 255, 0]))
    image.publish(frame)
    rate.sleep()
