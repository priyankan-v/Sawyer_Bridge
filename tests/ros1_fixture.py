"""Localhost-only ROS 1 fixtures; topics and services never touch Sawyer's master."""
import json
import os
import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from intera_core_msgs.msg import JointCommand, RobotAssemblyState, EndpointState, CollisionDetectionState, JointLimits, HeadPanCommand, HeadState
from intera_core_msgs.srv import (SolvePositionFK, SolvePositionFKResponse, SolvePositionIK, SolvePositionIKResponse,
                                  IOComponentCommandSrv, IOComponentCommandSrvResponse)
from geometry_msgs.msg import PoseStamped

assert os.environ['ROS_MASTER_URI'] == 'http://127.0.0.1:11421'
rospy.init_node('isolated_ros1_fixture')
names = [f'right_j{i}' for i in range(7)]
head_position = 0.
head_pub = rospy.Publisher('/robot/head/head_state', HeadState, queue_size=1)
joint_pub = rospy.Publisher('/robot/joint_states', JointState, queue_size=5)
state_pub = rospy.Publisher('/robot/state', RobotAssemblyState, queue_size=1)
endpoint_pub = rospy.Publisher('/robot/limb/right/endpoint_state', EndpointState, queue_size=1)
collision_pub = rospy.Publisher('/robot/limb/right/collision_detection_state', CollisionDetectionState, queue_size=1)
limits_pub = rospy.Publisher('/robot/joint_limits', JointLimits, queue_size=1, latch=True)
ack_pub = rospy.Publisher('/sawyer_bridge_test/ack', String, queue_size=100)


def command(msg):
    received = time.time_ns()
    payload = {key: list(getattr(msg, key)) if key in ('names','position','velocity','acceleration','effort')
               else getattr(msg, key) for key in ('mode','names','position','velocity','acceleration','effort')}
    payload['stamp_ns'] = msg.header.stamp.to_nsec()
    payload['frame_id'] = msg.header.frame_id
    ack_pub.publish(String(data=json.dumps({'received_ns':received, 'payload':payload})))


rospy.Subscriber('/sawyer_bridge_test/joint_command', JointCommand, command, queue_size=100, tcp_nodelay=True)


def head_command(msg):
    # This fixture is bound exclusively to the private localhost test master.
    global head_position
    if msg.pan_mode != HeadPanCommand.SET_ACTIVE_MODE or not -1 <= msg.target <= .9064:
        raise ValueError('Invalid isolated head command')
    head_position = msg.target
    with open(os.path.join(os.environ['SAWYER_ROOT'], '.runtime/head_commands.jsonl'), 'a') as stream:
        stream.write(json.dumps({'target':msg.target,'speed_ratio':msg.speed_ratio,'pan_mode':msg.pan_mode})+'\n')


rospy.Subscriber('/robot/head/command_head_pan', HeadPanCommand, head_command, queue_size=1, tcp_nodelay=True)


def fk(request):
    assert request.tip_names == ['right_hand'] and request.configuration[0].name == names
    pose = PoseStamped()
    pose.header.frame_id = 'base'
    pose.pose.position.x = .42
    return SolvePositionFKResponse(pose_stamp=[pose], isValid=[True], inCollision=[False])


def ik(request):
    assert request.tip_names == ['right_hand'] and request.seed_mode == 1
    joints = JointState(name=names, position=[.01]*7)
    return SolvePositionIKResponse(joints=[joints], result_type=[1])


def io(request):
    assert request.command.op == 'isolated_test'
    response = IOComponentCommandSrvResponse()
    response.time = request.command.time
    response.op = request.command.op
    response.status.tag = 'ready'
    response.response = request.command.args
    return response


rospy.Service('/sawyer_bridge_test/fk', SolvePositionFK, fk)
rospy.Service('/sawyer_bridge_test/ik', SolvePositionIK, ik)
rospy.Service('/sawyer_bridge_test/io', IOComponentCommandSrv, io)
limits_pub.publish(JointLimits(joint_names=names+['head_pan'], position_lower=[-1.]*8, position_upper=[1.]*8,
                              velocity=[.5]*8, accel=[.5]*8, effort=[1.]*8))
rate = rospy.Rate(100)
sequence = 0
while not rospy.is_shutdown():
    msg = JointState(name=names+['head_pan'], position=[0.]*7+[head_position], velocity=[0.]*8, effort=[0.]*8)
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = str(sequence)
    joint_pub.publish(msg)
    if sequence % 5 == 0:
        state_pub.publish(RobotAssemblyState(homed=True, ready=True, enabled=True))
        endpoint = EndpointState(valid=True)
        endpoint.header.stamp = rospy.Time.now()
        endpoint.pose.position.x = .42
        endpoint_pub.publish(endpoint)
        collision_pub.publish(CollisionDetectionState(collision_state=False))
        head_pub.publish(HeadState(pan=head_position,panMode=HeadState.ACTIVE_MODE))
    sequence += 1
    rate.sleep()
