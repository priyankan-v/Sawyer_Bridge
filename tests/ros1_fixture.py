"""Localhost-only ROS 1 fixtures; topics and services never touch Sawyer's master."""
import json
import math
import os
import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from intera_core_msgs.msg import JointCommand, RobotAssemblyState, EndpointState, CollisionDetectionState, JointLimits, HeadPanCommand, HeadState
from intera_core_msgs.srv import (SolvePositionFK, SolvePositionFKResponse, SolvePositionIK, SolvePositionIKResponse,
                                  IOComponentCommandSrv, IOComponentCommandSrvResponse)
from geometry_msgs.msg import PoseStamped

assert os.environ['ROS_MASTER_URI'] == 'http://127.0.0.1:11421'
rospy.init_node('isolated_ros1_fixture')
names = [f'right_j{i}' for i in range(7)]
head_position = 0.
arm_positions = dict.fromkeys(names, 0.)
arm_targets = dict(arm_positions)
arm_feedback_offset = 0.
arm_speed_ratio, arm_timeout = .05, .2
arm_commands_since_injection = 0
collision_armed = collision_state = False
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


def speed_ratio(msg):
    global arm_speed_ratio
    assert math.isfinite(msg.data) and 0 < msg.data <= 1.
    arm_speed_ratio = msg.data


def command_timeout(msg):
    global arm_timeout
    assert math.isfinite(msg.data) and 0 < msg.data <= .2
    arm_timeout = msg.data


def inject_collision(msg):
    global collision_armed, collision_state, arm_commands_since_injection
    collision_armed = msg.data
    collision_state = False
    arm_commands_since_injection = 0


def feedback_offset(msg):
    # Sensor-offset injection exists only on the private localhost fixture.
    global arm_feedback_offset
    assert math.isfinite(msg.data) and abs(msg.data) <= .005
    arm_feedback_offset = msg.data


def arm_command(msg):
    # These robot-named topics exist exclusively on the private localhost graph.
    global arm_targets, arm_commands_since_injection, collision_state
    assert msg.mode == JointCommand.POSITION_MODE
    assert set(msg.names).issubset(names) and len(msg.position) == len(msg.names)
    assert len(msg.names) == len(set(msg.names)) and msg.names
    assert not msg.velocity and not msg.acceleration and not msg.effort
    assert all(math.isfinite(q) and -1 <= q <= 1 for q in msg.position)
    assert arm_speed_ratio is not None and arm_timeout is not None
    arm_targets = dict(arm_targets, **dict(zip(msg.names, msg.position)))
    arm_commands_since_injection += 1
    if collision_armed and arm_commands_since_injection >= 20:
        collision_state = True
    payload = dict(received_ns=time.time_ns(), mode=msg.mode, names=list(msg.names),
                   position=list(msg.position), velocity=list(msg.velocity),
                   acceleration=list(msg.acceleration), effort=list(msg.effort),
                   stamp_ns=msg.header.stamp.to_nsec(), speed_ratio=arm_speed_ratio,
                   command_timeout=arm_timeout, collision=collision_state)
    with open(os.path.join(os.environ['SAWYER_ROOT'], '.runtime/arm_commands.jsonl'), 'a') as stream:
        stream.write(json.dumps(payload)+'\n')


rospy.Subscriber('/robot/limb/right/set_speed_ratio', Float64, speed_ratio, queue_size=1)
rospy.Subscriber('/robot/limb/right/joint_command_timeout', Float64, command_timeout, queue_size=1)
rospy.Subscriber('/robot/limb/right/joint_command', JointCommand, arm_command, queue_size=1, tcp_nodelay=True)
rospy.Subscriber('/sawyer_bridge_test/inject_collision', Bool, inject_collision, queue_size=1)
rospy.Subscriber('/sawyer_bridge_test/feedback_offset', Float64, feedback_offset, queue_size=1)


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
previous = time.monotonic()
while not rospy.is_shutdown():
    now = time.monotonic()
    dt = now - previous
    previous = now
    # Rate-limited actuator feedback, rather than copying commands into measurements.
    targets = arm_targets
    max_step = .5 * arm_speed_ratio * dt
    arm_positions = {j: q + max(-max_step, min(max_step, targets[j]-q)) for j,q in arm_positions.items()}
    current_arm = arm_positions
    msg = JointState(name=names+['head_pan'], position=[current_arm[j]+arm_feedback_offset for j in names]+[head_position], velocity=[0.]*8, effort=[0.]*8)
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = str(sequence)
    joint_pub.publish(msg)
    if sequence % 5 == 0:
        state_pub.publish(RobotAssemblyState(homed=True, ready=True, enabled=True))
        endpoint = EndpointState(valid=True)
        endpoint.header.stamp = rospy.Time.now()
        endpoint.pose.position.x = .42
        endpoint_pub.publish(endpoint)
        collision_pub.publish(CollisionDetectionState(collision_state=collision_state))
        head_pub.publish(HeadState(pan=head_position,panMode=HeadState.ACTIVE_MODE))
    sequence += 1
    rate.sleep()
