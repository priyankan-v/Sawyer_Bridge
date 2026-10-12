"""ROS 2 monitoring and kinematics queries only. Never create a command publisher."""
import json
import math
import statistics
import sys
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from intera_core_msgs.msg import RobotAssemblyState, EndpointState, CollisionDetectionState, JointLimits, HeadState, IONodeConfiguration, IODeviceStatus
from intera_core_msgs.srv import SolvePositionFK, SolvePositionIK

rclpy.init()
node=rclpy.create_node('sawyer_readonly_hardware_verification')
rows, latest = [], {}


def joints(msg):
    rows.append({'received_ns':time.monotonic_ns(), 'stamp_ns':msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,'names':list(msg.name)})
    if all(f'right_j{i}' in msg.name for i in range(7)):
        latest['joints']=msg


node.create_subscription(JointState,'/robot/joint_states',joints,qos_profile_sensor_data)
for key,topic,typ,qos in (
    ('state','/robot/state',RobotAssemblyState,1),
    ('endpoint','/robot/limb/right/endpoint_state',EndpointState,qos_profile_sensor_data),
    ('collision','/robot/limb/right/collision_detection_state',CollisionDetectionState,1),
    ('limits','/robot/joint_limits',JointLimits,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ('head','/robot/head/head_state',HeadState,1),
    ('gripper_config','/io/end_effector/config',IONodeConfiguration,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ('gripper_state','/io/end_effector/right_gripper/state',IODeviceStatus,1)):
    node.create_subscription(typ,topic,lambda m,k=key:latest.update({k:m}),qos)
deadline=time.monotonic()+10
while time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=.01)
required=('joints','state','endpoint','collision','limits','head','gripper_config','gripper_state')
assert all(k in latest for k in required), ('Missing hardware telemetry',list(latest))
joint=latest['joints']
names=[f'right_j{i}' for i in range(7)]
assert len(joint.name) == len(set(joint.name))
assert len(joint.position) == len(joint.name) == len(joint.velocity)
assert all(math.isfinite(v) for v in list(joint.position) + list(joint.velocity))
configuration=JointState(name=names,position=[joint.position[joint.name.index(j)] for j in names])


def call(typ,suffix,request):
    client=node.create_client(typ,'/ExternalTools/right/PositionKinematicsNode/'+suffix)
    assert client.wait_for_service(timeout_sec=5),suffix
    future=client.call_async(request)
    rclpy.spin_until_future_complete(node,future,timeout_sec=10)
    assert future.done(),suffix
    return future.result()


fk=call(SolvePositionFK,'FKService',SolvePositionFK.Request(configuration=[configuration],tip_names=['right_hand']))
assert fk is not None
ik_result=None
if len(fk.pose_stamp) and fk.is_valid[0]:
    ik=call(SolvePositionIK,'IKService',SolvePositionIK.Request(pose_stamp=[fk.pose_stamp[0]],seed_angles=[configuration],seed_mode=1,tip_names=['right_hand']))
    assert ik is not None
    ik_result={'result_type':list(ik.result_type),'solution_count':len(ik.joints)}
arm=[r for r in rows if all(j in r['names'] for j in names)]
intervals=[(b['received_ns']-a['received_ns'])/1e6 for a,b in zip(arm,arm[1:])]
result={'status':'PASS','scope':'read-only physical Sawyer monitoring and kinematics; no robot command publications',
        'telemetry_verified':list(latest),'positions':dict(zip(configuration.name,configuration.position)),
        'velocities':{j:joint.velocity[joint.name.index(j)] for j in names},
        'robot_state':{f:getattr(latest['state'],f) for f in latest['state'].get_fields_and_field_types()},
        'collision':latest['collision'].collision_state,'endpoint_valid':latest['endpoint'].valid,
        'head_pan':latest['head'].pan,'gripper_devices':[d.name for d in latest['gripper_config'].devices],
        'fk':{'is_valid':list(fk.is_valid),'in_collision':list(fk.in_collision),'poses':len(fk.pose_stamp)},
        'ik':ik_result,'joint_messages':len(rows),'arm_messages':len(arm),
        'ros2_arm_receive_hz':(len(arm)-1)/((arm[-1]['received_ns']-arm[0]['received_ns'])/1e9),
        'arm_interarrival_ms':{'mean':statistics.mean(intervals),'stddev':statistics.pstdev(intervals),'max':max(intervals)},
        'transport_latency':'Not reported: workstation/controller clock synchronization has not been established.'}
Path(sys.argv[1]).write_text(json.dumps(result,indent=2)+'\n')
Path(sys.argv[2]).write_text(json.dumps(rows)+'\n')
print(json.dumps(result,indent=2))
node.destroy_node()
rclpy.shutdown()
