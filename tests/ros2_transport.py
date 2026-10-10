"""Exact field conversion, direction checks, service calls and timestamped local measurements."""
import json
import os
import statistics
import sys
import time
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from intera_core_msgs.msg import JointCommand, RobotAssemblyState, EndpointState, JointLimits
from intera_core_msgs.srv import SolvePositionFK, SolvePositionIK, IOComponentCommandSrv
from geometry_msgs.msg import PoseStamped

assert os.environ['ROS_MASTER_URI'] == 'http://127.0.0.1:11421'
rclpy.init()
node = rclpy.create_node('isolated_ros2_transport_test')
acks, sent, joints, telemetry = {}, {}, [], {}


def joint_callback(msg):
    joints.append((int(msg.header.frame_id), time.time_ns(), msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec))


def ack_callback(msg):
    response = json.loads(msg.data)
    response['returned_ns'] = time.time_ns()
    acks[response['payload']['frame_id']] = response


node.create_subscription(JointState, '/robot/joint_states', joint_callback, qos_profile_sensor_data)
node.create_subscription(String, '/sawyer_bridge_test/ack', ack_callback, 100)
node.create_subscription(RobotAssemblyState, '/robot/state', lambda m: telemetry.update(state=m), 1)
node.create_subscription(EndpointState, '/robot/limb/right/endpoint_state', lambda m: telemetry.update(endpoint=m), qos_profile_sensor_data)
node.create_subscription(JointLimits, '/robot/joint_limits', lambda m: telemetry.update(limits=m),
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
pub = node.create_publisher(JointCommand, '/sawyer_bridge_test/joint_command', 100)


def spin_until(predicate, seconds=15):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.01)
        if predicate():
            return
    raise AssertionError('Timed out waiting for local transport condition')


spin_until(lambda: pub.get_subscription_count() == 1 and len(joints) > 10 and len(telemetry) == 3)
assert telemetry['state'].enabled and telemetry['state'].ready and not telemetry['state'].low_voltage
assert telemetry['endpoint'].valid and telemetry['endpoint'].pose.position.x == .42
assert list(telemetry['limits'].accel) == [.5]*8
assert not node.get_publishers_info_by_topic('/sawyer_bridge_test/joint_command') or \
    all(p.node_name != 'ros_bridge' for p in node.get_publishers_info_by_topic('/sawyer_bridge_test/joint_command'))
assert len(node.get_publishers_info_by_topic('/robot/joint_states')) == 1
assert len(node.get_subscriptions_info_by_topic('/robot/joint_states')) == 1
names = [f'right_j{i}' for i in range(7)]
next_tick = time.monotonic()
for sequence in range(200):
    while time.monotonic() < next_tick:
        rclpy.spin_once(node, timeout_sec=.001)
    msg = JointCommand(mode=1, names=names, position=[i*.01 for i in range(7)],
                       velocity=[i*.02 for i in range(7)], acceleration=[i*.03 for i in range(7)], effort=[i*.04 for i in range(7)])
    stamp = time.time_ns()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
    msg.header.frame_id = f'command:{sequence}'
    payload = {key:list(getattr(msg,key)) if key != 'mode' else msg.mode
               for key in ('mode','names','position','velocity','acceleration','effort')}
    payload.update(stamp_ns=stamp, frame_id=msg.header.frame_id)
    sent[msg.header.frame_id] = payload
    pub.publish(msg)
    next_tick += .01
    rclpy.spin_once(node, timeout_sec=0)
spin_until(lambda: len(acks) == len(sent))
for key, payload in sent.items():
    assert payload == acks[key]['payload'], (key, payload, acks[key])


def service(typ, name, request):
    client = node.create_client(typ, '/sawyer_bridge_test/' + name)
    assert client.wait_for_service(timeout_sec=10), name
    future = client.call_async(request)
    spin_until(future.done)
    result = future.result()
    assert result is not None
    return result


fk = SolvePositionFK.Request(configuration=[JointState(name=names, position=[0.]*7)], tip_names=['right_hand'])
fk_result = service(SolvePositionFK, 'fk', fk)
assert list(fk_result.is_valid) == [True] and list(fk_result.in_collision) == [False]
assert fk_result.pose_stamp[0].pose.position.x == .42
ik = SolvePositionIK.Request(pose_stamp=[PoseStamped()], seed_mode=1, tip_names=['right_hand'])
ik_result = service(SolvePositionIK, 'ik', ik)
assert list(ik_result.joints[0].position) == [.01]*7 and list(ik_result.result_type) == [1]
io = IOComponentCommandSrv.Request(timeout=.1)
io.command.op, io.command.args = 'isolated_test', '{"value":7}'
io.command.time.sec, io.command.time.nanosec = 123, 456789
io_result = service(IOComponentCommandSrv, 'io', io)
assert io_result.time == io.command.time and io_result.op == io.command.op and io_result.response == io.command.args


def summarize(values):
    values = sorted(values)
    return {'samples':len(values), 'mean':statistics.mean(values), 'p50':statistics.median(values),
            'p95':values[int(.95*(len(values)-1))], 'max':max(values)}


interarrival = [(b[1]-a[1])/1e6 for a,b in zip(joints,joints[1:])]
ids = [item[0] for item in joints]
result = {'status':'PASS', 'clock_basis':'Same workstation CLOCK_REALTIME for source and receiver timestamps; includes serialization and scheduling',
          'command_fields_identical':True, 'commands_sent':len(sent), 'commands_received':len(acks),
          'command_loss':len(sent)-len(acks), 'services_verified':['SolvePositionFK','SolvePositionIK','IOComponentCommandSrv'],
          'direction_checks':'No ROS 2 bridge publisher on commands and no ROS 2 bridge subscriber on joint states',
          'ros1_joint_source_hz':(len(joints)-1)/((joints[-1][2]-joints[0][2])/1e9),
          'ros2_joint_receive_hz':(len(joints)-1)/((joints[-1][1]-joints[0][1])/1e9),
          'joint_messages_received':len(joints), 'joint_sequence_gaps':sum(max(0,b-a-1) for a,b in zip(ids,ids[1:])),
          'joint_interarrival_ms':summarize(interarrival), 'joint_interarrival_jitter_stddev_ms':statistics.pstdev(interarrival),
          'joint_local_timestamp_latency_ms':summarize([(r-s)/1e6 for _,r,s in joints]),
          'command_forwarding_latency_ms':summarize([(acks[k]['received_ns']-sent[k]['stamp_ns'])/1e6 for k in sent]),
          'command_ack_roundtrip_ms':summarize([(acks[k]['returned_ns']-sent[k]['stamp_ns'])/1e6 for k in sent])}
Path(sys.argv[1]).write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2))
node.destroy_node()
rclpy.shutdown()
