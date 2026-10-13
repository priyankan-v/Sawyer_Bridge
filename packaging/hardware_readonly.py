"""Bounded telemetry check of the installed C++ bridge; never create command paths."""
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client

from sawyer_ros2_control.runtime import package_prefix

socket.setdefaulttimeout(3)
uri = os.environ['ROS_MASTER_URI']
master = xmlrpc.client.ServerProxy(uri)
caller = '/sawyer_package_readonly_' + str(os.getpid())
report = {'scope': 'Installed package telemetry-only bridge; no motion or camera-control commands',
          'master': uri, 'package_prefix': str(package_prefix()), 'temporary_namespace': caller}
try:
    assert master.getUri(caller)[0] == 1
except (OSError, AssertionError, xmlrpc.client.Error) as error:
    report.update(status='UNAVAILABLE', error=str(error))
    print(json.dumps(report, indent=2))
    raise SystemExit(2)

topics = [dict(topic='/robot/joint_states', type='sensor_msgs/msg/JointState', direction='1to2',
               enabled=True, queue_size=5, qos={'reliability': 'best_effort', 'history': 'keep_last', 'depth': 5}),
          dict(topic='/robot/joint_limits', type='intera_core_msgs/msg/JointLimits', direction='1to2',
               enabled=True, queue_size=1, qos={'durability': 'transient_local', 'history': 'keep_last', 'depth': 1}),
          dict(topic='/robot/state', type='intera_core_msgs/msg/RobotAssemblyState', direction='1to2',
               enabled=True, queue_size=1)]
assert all(topic['direction'] == '1to2' for topic in topics)
parameter = caller + '/topics'
assert master.setParam(caller, parameter, topics)[0] == 1
environment = dict(os.environ, ROS_DOMAIN_ID='198')
node = None
process = None
with tempfile.TemporaryFile(mode='w+') as log:
    try:
        process = subprocess.Popen([str(package_prefix() / 'lib/ros1_bridge/parameter_bridge'),
                                    parameter, caller + '/no_services', caller + '/no_services',
                                    '__name:=' + caller.lstrip('/')], env=environment,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        os.environ['ROS_DOMAIN_ID'] = '198'
        import rclpy
        from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from intera_core_msgs.msg import JointLimits, RobotAssemblyState
        rclpy.init()
        node = rclpy.create_node('sawyer_package_readonly_receiver')
        counts = []
        positions, velocities, feedback = {}, {}, {}
        def on_state(msg):
            counts.append(time.monotonic())
            positions.update(zip(msg.name, msg.position))
            velocities.update(zip(msg.name, msg.velocity))
        subscriptions = [node.create_subscription(JointState, '/robot/joint_states', on_state, qos_profile_sensor_data),
                         node.create_subscription(JointLimits, '/robot/joint_limits', lambda msg: feedback.update(limits=list(msg.joint_names)),
                                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
                         node.create_subscription(RobotAssemblyState, '/robot/state',
                                                  lambda msg: feedback.update(robot_enabled=msg.enabled, robot_ready=msg.ready), 1)]
        end = time.monotonic() + 6
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        assert process.poll() is None, 'Bridge exited'
        assert len(counts) > 20 and all(f'right_j{i}' in positions for i in range(7)), positions
        assert all(math.isfinite(value) for value in positions.values())
        assert all(f'right_j{i}' in velocities for i in range(7)), velocities
        assert 'limits' in feedback and 'robot_enabled' in feedback, feedback
        report.update(status='PASS', samples=len(counts), positions=positions, velocities=velocities,
                      receive_hz=(len(counts)-1)/(counts[-1]-counts[0]), **feedback)
        print(json.dumps(report, indent=2))
    except Exception:
        log.seek(0)
        print(log.read())
        raise
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        master.deleteParam(caller, caller)
