"""Read-only FK/IK and teaching workflow on a private ROS 1/ROS 2 graph."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT / '.runtime/cartesian-local'
MASTER = 'http://127.0.0.1:11424'
DOMAIN = '90'
SERVICE_ROOT = '/ExternalTools/right/PositionKinematicsNode/'


def fixture():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from intera_core_msgs.msg import JointLimits
    from intera_core_msgs.srv import SolvePositionFK, SolvePositionFKResponse, SolvePositionIK, SolvePositionIKResponse
    from sensor_msgs.msg import JointState

    assert os.environ['ROS_MASTER_URI'] == MASTER and os.environ['ROS_LOCALHOST_ONLY'] == '1'
    rospy.init_node('isolated_cartesian_fixture')
    names = [f'right_j{i}' for i in range(7)]
    offsets = dict(right_hand=0., right_gripper_tip=.135)

    def fk(request):
        assert len(request.tip_names) == len(request.configuration) == 1
        configuration = request.configuration[0]
        assert set(configuration.name) == set(names)
        joints = dict(zip(configuration.name, configuration.position))
        tip = request.tip_names[0]
        if tip not in offsets:
            return SolvePositionFKResponse(pose_stamp=[PoseStamped()], isValid=[False], inCollision=[False])
        pose = PoseStamped()
        pose.header.frame_id = 'base'
        pose.pose.position.x = .4 + offsets[tip] + .1 * joints['right_j0']
        pose.pose.position.y = .1 + .1 * joints['right_j1']
        pose.pose.position.z = .3 + .1 * joints['right_j2']
        pose.pose.orientation.y = 1.
        return SolvePositionFKResponse(pose_stamp=[pose], isValid=[True], inCollision=[any(abs(q) > .9 for q in joints.values())])

    def ik(request):
        assert request.seed_mode == 1 and len(request.seed_angles) == len(request.tip_names) == len(request.pose_stamp) == 1
        assert list(request.seed_angles[0].name) == names and list(request.seed_angles[0].position) == [0.] * 7
        tip = request.tip_names[0]
        pose = request.pose_stamp[0]
        assert pose.header.frame_id == 'base'
        if tip not in offsets:
            return SolvePositionIKResponse(result_type=[-3])
        if pose.pose.position.x < 0:
            return SolvePositionIKResponse(result_type=[-1])
        position = [(pose.pose.position.x - .4 - offsets[tip]) / .1,
                    (pose.pose.position.y - .1) / .1, (pose.pose.position.z - .3) / .1] + [0.] * 4
        return SolvePositionIKResponse(joints=[JointState(name=names, position=position)], result_type=[1])

    rospy.Service(SERVICE_ROOT + 'FKService', SolvePositionFK, fk)
    rospy.Service(SERVICE_ROOT + 'IKService', SolvePositionIK, ik)
    publisher = rospy.Publisher('/robot/joint_states', JointState, queue_size=1)
    limits = rospy.Publisher('/robot/joint_limits', JointLimits, queue_size=1, latch=True)
    limits.publish(JointLimits(joint_names=names, position_lower=[-1.] * 7, position_upper=[1.] * 7,
                               velocity=[1.] * 7, accel=[1.] * 7, effort=[1.] * 7))
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        publisher.publish(JointState(name=names, position=[0.] * 7, velocity=[0.] * 7))
        rate.sleep()


def run():
    os.chdir(ROOT)
    os.environ.update(ROS_MASTER_URI=MASTER, ROS_IP='127.0.0.1', ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='1')
    os.environ.pop('ROS_HOSTNAME', None)
    WORK.mkdir(parents=True, exist_ok=True)
    socket.setdefaulttimeout(3)
    owned, logs = [], []

    def start(name, args):
        log = (WORK / (name + '.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        owned.append(process)
        return process

    def stop(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)

    def command(*args, expected=0):
        result = subprocess.run(['scripts/env.sh', 'ros2', 'ros2', 'run', 'sawyer_ros2_control', 'cartesian',
                                 *args, '--timeout', '3'], capture_output=True, text=True, timeout=15)
        assert result.returncode == expected, result.stdout + result.stderr
        return json.loads(result.stdout) if expected == 0 else result.stderr

    try:
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 11424))
        start('master', ['scripts/env.sh', 'ros1', 'roscore', '-p', '11424'])
        master = xmlrpc.client.ServerProxy(MASTER)
        deadline = time.monotonic() + 15
        while True:
            try:
                assert master.getUri('/cartesian_test')[0] == 1
                break
            except (OSError, AssertionError):
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.2)
        start('fixture', ['scripts/env.sh', 'ros1', 'python', __file__, '--fixture'])
        launch = ['scripts/env.sh', 'ros2', 'ros2', 'launch', 'sawyer_ros2_control', 'bridge.launch.py']
        bridge = start('default_bridge', launch)
        assert 'kinematics:=true' in command('read', expected=1)
        stop(bridge)
        start('kinematics_bridge', launch + ['kinematics:=true'])
        capture = WORK / 'pick.json'
        capture.unlink(missing_ok=True)
        report = command('capture', '--output', str(capture))
        assert report['tip_name'] == 'right_gripper_tip' and report['position'] == [.535, .1, .3]
        assert command('read', '--tip', 'right_hand')['position'] == [.4, .1, .3]
        assert 'already exists' in command('capture', '--output', str(capture), expected=2)
        assert command('ik', '--pose', str(capture))['status'] == 'ik_valid'
        for changed, error in [(dict(position=[-1., .1, .3]), 'unreachable'),
                               (dict(tip_name='unknown_tip'), 'unknown tool frame'),
                               (dict(position=[.635, .1, .3]), 'self-collision'),
                               (dict(position=[.735, .1, .3]), 'outside'),
                               (dict(frame_id='camera'), 'frame_id must be base'),
                               (dict(orientation_xyzw=[1., 0., 0., 0.]), 'does not match')]:
            goal = WORK / 'invalid_goal.json'
            goal.write_text(json.dumps(dict(report, **changed)))
            assert error in command('ik', '--pose', str(goal), expected=1)
        publishers = dict(master.getSystemState('/cartesian_test')[2][0])
        assert not any(topic in publishers for topic in ('/robot/limb/right/joint_command',
            '/robot/limb/right/set_speed_ratio', '/io/end_effector/right_gripper/command'))
        print('PASS: default-disabled FK/IK, explicit tool capture, pose preservation, seeded IK, invalid goals and no robot command publishers; localhost only.')
    finally:
        for process in reversed(owned):
            stop(process)
        for log in logs:
            log.close()


if __name__ == '__main__':
    fixture() if sys.argv[1:] == ['--fixture'] else run()
