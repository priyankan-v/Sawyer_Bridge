"""Execute the full task against a private simulated ROS 2 arm and gripper."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / '.runtime/moveit_model'
WORK = ROOT / '.runtime/pick-place-local'


def fixture():
    import rclpy
    from intera_core_msgs.msg import (JointCommand, RobotAssemblyState, CollisionDetectionState,
        JointLimits, IOComponentCommand, IOComponentStatus, IODataStatus, IODeviceStatus, IOStatus)
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64

    assert os.environ['ROS_LOCALHOST_ONLY'] == '1' and os.environ['ROS_DOMAIN_ID'] == '92'
    rclpy.init()
    node = rclpy.create_node('isolated_pick_place_controller')
    current = json.loads((WORK / 'test_state.json').read_text())['joints']
    target = dict(current)
    signals = dict(has_error=False, is_calibrated=True, is_moving=False, is_gripping=False,
                   go=True, calibrate=False, position_m=.04, position_response_m=.04, dead_zone_m=.002)
    acks, counts = [], dict(arm=0, gripper=0)
    joint_pub = node.create_publisher(JointState, '/robot/joint_states', 5)
    state_pub = node.create_publisher(RobotAssemblyState, '/robot/state', 1)
    collision_pub = node.create_publisher(CollisionDetectionState, '/robot/limb/right/collision_detection_state', 1)
    limits_pub = node.create_publisher(JointLimits, '/robot/joint_limits', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    gripper_pub = node.create_publisher(IODeviceStatus, '/io/end_effector/right_gripper/state', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    limits_pub.publish(JointLimits(**json.loads((MODEL / 'live_limits.json').read_text())))

    def arm(message):
        assert message.mode == 1 and len(message.names) == len(message.position) == 7
        target.update(zip(message.names, message.position))
        counts['arm'] += 1

    def publish_gripper():
        gripper_pub.publish(IODeviceStatus(time=node.get_clock().now().to_msg(),
            device=IOComponentStatus(name='right_gripper', status=IOStatus(tag='ready')),
            signals=[IODataStatus(name=name, format=json.dumps(dict(type='bool' if type(value) is bool else 'float')),
                                 data=json.dumps([value]), status=IOStatus(tag='ready')) for name,value in signals.items()],
            commands=acks[-20:]))

    def grip(message):
        if message.time in acks:
            return
        assert message.op == 'set'
        values = {name: entry['data'][0] for name,entry in json.loads(message.args)['signals'].items()}
        signals.update(values)
        if 'position_m' in values:
            signals['position_response_m'] = max(.01, signals['position_m'] - .0011160803571429)
            signals['is_gripping'] = signals['position_m'] == 0.
        acks.append(message.time)
        counts['gripper'] += 1
        publish_gripper()

    node.create_subscription(JointCommand, '/robot/limb/right/joint_command', arm, 1)
    node.create_subscription(Float64, '/robot/limb/right/set_speed_ratio', lambda message: None, 1)
    node.create_subscription(Float64, '/robot/limb/right/joint_command_timeout', lambda message: None, 1)
    node.create_subscription(IOComponentCommand, '/io/end_effector/right_gripper/command', grip, 1)
    previous = time.monotonic()

    def tick():
        nonlocal previous
        now = time.monotonic()
        dt, previous = max(1e-5, now - previous), now
        velocity = {}
        for name in current:
            if name.startswith('right_j'):
                step = max(-.1 * dt, min(.1 * dt, target[name] - current[name]))
                current[name] += step
                velocity[name] = step / dt
        current['right_gripper_l_finger_joint'] = signals['position_response_m']  # SDK publishes full aperture.
        message = JointState(name=sorted(current), position=[current[name] for name in sorted(current)],
                             velocity=[velocity.get(name, 0.) for name in sorted(current)])
        message.header.stamp = node.get_clock().now().to_msg()
        joint_pub.publish(message)
        state_pub.publish(RobotAssemblyState(homed=True, ready=True, enabled=True))
        collision_pub.publish(CollisionDetectionState(collision_state=False))

    node.create_timer(.01, tick)
    publish_gripper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        (WORK / 'commands.json').write_text(json.dumps(counts))
        node.destroy_node()
        rclpy.try_shutdown()


def goals():
    import rclpy
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetPositionFK
    from sensor_msgs.msg import JointState
    rclpy.init()
    node = rclpy.create_node('isolated_task_goal_builder')
    client = node.create_client(GetPositionFK, '/sawyer_planning/compute_fk')
    try:
        assert client.wait_for_service(timeout_sec=15.)
        for index, name in enumerate(('pick', 'place')):
            state = json.loads((WORK / 'test_state.json').read_text())['joints']
            state['right_j0'] += index * .005
            request = GetPositionFK.Request(fk_link_names=['right_gripper_tip'], robot_state=RobotState(
                joint_state=JointState(name=list(state), position=list(state.values()))))
            request.header.frame_id = 'base'
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=15.)
            assert future.done() and future.result().error_code.val == 1
            p = future.result().pose_stamped[0].pose
            (WORK / (name + '.json')).write_text(json.dumps(dict(frame_id='base', tip_name='right_gripper_tip',
                position=[p.position.x,p.position.y,p.position.z], orientation_xyzw=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def run():
    os.chdir(ROOT)
    os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='92')
    WORK.mkdir(parents=True, exist_ok=True)
    state = json.loads((MODEL / 'state.json').read_text())
    state['joints']['right_j6'] = -4.65
    state['source'] = 'Hypothetical simulation state; never physical feedback'
    (WORK / 'test_state.json').write_text(json.dumps(state))
    owned, logs = [], []

    def start(name, command):
        log = (WORK / (name + '.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        owned.append(process)
        return process

    def command(arguments, timeout=60):
        result = subprocess.run(arguments, capture_output=True, text=True, timeout=timeout)
        assert result.returncode == 0, result.stdout + result.stderr
        return result

    try:
        start('fixture', ['scripts/env.sh', 'ros2', 'python', __file__, '--fixture'])
        start('moveit', ['scripts/env.sh', 'system-ros2', 'ros2', 'launch', 'planning/moveit.launch.py'])
        command(['scripts/env.sh', 'system-ros2', 'python3', __file__, '--goals'])
        plan = WORK / 'plan.json'
        command(['scripts/env.sh', 'system-ros2', 'python3', 'planning/plan_pick_place.py', '--pick', str(WORK/'pick.json'),
                 '--place', str(WORK/'place.json'), '--clearance', '.005', '--output', str(plan)])
        assert json.loads(plan.read_text())['state_source'] == 'live_feedback'
        preview = command(['scripts/env.sh', 'ros2', 'ros2', 'run', 'sawyer_ros2_control', 'pick_place', '--plan', str(plan)])
        assert json.loads(preview.stdout)['status'] == 'preview'
        result = command(['scripts/env.sh', 'ros2', 'ros2', 'run', 'sawyer_ros2_control', 'pick_place', '--plan', str(plan),
                          '--execute', '--max-velocity', '.05', '--max-acceleration', '.1'], timeout=150)
        (WORK / 'execution.log').write_text(result.stdout + result.stderr)
        assert '"status": "completed"' in result.stdout
        print('PASS: complete guarded arm/gripper task executed against private simulated feedback; no physical robot accessed.')
    finally:
        for process in reversed(owned):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        for log in logs:
            log.close()
        if (WORK/'commands.json').exists():
            counts = json.loads((WORK/'commands.json').read_text())
            print('Simulated command counts:', json.dumps(counts))


if __name__ == '__main__':
    if sys.argv[1:] == ['--fixture']:
        fixture()
    elif sys.argv[1:] == ['--goals']:
        goals()
    else:
        run()
