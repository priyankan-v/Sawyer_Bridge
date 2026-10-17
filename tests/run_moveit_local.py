"""Verify planning services/sequence in a private DDS domain, without hardware execution."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT / '.runtime/moveit-local'
MODEL = ROOT / '.runtime/moveit_model'


def checks():
    import rclpy
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState
    from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive

    assert os.environ['ROS_LOCALHOST_ONLY'] == '1' and os.environ['ROS_DOMAIN_ID'] == '91'
    rclpy.init()
    node = rclpy.create_node('isolated_moveit_service_checks')

    def call(typ, name, request):
        client = node.create_client(typ, '/sawyer_planning/' + name)
        assert client.wait_for_service(timeout_sec=15), name
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15.)
        assert future.done() and future.result() is not None, name
        return future.result()

    def plan(pick, place, output, expected=0):
        result = subprocess.run(['scripts/env.sh', 'system-ros2', 'python3', 'planning/plan_pick_place.py',
            '--pick', str(pick), '--place', str(place), '--output', str(output), '--clearance', '.005'],
            capture_output=True, text=True, timeout=90)
        assert result.returncode == expected, result.stdout + result.stderr
        return result

    try:
        # The measured wrist is out of bounds: keep the hardware recordings intact
        # and use a clearly labelled hypothetical start solely in this private test.
        measured = json.loads((MODEL / 'state.json').read_text())
        test_state = json.loads((WORK / 'test_state.json').read_text())['joints']
        poses = []
        for index in range(2):
            positions = dict(test_state)
            positions['right_j0'] += index * .03
            request = GetPositionFK.Request(fk_link_names=['right_gripper_tip'],
                robot_state=RobotState(joint_state=JointState(name=list(positions), position=list(positions.values()))))
            request.header.frame_id = 'base'
            result = call(GetPositionFK, 'compute_fk', request)
            assert result.error_code.val == 1 and len(result.pose_stamped) == 1
            p = result.pose_stamped[0].pose
            path = WORK / ('test_pick.json' if index == 0 else 'test_place.json')
            path.write_text(json.dumps(dict(frame_id='base', tip_name='right_gripper_tip',
                position=[p.position.x, p.position.y, p.position.z],
                orientation_xyzw=[p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w], joints=positions)))
            poses.append(path)
        output = WORK / 'sequence.json'
        plan(*poses, output)
        report = json.loads(output.read_text())
        assert [stage['name'] for stage in report['stages']] == [
            'open_before_pick', 'pre_pick', 'pick', 'grasp', 'lift', 'pre_place', 'place', 'release', 'retreat']
        assert not report['execution_enabled']
        assert all(stage['trajectory']['joint_trajectory']['points'] for stage in report['stages'] if stage['kind'] == 'arm')
        # A known scene blocker must reject the whole task, rather than return a partial sequence.
        object_pose = Pose()
        object_pose.orientation.w = 1.
        object_pose.position.z = .3
        blocker = CollisionObject(id='test_blocker', operation=CollisionObject.ADD,
            primitives=[SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[3., 3., 3.])], primitive_poses=[object_pose])
        blocker.header.frame_id = 'base'
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [blocker]
        assert call(ApplyPlanningScene, 'apply_planning_scene', ApplyPlanningScene.Request(scene=scene)).success
        result = plan(*poses, WORK / 'blocked_sequence.json', expected=1)
        assert 'collision/constraint' in result.stderr and not (WORK / 'blocked_sequence.json').exists()
        names = [name for name, _ in node.get_service_names_and_types()]
        assert not any('/execute_trajectory/' in name or '/move_action/' in name for name in names)
        assert not node.get_publishers_info_by_topic('/robot/limb/right/joint_command')
        assert not node.get_publishers_info_by_topic('/io/end_effector/right_gripper/command')
        assert json.loads((MODEL / 'state.json').read_text()) == measured
        print('PASS: MoveIt FK/IK, complete taught-pose sequence, blocker rejection, execution interfaces disabled; private hypothetical state only.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def run():
    os.chdir(ROOT)
    os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='91')
    WORK.mkdir(parents=True, exist_ok=True)
    state = json.loads((MODEL / 'state.json').read_text())
    state['joints']['right_j6'] = -4.65
    state['source'] = 'Explicit hypothetical state for private offline planning tests only'
    (WORK / 'test_state.json').write_text(json.dumps(state, indent=2))
    (WORK / 'blocked_sequence.json').unlink(missing_ok=True)
    log = (WORK / 'launch.log').open('w')
    process = subprocess.Popen(['scripts/env.sh', 'system-ros2', 'ros2', 'launch', 'planning/moveit.launch.py',
        'offline_state:=' + str(WORK / 'test_state.json')], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        result = subprocess.run(['scripts/env.sh', 'system-ros2', 'python3', __file__, '--checks'],
                                capture_output=True, text=True, timeout=150)
        print(result.stdout, end='')
        print(result.stderr, end='', file=sys.stderr)
        assert result.returncode == 0
    finally:
        if process.poll() is None:
            # Let launch deliver a single orderly signal to each child.
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        log.close()
        text = (WORK / 'launch.log').read_text()
        assert 'Segmentation fault' not in text and 'process has died' not in text, 'Planner shutdown failed; inspect launch.log'
        assert '[move_group-2]: process has finished cleanly' in text, 'MoveIt clean shutdown was not verified'


if __name__ == '__main__':
    checks() if sys.argv[1:] == ['--checks'] else run()
