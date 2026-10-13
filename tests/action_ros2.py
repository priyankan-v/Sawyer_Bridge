"""Native rclpy action fixtures and clients; never use physical robot endpoints."""
import os
import sys
import time
assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from intera_core_msgs.action import CalibrationCommand
from intera_motion_msgs.action import MotionCommand
from intera_core_msgs.srv import IOComponentCommandSrv, SolvePositionFK, SolvePositionIK
rclpy.init()
node = rclpy.create_node('sawyer_action_fixture_' + sys.argv[1])
TYPES = [('calibration', CalibrationCommand), ('motion', MotionCommand)]

def result_value(typ, command):
    result = typ.Result(result=command == 'success')
    if typ == CalibrationCommand: result.status_id = command
    else:
        result.error_id = command; result.last_successful_waypoint = 7
        result.trajectory_analysis.planned_duration = 1.25
    return result

if sys.argv[1] == 'server':
    group = ReentrantCallbackGroup()
    servers = []
    for name, typ in TYPES:
        def execute(handle, name=name, typ=typ):
            assert name != 'motion' or handle.request.trajectory.label == 'nested-trajectory'
            feedback = typ.Feedback()
            if name == 'calibration':
                feedback.current_state = 'fixture'; feedback.number_of_poses = 3; feedback.current_pose_number = 2
            else: feedback.status.motion_status = 'fixture'; feedback.status.current_waypoint = 2
            start = time.monotonic()
            while True:
                handle.publish_feedback(feedback)
                if handle.is_cancel_requested:
                    handle.canceled(); return result_value(typ, 'canceled')
                if handle.request.command != 'wait' and time.monotonic() - start > .3:
                    if handle.request.command == 'abort': handle.abort()
                    else: handle.succeed()
                    return result_value(typ, handle.request.command)
                time.sleep(.02)
        server = ActionServer(node, typ, '/sawyer_pkg_test/reverse_' + name, execute,
            goal_callback=lambda goal: GoalResponse.REJECT if goal.command == 'reject' else GoalResponse.ACCEPT,
            cancel_callback=lambda handle: CancelResponse.ACCEPT, callback_group=group)
        servers.append(server)
    def io(request, response): response.response = request.command.op; response.op = 'fixture'; return response
    def fk(request, response): response.is_valid = [True]; response.in_collision = [False]; return response
    def ik(request, response): response.result_type = [1]; return response
    services = [node.create_service(typ, '/sawyer_pkg_test/reverse_' + name, callback, callback_group=group)
                for name, typ, callback in [('io', IOComponentCommandSrv, io), ('fk', SolvePositionFK, fk), ('ik', SolvePositionIK, ik)]]
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    print('Native ROS 2 action servers ready', flush=True)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
elif sys.argv[1] == 'loss':
    from pathlib import Path
    client = ActionClient(node, CalibrationCommand, '/sawyer_pkg_test/forward_calibration')
    assert client.wait_for_server(timeout_sec=10)
    feedback = []
    future = client.send_goal_async(CalibrationCommand.Goal(command='wait'), feedback_callback=lambda msg: feedback.append(msg.feedback))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    handle = future.result(); assert handle.accepted
    deadline = time.monotonic() + 10
    while not feedback and time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=.02)
    assert feedback
    Path(sys.argv[2]).write_text('ready')
    future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, future, timeout_sec=12)
    assert future.done()
    assert future.result().status == GoalStatus.STATUS_ABORTED
    assert future.result().result.status_id == 'ROS1_ACTION_SERVER_UNAVAILABLE'
    print('PASS: ROS 1 action server loss aborts native ROS 2 client goal', flush=True)
    node.destroy_node(); rclpy.shutdown()
else:
    def wait(future):
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        assert future.done(), 'ROS 2 future timed out'
        return future.result()
    for name, typ in TYPES:
        client = ActionClient(node, typ, '/sawyer_pkg_test/forward_' + name)
        assert client.wait_for_server(timeout_sec=10), name
        for command, expected in [('success', GoalStatus.STATUS_SUCCEEDED), ('abort', GoalStatus.STATUS_ABORTED), ('reject', GoalStatus.STATUS_ABORTED), ('wait', GoalStatus.STATUS_CANCELED)]:
            goal = typ.Goal(command=command)
            if name == 'motion': goal.trajectory.label = 'nested-trajectory'
            feedback = []
            handle = wait(client.send_goal_async(goal, feedback_callback=lambda msg: feedback.append(msg.feedback)))
            deadline = time.monotonic() + 5
            while not handle.accepted and time.monotonic() < deadline:
                time.sleep(.1)
                handle = wait(client.send_goal_async(goal, feedback_callback=lambda msg: feedback.append(msg.feedback)))
            assert handle.accepted, (name, command)
            if command == 'wait':
                deadline = time.monotonic() + 5
                while not feedback and time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=.02)
                assert feedback, (name, command)
                canceled = wait(handle.cancel_goal_async())
                assert canceled.goals_canceling, canceled
            wrapped = wait(handle.get_result_async())
            assert wrapped.status == expected, (name, command, wrapped.status)
            result = wrapped.result
            actual = result.status_id if name == 'calibration' else result.error_id
            assert actual == ('canceled' if command == 'wait' else command), (name, command, actual)
            if command == 'success':
                assert feedback, (name, command)
                if name == 'calibration': assert feedback[-1].current_pose_number == 2
                else: assert result.last_successful_waypoint == 7 and result.trajectory_analysis.planned_duration == 1.25
        client.destroy()
    for name, typ in [('io', IOComponentCommandSrv), ('fk', SolvePositionFK), ('ik', SolvePositionIK)]:
        client = node.create_client(typ, '/sawyer_pkg_test/forward_' + name)
        assert client.wait_for_service(timeout_sec=10), name
        request = typ.Request()
        if name == 'io': request.command.op = 'echo'
        result = wait(client.call_async(request))
        if name == 'io': assert result.response == 'echo'
        elif name == 'fk': assert list(result.is_valid) == [True] and list(result.in_collision) == [False]
        else: assert list(result.result_type) == [1]
    print('PASS: native ROS 2 action clients; both Intera actions and all three services, forward direction', flush=True)
    node.destroy_node(); rclpy.shutdown()
