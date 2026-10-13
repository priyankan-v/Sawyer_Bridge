"""Native actionlib fixtures and reverse-direction clients, strictly localhost."""
import os
import sys
import threading
import time
assert os.environ['ROS_IP'] == '127.0.0.1'
assert os.environ['ROS_MASTER_URI'].startswith('http://127.0.0.1:')
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from intera_core_msgs.msg import CalibrationCommandAction, CalibrationCommandGoal, CalibrationCommandResult, CalibrationCommandFeedback
from intera_motion_msgs.msg import MotionCommandAction, MotionCommandGoal, MotionCommandResult, MotionCommandFeedback
from intera_core_msgs.srv import IOComponentCommandSrv, IOComponentCommandSrvResponse, SolvePositionFK, SolvePositionFKResponse, SolvePositionIK, SolvePositionIKResponse

rospy.init_node('sawyer_action_fixture_' + sys.argv[1])
TYPES = [('calibration', CalibrationCommandAction, CalibrationCommandGoal, CalibrationCommandResult, CalibrationCommandFeedback),
         ('motion', MotionCommandAction, MotionCommandGoal, MotionCommandResult, MotionCommandFeedback)]

def result_value(typ, command):
    result = typ(result=command == 'success')
    if hasattr(result, 'statusId'): result.statusId = command
    else:
        result.errorId = command
        result.last_successful_waypoint = 7
        result.trajectory_analysis.planned_duration = 1.25
    return result

if sys.argv[1] == 'server':
    servers = []
    for name, spec, goal_type, result_type, feedback_type in TYPES:
        def accept(handle, name=name, result_type=result_type, feedback_type=feedback_type):
            command = handle.get_goal().command
            if command == 'reject':
                handle.set_rejected(result_value(result_type, command), 'fixture rejection'); return
            if name == 'motion': assert handle.get_goal().trajectory.label == 'nested-trajectory'
            handle.set_accepted()
            def execute():
                feedback = feedback_type()
                if name == 'calibration':
                    feedback.currentState = 'fixture'; feedback.numberOfPoses = 3; feedback.currentPoseNumber = 2
                else:
                    feedback.status.motion_status = 'fixture'; feedback.status.current_waypoint = 2
                start = time.monotonic()
                while handle.get_goal_status().status == GoalStatus.ACTIVE:
                    handle.publish_feedback(feedback)
                    if command != 'wait' and time.monotonic() - start > .3:
                        result = result_value(result_type, command)
                        if command == 'abort': handle.set_aborted(result)
                        else: handle.set_succeeded(result)
                        return
                    time.sleep(.03)
            threading.Thread(target=execute, daemon=True).start()
        def cancel(handle, result_type=result_type): handle.set_canceled(result_value(result_type, 'canceled'))
        server = actionlib.ActionServer('/sawyer_pkg_test/forward_' + name, spec, accept, cancel, auto_start=False)
        server.start(); servers.append(server)
    services = [rospy.Service('/sawyer_pkg_test/forward_io', IOComponentCommandSrv,
                 lambda request: IOComponentCommandSrvResponse(response=request.command.op, op='fixture')),
                rospy.Service('/sawyer_pkg_test/forward_fk', SolvePositionFK,
                 lambda request: SolvePositionFKResponse(isValid=[True], inCollision=[False])),
                rospy.Service('/sawyer_pkg_test/forward_ik', SolvePositionIK,
                 lambda request: SolvePositionIKResponse(result_type=[1]))]
    print('Native ROS 1 actionlib servers ready', flush=True)
    rospy.spin()
elif sys.argv[1] == 'loss':
    from pathlib import Path
    client = actionlib.SimpleActionClient('/sawyer_pkg_test/reverse_calibration', CalibrationCommandAction)
    assert client.wait_for_server(rospy.Duration(10))
    feedback = []
    client.send_goal(CalibrationCommandGoal(command='wait'), feedback_cb=lambda msg: feedback.append(msg))
    deadline = time.monotonic() + 10
    while not feedback and time.monotonic() < deadline: time.sleep(.02)
    assert feedback
    Path(sys.argv[2]).write_text('ready')
    assert client.wait_for_result(rospy.Duration(45))
    assert client.get_state() == GoalStatus.ABORTED, client.get_state()
    assert client.get_result().statusId == 'ROS2_ACTION_SERVER_UNAVAILABLE'
    print('PASS: ROS 2 action server loss aborts native ROS 1 client goal', flush=True)
else:
    for name, spec, goal_type, result_type, feedback_type in TYPES:
        client = actionlib.SimpleActionClient('/sawyer_pkg_test/reverse_' + name, spec)
        assert client.wait_for_server(rospy.Duration(10)), name
        for command, expected in [('success', GoalStatus.SUCCEEDED), ('abort', GoalStatus.ABORTED), ('reject', GoalStatus.REJECTED), ('wait', GoalStatus.PREEMPTED)]:
            feedback = []
            goal = goal_type(command=command)
            if name == 'motion': goal.trajectory.label = 'nested-trajectory'
            client.send_goal(goal, feedback_cb=lambda msg: feedback.append(msg))
            if command == 'wait':
                deadline = time.monotonic() + 5
                while not feedback and time.monotonic() < deadline: time.sleep(.01)
                assert feedback, (name, 'cancel goal never got feedback')
                client.cancel_goal()
            assert client.wait_for_result(rospy.Duration(5)), (name, command, 'timeout')
            assert client.get_state() == expected, (name, command, client.get_state())
            result = client.get_result()
            actual = result.statusId if name == 'calibration' else result.errorId
            assert actual == ('canceled' if command == 'wait' else 'ROS2_GOAL_REJECTED' if command == 'reject' else command), (name, command, actual)
            if command not in ('reject', 'wait'):
                assert feedback, (name, command)
                if name == 'calibration': assert feedback[-1].currentState == 'fixture' and feedback[-1].currentPoseNumber == 2
                else: assert feedback[-1].status.current_waypoint == 2 and result.last_successful_waypoint == 7
        # Native ROS 1 cancel-all must cancel every correlated ROS 2 goal.
        second = actionlib.SimpleActionClient('/sawyer_pkg_test/reverse_' + name, spec)
        assert second.wait_for_server(rospy.Duration(5))
        feedback1, feedback2 = [], []
        goal = goal_type(command='wait')
        if name == 'motion': goal.trajectory.label = 'nested-trajectory'
        client.send_goal(goal, feedback_cb=lambda msg: feedback1.append(msg))
        second.send_goal(goal, feedback_cb=lambda msg: feedback2.append(msg))
        deadline = time.monotonic() + 5
        while (not feedback1 or not feedback2) and time.monotonic() < deadline: time.sleep(.02)
        assert feedback1 and feedback2
        client.cancel_all_goals()
        for handle in (client, second):
            assert handle.wait_for_result(rospy.Duration(5))
            assert handle.get_state() == GoalStatus.PREEMPTED
    for name, typ in [('io', IOComponentCommandSrv), ('fk', SolvePositionFK), ('ik', SolvePositionIK)]:
        service = '/sawyer_pkg_test/reverse_' + name
        rospy.wait_for_service(service, timeout=10)
        request = typ._request_class()
        if name == 'io': request.command.op = 'echo'
        result = rospy.ServiceProxy(service, typ)(request)
        if name == 'io': assert result.response == 'echo'
        elif name == 'fk': assert result.isValid == [True] and result.inCollision == [False]
        else: assert list(result.result_type) == [1]
    print('PASS: native ROS 1 actionlib clients; both Intera actions and all three services, reverse direction', flush=True)
