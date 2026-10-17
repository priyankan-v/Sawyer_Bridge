"""Pose capture, frame conventions and read-only IK response validation."""
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from builtin_interfaces.msg import Time
from intera_core_msgs.msg import JointLimits
from intera_core_msgs.srv import SolvePositionFK, SolvePositionIK
from sensor_msgs.msg import JointState
from sawyer_ros2_control.cartesian import (Cartesian, DEFAULT_TIP, checked_joints,
    checked_pose, parse_args, pose_data, pose_error, pose_message, write_json)
from sawyer_ros2_control.safety import JOINTS


def pose():
    return dict(frame_id='base', tip_name=DEFAULT_TIP, position=[0.4, 0.1, 0.2],
                orientation_xyzw=[0., 1., 0., 0.])


def client():
    node = Cartesian.__new__(Cartesian)
    node.args = parse_args([])
    node.positions, node.velocities, node.received = {}, {}, {}
    node.fault = None
    node.wait_until = Mock()
    node.service = Mock()
    node.fk_client, node.ik_client = Mock(), Mock()
    node.get_clock = Mock()
    node.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=1)
    node.limits_message = JointLimits(joint_names=list(JOINTS), position_lower=[-1.] * 7,
        position_upper=[1.] * 7, velocity=[1.] * 7, accel=[1.] * 7)
    node.receive_joints(JointState(name=list(JOINTS), position=[0.] * 7, velocity=[0.] * 7))
    return node


class PoseTests(unittest.TestCase):
    def test_pose_round_trip_preserves_base_and_tool_conventions(self):
        self.assertEqual(pose_data(pose_message(pose()), DEFAULT_TIP), pose())
        scaled = dict(pose(), orientation_xyzw=[0., 2., 0., 0.])
        self.assertEqual(checked_pose(scaled), pose())

    def test_unsupported_frame_or_different_tool_is_rejected(self):
        for data in (dict(pose(), frame_id='camera'), dict(pose(), tip_name=''),
                     dict(pose(), position=[0., 1.]), dict(pose(), position=[0., float('nan'), 1.]),
                     dict(pose(), orientation_xyzw=[0.] * 4), dict(pose(), position=[True, 0., 0.])):
            with self.subTest(data=data), self.assertRaises(ValueError):
                checked_pose(data)
        with self.assertRaisesRegex(ValueError, 'does not match'):
            checked_pose(pose(), 'right_hand')

    def test_quaternion_sign_is_not_a_rotation_error(self):
        second = dict(pose(), orientation_xyzw=[0., -1., 0., 0.])
        self.assertEqual(pose_error(pose(), second), (0., 0.))
        second = dict(pose(), orientation_xyzw=[1., 0., 0., 0.])
        self.assertAlmostEqual(pose_error(pose(), second)[1], math.pi)

    def test_capture_file_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'poses/pick.json'
            write_json(output, pose())
            with self.assertRaises(FileExistsError):
                write_json(output, dict(pose(), position=[0., 0., 0.]))
            self.assertEqual(json.loads(output.read_text()), pose())

    def test_ik_joint_response_rejects_missing_duplicate_or_nonfinite_values(self):
        for message in (JointState(name=['right_j0'], position=[0.]),
                        JointState(name=['right_j0'] * 7, position=[0.] * 7),
                        JointState(name=list(JOINTS), position=[float('nan')] * 7)):
            with self.assertRaises(ValueError):
                checked_joints(message)


class CartesianTests(unittest.TestCase):
    def test_partial_joint_feedback_requires_all_joints_fresh(self):
        node = client()
        node.received['right_j0'] = time.monotonic() - 2.
        node.receive_joints(JointState(name=['right_gripper_l_finger_joint'], position=[0.]))
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            node.snapshot()
        node.receive_joints(JointState(name=['right_j0'], position=[0.], velocity=[0.]))
        self.assertEqual(node.snapshot(), dict.fromkeys(JOINTS, 0.))

    def test_moving_arm_and_invalid_arm_feedback_block_capture(self):
        node = client()
        node.velocities['right_j0'] = .1
        with self.assertRaisesRegex(RuntimeError, 'stationary'):
            node.snapshot()
        node.receive_joints(JointState(name=['right_j0', 'right_j0'], position=[0., 0.], velocity=[0., 0.]))
        self.assertIsNotNone(node.fault)

    def test_fk_rejects_unknown_tip_self_collision_and_malformed_responses(self):
        node = client()
        for response in (SolvePositionFK.Response(),
                         SolvePositionFK.Response(pose_stamp=[pose_message(pose())], is_valid=[False], in_collision=[False]),
                         SolvePositionFK.Response(pose_stamp=[pose_message(pose())], is_valid=[True], in_collision=[True])):
            node.service.return_value = response
            with self.assertRaises(RuntimeError):
                node.fk(dict.fromkeys(JOINTS, 0.), DEFAULT_TIP)

    def test_ik_uses_current_seed_and_verifies_pose_with_fk(self):
        node = client()
        node.service.return_value = SolvePositionIK.Response(result_type=[1],
            joints=[JointState(name=list(JOINTS), position=[.1] * 7)])
        node.fk = Mock(return_value=pose())
        result = node.ik(pose(), dict.fromkeys(JOINTS, 0.))
        self.assertEqual(result['status'], 'ik_valid')
        request = node.service.call_args.args[1]
        self.assertEqual(request.tip_names, [DEFAULT_TIP])
        self.assertEqual(request.seed_mode, SolvePositionIK.Request.SEED_USER)
        self.assertEqual(list(request.seed_angles[0].position), [0.] * 7)
        self.assertFalse(result['robot_commands_published'])
        node.fk.assert_called_once_with(dict.fromkeys(JOINTS, .1), DEFAULT_TIP)

    def test_ik_rejects_solver_failure_limits_and_fk_mismatch(self):
        node = client()
        for code in (-1, -2, -3, 0):
            node.service.return_value = SolvePositionIK.Response(result_type=[code])
            with self.assertRaisesRegex(RuntimeError, 'IK rejected'):
                node.ik(pose(), dict.fromkeys(JOINTS, 0.))
        node.service.return_value = SolvePositionIK.Response(result_type=[1],
            joints=[JointState(name=list(JOINTS), position=[2.] * 7)])
        with self.assertRaisesRegex(ValueError, 'outside'):
            node.ik(pose(), dict.fromkeys(JOINTS, 0.))
        node.service.return_value = SolvePositionIK.Response(result_type=[1],
            joints=[JointState(name=list(JOINTS), position=[0.] * 7)])
        node.fk = Mock(return_value=dict(pose(), position=[.5, .1, .2]))
        with self.assertRaisesRegex(RuntimeError, 'does not match'):
            node.ik(pose(), dict.fromkeys(JOINTS, 0.))

    def test_arm_moved_during_capture_does_not_write_file(self):
        node = client()
        with tempfile.TemporaryDirectory() as directory:
            node.args.output = Path(directory) / 'pick.json'
            node.snapshot = Mock(side_effect=[dict.fromkeys(JOINTS, 0.), dict.fromkeys(JOINTS, .01)])
            node.fk = Mock(return_value=pose())
            with self.assertRaisesRegex(RuntimeError, 'Arm moved'):
                node.run()
            self.assertFalse(node.args.output.exists())

    def test_invalid_cli_combinations_fail_before_ros_initialization(self):
        for args in (['capture'], ['ik'], ['read', '--pose', 'pick.json'], ['--timeout', '0']):
            with self.subTest(args=args), patch('sys.stderr'), self.assertRaises(SystemExit):
                parse_args(args)


if __name__ == '__main__':
    unittest.main()
