"""Task path retiming and execution guards; no ROS graph or robot commands."""
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from sawyer_ros2_control.pick_place import ArmExecutor, ORDER, read_plan, run
from sawyer_ros2_control.safety import JOINTS, Limit
from sawyer_ros2_control.task_trajectory import retime, sample, waypoints


class TrajectoryTests(unittest.TestCase):
    def test_retiming_stays_on_planned_segments_and_respects_derivative_caps(self):
        limits = dict.fromkeys(JOINTS, Limit(-1., 1., .015, .03))
        points = [dict.fromkeys(JOINTS, 0.), {j: .001 * (i + 1) for i, j in enumerate(JOINTS)}, dict.fromkeys(JOINTS, .01)]
        segments = retime(points, limits)
        self.assertEqual(sample(segments, 0.), points[0])
        final = sample(segments, sum(segment.duration for segment in segments) + 1.)
        for joint in JOINTS:
            self.assertAlmostEqual(final[joint], points[-1][joint])
        for segment, first, last in zip(segments, points, points[1:]):
            for fraction in (0., .2, .5, .8, 1.):
                q, dq, ddq = segment.sample(segment.duration * fraction)
                factors = []
                for j in JOINTS:
                    self.assertLessEqual(abs(dq[j]), .015 + 1e-12)
                    self.assertLessEqual(abs(ddq[j]), .03 + 1e-12)
                    factors.append((q[j] - first[j]) / (last[j] - first[j]))
                self.assertLess(max(factors) - min(factors), 1e-10)

    def test_large_jumps_and_invalid_trajectory_joints_are_rejected(self):
        limits = dict.fromkeys(JOINTS, Limit(-1., 1., .015, .03))
        with self.assertRaisesRegex(ValueError, 'displacement'):
            retime([dict.fromkeys(JOINTS, 0.), dict.fromkeys(JOINTS, .3)], limits)
        with self.assertRaises(ValueError):
            waypoints(dict(joint_trajectory=dict(joint_names=['right_j0'] * 7, points=[])))

    def test_exact_limit_endpoint_is_preserved_without_roundoff_overshoot(self):
        lower = -4.7124
        limits = dict.fromkeys(JOINTS, Limit(lower, 4.7124, .015, .03))
        start, target = dict.fromkeys(JOINTS, lower + .01), dict.fromkeys(JOINTS, lower)
        segments = retime([start, target], limits)
        self.assertEqual(sample(segments, segments[0].duration + 1.), target)
        for index in range(1001):
            self.assertTrue(all(lower <= value <= lower + .01 for value in sample(segments, segments[0].duration * index / 1000).values()))


class TaskTests(unittest.TestCase):
    def fixture(self, path):
        (path / 'manifest.json').write_text(json.dumps(dict(source_sha256='test')))
        (path / 'planning_limits.json').write_text(json.dumps(dict(joint_limits={
            j: dict(max_velocity=.15, max_acceleration=.3) for j in JOINTS})))
        (path / 'sawyer.urdf').write_text('<robot name="test">' + ''.join(
            f'<joint name="{j}" type="revolute"><limit lower="-1" upper="1"/></joint>' for j in JOINTS) + '</robot>')
        stages = []
        for index, name in enumerate(ORDER):
            if index in (0, 3, 7):
                stages.append(dict(name=name, kind='gripper', operation='close' if index == 3 else 'open', require_grasp=index == 3))
            else:
                stages.append(dict(name=name, kind='arm', trajectory=dict(joint_trajectory=dict(
                    joint_names=list(JOINTS), points=[dict(positions=[0.] * 7)]))))
        plan = dict(status='planned', model_source_sha256='test', planned_at_unix=time.time(),
            state_source='live_feedback', collision_scope='self-collision only',
            start_state=dict(dict.fromkeys(JOINTS, 0.), head_pan=0.), stages=stages)
        (path / 'plan.json').write_text(json.dumps(plan))
        args = SimpleNamespace(plan=path / 'plan.json', model=path, execute=False, max_velocity=.015, max_acceleration=.03)
        return args, plan

    def test_preview_does_not_actuate(self):
        with tempfile.TemporaryDirectory() as temp, patch('sawyer_ros2_control.pick_place.gripper_operation') as gripper:
            args, _ = self.fixture(Path(temp))
            result = run(args, read_plan(args))
            self.assertEqual(result['status'], 'preview')
            gripper.assert_not_called()

    def test_offline_old_or_incomplete_plans_cannot_execute(self):
        for change in (dict(state_source='offline_snapshot'), dict(planned_at_unix=time.time() - 130), dict(stages=[])):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as temp:
                path = Path(temp)
                args, plan = self.fixture(path)
                args.execute = True
                plan.update(change)
                args.plan.write_text(json.dumps(plan))
                with self.assertRaises(ValueError):
                    read_plan(args)

    def test_failed_grasp_prevents_lift_and_does_not_open_on_failure(self):
        with tempfile.TemporaryDirectory() as temp:
            args, _ = self.fixture(Path(temp))
            args.execute = True
            plan = read_plan(args)
            fake = Mock()
            fake.positions = dict.fromkeys(JOINTS, 0.)
            status = dict(signals=dict(has_error=False, is_calibrated=True, position_response_m=.04, is_gripping=False))
            with patch('sawyer_ros2_control.pick_place.ArmExecutor', return_value=fake), \
                 patch('sawyer_ros2_control.pick_place.check_path', return_value=dict(head_pan=0., waypoints=[fake.positions])), \
                 patch('sawyer_ros2_control.pick_place.gripper_operation', return_value=status) as gripper:
                with self.assertRaisesRegex(RuntimeError, 'did not detect a grasp'):
                    run(args, plan)
                self.assertEqual([call.args[0] for call in gripper.call_args_list], ['status', 'open', 'close'])
                self.assertEqual(fake.execute.call_count, 2)

    def test_lost_grasp_stops_transport(self):
        fake = ArmExecutor.__new__(ArmExecutor)
        fake.head_received, fake.expected_head, fake.head_pan = time.monotonic(), 0., 0.
        fake.expect_grasp, fake.gripping, fake.gripper_fault = True, False, False
        with patch('sawyer_ros2_control.pick_place.Controller.check', return_value={}):
            with self.assertRaisesRegex(RuntimeError, 'Grasp was lost'):
                ArmExecutor.check(fake)

    def test_hardware_execution_rejects_simulated_clock_before_publishing(self):
        fake = ArmExecutor.__new__(ArmExecutor)
        fake.get_parameter = Mock(return_value=SimpleNamespace(value=True))
        fake.wait_for_feedback = Mock()
        with self.assertRaisesRegex(RuntimeError, 'simulated time'):
            fake.execute([])
        fake.wait_for_feedback.assert_not_called()


if __name__ == '__main__':
    unittest.main()
