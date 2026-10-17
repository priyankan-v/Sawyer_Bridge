"""System Humble unit checks for the state relay and plan-only coordinator."""
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'planning'))
from builtin_interfaces.msg import Time
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState
from joint_state_relay import JointStateRelay
from plan_pick_place import PickPlacePlanner


class RelayTests(unittest.TestCase):
    def node(self):
        node = JointStateRelay.__new__(JointStateRelay)
        node.required = {'right_j0', 'head_pan', 'right_gripper_l_finger_joint'}
        node.positions, node.received = {}, {}
        node.offline = False
        node.publisher, node.get_clock, node.get_logger = Mock(), Mock(), Mock()
        node.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=123)
        return node

    def test_partial_feedback_merges_and_converts_gripper_without_clamping_arm(self):
        node = self.node()
        node.receive(JointState(name=['right_gripper_l_finger_joint'], position=[.04]))
        node.publish()
        node.publisher.publish.assert_not_called()
        node.receive(JointState(name=['right_j0', 'head_pan'], position=[-4.7, .2]))
        node.publish()
        result = node.publisher.publish.call_args.args[0]
        self.assertEqual(dict(zip(result.name, result.position)),
                         dict(right_j0=-4.7, head_pan=.2, right_gripper_l_finger_joint=.02))
        self.assertEqual(result.header.stamp.sec, 123)

    def test_arm_staleness_stops_output_but_on_change_gripper_silence_is_valid(self):
        node = self.node()
        node.receive(JointState(name=sorted(node.required), position=[0., .02, 0.]))
        node.received['right_gripper_l_finger_joint'] -= 60.
        node.publish()
        node.publisher.publish.assert_called_once()
        node.received['right_j0'] -= 2.
        node.publish()
        node.publisher.publish.assert_called_once()

    def test_malformed_feedback_invalidates_freshness(self):
        node = self.node()
        node.receive(JointState(name=['right_j0', 'right_j0'], position=[0., 0.]))
        node.publish()
        node.publisher.publish.assert_not_called()


class PlannerTests(unittest.TestCase):
    def node(self):
        node = PickPlacePlanner.__new__(PickPlacePlanner)
        node.bounds = {f'right_j{i}': (-1., 1.) for i in range(7)}
        node.bounds['right_gripper_l_finger_joint'] = (0., .02)
        node.positions = dict.fromkeys(node.bounds, 0.)
        node.received = time.monotonic()
        node.state_source = 'offline_snapshot'
        node.call = Mock(return_value=GetStateValidity.Response(valid=True))
        return node

    def test_out_of_bounds_or_missing_joints_are_rejected_before_planning(self):
        node = self.node()
        for changed in (dict(node.positions, right_j6=-1.001), {'right_j0': 0.}):
            with self.assertRaisesRegex(RuntimeError, 'outside'):
                node.validate(changed)
            node.call.assert_not_called()

    def test_sequence_orders_grasp_and_release_and_contains_no_execution(self):
        node = self.node()
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)
            pose = dict(frame_id='base', tip_name='right_gripper_tip', position=[.5, .1, .2], orientation_xyzw=[0., 1., 0., 0.])
            (path / 'pose.json').write_text(json.dumps(pose))
            (path / 'manifest.json').write_text(json.dumps(dict(source_sha256='test')))
            node.args = SimpleNamespace(pick=path / 'pose.json', place=path / 'pose.json', model=path,
                                        approach=[0., 0., 1.], clearance=.05)
            states = {}

            def move(name, start, goal):
                states[name] = dict(start)
                return dict(kind='arm', name=name, trajectory={}), dict(start)

            node.joint_move = Mock(side_effect=move)
            node.cartesian_move = Mock(side_effect=move)
            report = node.run()
            self.assertFalse(report['execution_enabled'])
            self.assertEqual([stage['name'] for stage in report['stages']],
                ['open_before_pick', 'pre_pick', 'pick', 'grasp', 'lift', 'pre_place', 'place', 'release', 'retreat'])
            self.assertEqual(states['pick']['right_gripper_l_finger_joint'], .02)
            self.assertEqual(states['lift']['right_gripper_l_finger_joint'], 0.)
            self.assertEqual(states['retreat']['right_gripper_l_finger_joint'], .02)


if __name__ == '__main__':
    unittest.main()
