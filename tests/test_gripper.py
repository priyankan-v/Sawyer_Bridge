"""Gripper protocol, feedback guards and CLI checks without a robot connection."""
import json
import time
import unittest
from unittest.mock import Mock, patch

from builtin_interfaces.msg import Time
from intera_core_msgs.msg import IOComponentStatus, IODataStatus, IODeviceStatus, IOStatus
from sawyer_ros2_control.gripper import Gripper, MAX_POSITION, decode_signals, parse_args


def feedback(**overrides):
    values = dict(has_error=False, is_calibrated=True, is_moving=False,
                  is_gripping=False, position_response_m=0., position_m=0.,
                  go=True, calibrate=False, dead_zone_m=0.002)
    values.update(overrides)
    return IODeviceStatus(device=IOComponentStatus(name='right_gripper', status=IOStatus(tag='ready')),
                          signals=[IODataStatus(name=name, data=json.dumps([value]),
                                                format=json.dumps(dict(type='bool' if type(value) is bool else 'float')),
                                                status=IOStatus(tag='ready')) for name, value in values.items()])


def client(operation='open', **signals):
    node = Gripper.__new__(Gripper)
    node.args = parse_args([operation] + (['--position', '0.02'] if operation == 'position' else []))
    node.publisher = None
    node.count_publishers = Mock(return_value=1)
    node.create_publisher = Mock()
    node.receive(feedback(**signals))
    node.wait_until = Mock(side_effect=lambda predicate, description, check=None: (
        check() if check else None, predicate())[-1])
    node.send = Mock()
    return node


class GripperTests(unittest.TestCase):
    def test_signal_protocol_matches_intera_set_command(self):
        node = client()
        command = node.command(dict(position_m=0.02, go=True))
        self.assertEqual(command.op, 'set')
        self.assertEqual(json.loads(command.args), dict(signals={
            'position_m': dict(format=dict(type='float'), data=[0.02]),
            'go': dict(format=dict(type='bool'), data=[True])}))

    def test_status_and_dry_run_create_no_command_publisher(self):
        for operation, dry_run in [('status', False), ('open', True), ('calibrate', True)]:
            with self.subTest(operation=operation):
                node = client(operation)
                node.args.dry_run = dry_run
                report = node.run()
                self.assertEqual(report['status'], 'dry_run' if dry_run else 'status')
                node.send.assert_not_called()
                node.create_publisher.assert_not_called()

    def test_operations_send_expected_signals(self):
        for operation, expected in [('open', dict(position_m=MAX_POSITION, go=True)),
                                    ('close', dict(position_m=0., go=True)),
                                    ('position', dict(position_m=0.02, go=True)),
                                    ('calibrate', dict(calibrate=True)), ('stop', dict(go=False))]:
            node = client(operation)
            node.run()
            self.assertEqual({name: value['data'][0] for name, value in
                              json.loads(node.send.call_args.args[0].args)['signals'].items()}, expected)

    def test_faults_uncalibrated_and_invalid_feedback_block_motion(self):
        for signals in [dict(has_error=True), dict(is_calibrated=False), dict(has_error=0),
                        dict(position_m=float('nan'))]:
            with self.subTest(signals=signals):
                node = client(**signals)
                with self.assertRaises(RuntimeError):
                    node.run()
                node.send.assert_not_called()
        node = client()
        node.count_publishers.return_value = 0
        with self.assertRaisesRegex(RuntimeError, 'disconnected'):
            node.run()

    def test_idle_retained_feedback_does_not_require_a_heartbeat(self):
        node = client()
        with patch('sawyer_ros2_control.gripper.time.monotonic', return_value=time.monotonic() + 60.):
            node.check_ready()

    def test_missing_or_unavailable_signals_block_motion(self):
        for change in ('missing', 'unready'):
            node = client()
            if change == 'missing':
                del node.signals['position_m']
            else:
                node.signals['position_m']['status'] = 'busy'
            with self.assertRaises(RuntimeError):
                node.run()
            node.send.assert_not_called()

    def test_calibration_accepts_uncalibrated_and_busy_feedback(self):
        node = client('calibrate', is_calibrated=False)
        node.message.device.status.tag = 'unready'
        node.check_ready()
        node.message.device.status.tag = 'busy'
        node.check_running()
        self.assertFalse(node.completed(None))
        node.receive(feedback())
        self.assertTrue(node.completed(None))

    def test_uncalibrated_unready_device_rejects_motion_with_calibration_hint(self):
        node = client(is_calibrated=False)
        node.message.device.status.tag = 'unready'
        with self.assertRaisesRegex(RuntimeError, 'not calibrated'):
            node.run()
        node.send.assert_not_called()

    def test_calibration_still_rejects_faults_and_inoperative_device(self):
        node = client('calibrate', has_error=True, is_calibrated=False)
        node.message.device.status.tag = 'unready'
        with self.assertRaises(RuntimeError):
            node.run()
        node.receive(feedback(is_calibrated=False))
        node.message.device.status.tag = 'down'
        with self.assertRaises(RuntimeError):
            node.run()
        node.send.assert_not_called()

    def test_stop_is_available_during_fault_or_without_calibration(self):
        node = client('stop', has_error=True, is_calibrated=False)
        node.message.device.status.tag = 'error'
        node.run()
        node.send.assert_called_once()

    def test_completion_requires_stopped_feedback_at_target_or_close_grasp(self):
        node = client(position_response_m=MAX_POSITION)
        self.assertTrue(node.completed(MAX_POSITION))
        node.receive(feedback(position_response_m=MAX_POSITION, is_moving=True))
        self.assertFalse(node.completed(MAX_POSITION))
        node.receive(feedback(position_response_m=0.02, is_gripping=True))
        self.assertFalse(node.completed(MAX_POSITION))
        node.args.operation = 'close'
        self.assertTrue(node.completed(0.))
        node.args.operation = 'position'
        self.assertFalse(node.completed(0.))

    def test_live_open_residual_within_device_dead_zone_completes(self):
        node = client(position_response_m=0.0405509196428571)
        self.assertAlmostEqual(node.position_tolerance(), 0.002)
        self.assertTrue(node.completed(MAX_POSITION))
        node.receive(feedback(position_response_m=0.0405509196428571, is_moving=True))
        self.assertFalse(node.completed(MAX_POSITION))
        node.receive(feedback(position_response_m=MAX_POSITION - 0.0021))
        self.assertFalse(node.completed(MAX_POSITION))

    def test_explicit_tolerance_overrides_device_dead_zone(self):
        node = client(position_response_m=0.0405509196428571)
        node.args.tolerance = 0.001
        self.assertFalse(node.completed(MAX_POSITION))
        node.args.tolerance = 0.0015
        self.assertTrue(node.completed(MAX_POSITION))

    def test_missing_dead_zone_uses_original_one_mm_fallback(self):
        node = client(position_response_m=0.0405509196428571)
        del node.signals['dead_zone_m']
        self.assertEqual(node.position_tolerance(), 0.001)
        self.assertFalse(node.completed(MAX_POSITION))

    def test_invalid_device_dead_zone_blocks_motion(self):
        for dead_zone in (-0.001, MAX_POSITION, float('nan'), True):
            with self.subTest(dead_zone=dead_zone):
                node = client(dead_zone_m=dead_zone)
                with self.assertRaises(RuntimeError):
                    node.run()
                node.send.assert_not_called()

    def test_malformed_state_replaces_old_values_and_blocks_commands(self):
        node = client()
        bad = feedback()
        bad.signals[0].data = 'false'
        node.receive(bad)
        self.assertEqual(node.signals, {})
        with self.assertRaisesRegex(RuntimeError, 'Invalid gripper feedback'):
            node.run()
        with self.assertRaises(ValueError):
            decode_signals(bad)

    def test_wrong_device_blocks_commands(self):
        node = client()
        node.message.device.name = 'other'
        with self.assertRaisesRegex(RuntimeError, 'Unexpected device'):
            node.run()

    @patch('sawyer_ros2_control.gripper.rclpy.spin_once')
    @patch('sawyer_ros2_control.gripper.rclpy.ok', return_value=True)
    def test_wait_rejects_timeout_and_disconnected_publisher(self, ok, spin):
        node = client()
        node.args.timeout = .001
        with self.assertRaisesRegex(RuntimeError, 'Timed out'):
            Gripper.wait_until(node, lambda: False, 'test')
        node.count_publishers.return_value = 0
        with self.assertRaisesRegex(RuntimeError, 'disconnected'):
            Gripper.wait_until(node, lambda: True, 'test', node.check_running)

    def test_retries_keep_timestamp_and_require_matching_acknowledgement(self):
        node = client()
        del node.send
        command = node.command(dict(go=True))
        node.get_clock = Mock()
        node.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=123, nanosec=456)
        node.create_publisher.return_value.get_subscription_count.return_value = 1
        node.message.commands = [Time(sec=123, nanosec=455)]

        def wait(predicate, description, check=None):
            if 'subscription' in description:
                self.assertTrue(predicate())
                return
            with patch('sawyer_ros2_control.gripper.time.monotonic', side_effect=[1., 1., 2., 2.]):
                self.assertFalse(predicate())
                self.assertFalse(predicate())
            node.message.commands.append(Time(sec=123, nanosec=456))
            self.assertTrue(predicate())

        node.wait_until = wait
        node.send(command)
        pub = node.create_publisher.return_value
        self.assertEqual(pub.publish.call_count, 2)
        self.assertEqual(command.time, Time(sec=123, nanosec=456))

    def test_cli_rejects_invalid_targets_and_timeouts(self):
        for argv in [['position'], ['open', '--position', '0.02'], ['position', '--position', '-1'],
                     ['position', '--position', '0.05'], ['position', '--position', 'nan'],
                     ['--timeout', 'inf'], ['--timeout', '0'], ['--tolerance', '0.1'],
                     ['--tolerance', 'nan'], ['--tolerance', '0']]:
            with self.subTest(argv=argv), patch('sys.stderr'), self.assertRaises(SystemExit):
                parse_args(argv)
        self.assertEqual(parse_args([]).operation, 'status')
        self.assertIsNone(parse_args([]).tolerance)
        self.assertEqual(parse_args(['--tolerance', '0.002']).tolerance, 0.002)


if __name__ == '__main__':
    unittest.main()
