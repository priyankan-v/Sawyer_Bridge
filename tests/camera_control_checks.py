"""Run in Noetic: IO acknowledgements and single-camera policy without hardware."""
import importlib.util
import json
from pathlib import Path
import threading
import unittest
from unittest.mock import Mock, patch

import rospy
from intera_core_msgs.msg import IODataStatus, IODeviceStatus

spec = importlib.util.spec_from_file_location('camera_control', Path(__file__).resolve().parents[1] / 'scripts/camera_control.py')
control_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(control_module)


def state(enabled, command_time=None):
    message = IODeviceStatus()
    message.signals = [IODataStatus(name='camera_streaming', format='{"type":"bool"}',
                                    data=json.dumps([enabled]))]
    message.commands = [] if command_time is None else [command_time]
    return message


class ControlChecks(unittest.TestCase):
    def setUp(self):
        self.control = control_module.CameraControl.__new__(control_module.CameraControl)
        self.control.condition = threading.Condition()
        self.control.timeout = .03
        self.control.states = {'head': state(False), 'hand': state(True)}
        self.control.publishers = {}

    def test_bool_state_validation(self):
        self.assertTrue(control_module.streaming_value(state(True)))
        with self.assertRaises(ValueError):
            control_module.streaming_value(state('true'))

    def test_acknowledged_command_fields(self):
        publisher = Mock()
        publisher.get_num_connections.return_value = 1
        self.control.publishers['head'] = publisher

        def acknowledge(command):
            self.assertEqual(command.op, 'set')
            self.assertEqual(json.loads(command.args), {
                'signals': {'camera_streaming': {'format': {'type': 'bool'}, 'data': [True]}}})
            self.control.receive('head', state(True, command.time))

        publisher.publish.side_effect = acknowledge
        with patch.object(rospy, 'is_shutdown', return_value=False), patch.object(rospy.Time, 'now', return_value=rospy.Time(123)):
            self.control.set_streaming('head', True)
        self.assertTrue(self.control.status()['head'])

    def test_unacknowledged_command_fails_even_if_state_changes(self):
        publisher = Mock()
        publisher.get_num_connections.return_value = 1
        self.control.publishers['head'] = publisher
        publisher.publish.side_effect = lambda command: self.control.receive('head', state(True))
        with patch.object(rospy, 'is_shutdown', return_value=False), patch.object(rospy.Time, 'now', return_value=rospy.Time(123)):
            with self.assertRaisesRegex(RuntimeError, 'no acknowledged'):
                self.control.set_streaming('head', True)

    def test_redundant_command_creates_no_publisher(self):
        self.control.set_streaming('head', False)
        self.assertEqual(self.control.publishers, {})

    def test_stop_other_before_start(self):
        order = []

        def update(key, enabled):
            order.append((key, enabled))
            self.control.receive(key, state(enabled))

        self.control.set_streaming = update
        self.control.start('head')
        self.assertEqual(order, [('hand', False), ('head', True)])

    def test_failed_stop_prevents_start(self):
        self.control.set_streaming = Mock(side_effect=RuntimeError('stop failed'))
        with self.assertRaisesRegex(RuntimeError, 'stop failed'):
            self.control.start('head')
        self.control.set_streaming.assert_called_once_with('hand', False)

    def test_stop_all(self):
        self.control.set_streaming = Mock()
        self.control.stop('all')
        self.assertEqual(self.control.set_streaming.call_count, 2)
        self.control.set_streaming.assert_any_call('head', False)
        self.control.set_streaming.assert_any_call('hand', False)

    def test_explicit_both_does_not_stop_other_camera(self):
        order = []

        def update(key, enabled):
            order.append((key, enabled))
            self.control.receive(key, state(enabled))

        self.control.set_streaming = update
        self.control.start_both()
        self.assertEqual(order, [('head', True), ('hand', True)])


if __name__ == '__main__':
    unittest.main()
