import contextlib
import io
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/sawyer_ros2_control'))
from sawyer_ros2_control.move_joint import JointMover, minimum_duration, parse_args, quintic_position
from intera_core_msgs.msg import HeadPanCommand, HeadState, JointCommand, JointLimits
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class SimulatedMover(JointMover):
    def __init__(self, args, clock):
        self.args, self.clock = args, clock
        self.position = 0.
        self.position_received = clock[0]
        self.position_stamp = 0
        self.position_sequence = 1
        self.bounds = (-1., 1.)
        self.velocity_limit = 2.
        self.acceleration_limit = 10.
        self.minimum_time = 0.
        self.head_state = None
        self.head_state_received = None
        self.model = dict(lower=-1., upper=1., velocity=2.)
        self.error = None
        self.pub = None
        self.messages = []
        self.settings = []

    def refresh_feedback(self):
        self.position_received = self.clock[0]
        self.position_sequence += 1

    def create_publisher(self, msg_type, topic, qos):
        if msg_type == Float64:
            return NS(publish=lambda msg: self.settings.append(msg.data), get_subscription_count=lambda: 1)
        self.pub = NS(publish=self.capture, get_subscription_count=lambda: 1)
        return self.pub

    def capture(self, msg):
        self.messages.append(msg)
        self.position = float(msg.target) if isinstance(msg, HeadPanCommand) else msg.position[0]

    def get_clock(self):
        return NS(now=lambda: NS(to_msg=lambda: Time(sec=123)))


class MoveJointTests(unittest.TestCase):
    def test_direct_execution_and_argument_validation(self):
        args = parse_args(['--joint', 'head_pan', '--distance', '-.1', '--time', '2'])
        self.assertFalse(args.dry_run)
        self.assertFalse(hasattr(args, 'operator_approved'))
        self.assertEqual(args.settle_timeout, 0.)
        self.assertEqual(args.tolerance, .008726646)
        for options in (['--joint', 'bad', '--position', '0', '--time', '2'],
                        ['--joint', 'right_j0', '--distance', '.1', '--position', '0', '--time', '2'],
                        ['--joint', 'right_j0', '--distance', 'nan', '--time', '2'],
                        ['--joint', 'right_j0', '--distance', '.1', '--time', '0'],
                        ['--joint', 'right_j0', '--distance', '.1', '--time', '2', '--settle-timeout', '-1'],
                        ['--joint', 'right_j0', '--distance', '.1', '--time', '2', '--settle-timeout', 'nan']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(options)

    def test_quintic_shape_and_zero_endpoint_derivatives(self):
        for start, target in ((0., .1), (.1, 0.)):
            self.assertEqual(quintic_position(start, target, 0, 2), start)
            self.assertAlmostEqual(quintic_position(start, target, 2, 2), target)
            self.assertAlmostEqual(quintic_position(start, target, 1, 2), (start+target)/2)
            h = .0001
            for t in (0., 2.):
                q = quintic_position(start, target, t, 2)
                left = quintic_position(start, target, t-h, 2)
                right = quintic_position(start, target, t+h, 2)
                self.assertAlmostEqual((right-left)/(2*h), 0., places=6)
                self.assertAlmostEqual((right-2*q+left)/h**2, 0., places=3)

    def test_pi_expressions_for_absolute_and_relative_angles(self):
        expressions = [('pi', math.pi), ('pi/2', math.pi/2), ('-pi/4', -math.pi/4),
                       ('2*pi/3', 2*math.pi/3), ('(pi - 0.1)/2', (math.pi-.1)/2),
                       ('1e-3', .001), ('-.1', -.1)]
        for option in ('--position', '--distance'):
            for expression, expected in expressions:
                args = parse_args(['--joint', 'right_j0', option+'='+expression, '--time', '5'])
                self.assertAlmostEqual(getattr(args, option[2:]), expected)

    def test_angle_expressions_reject_code_and_invalid_arithmetic(self):
        for expression in ('math.pi', 'pi**2', '1/0', 'nan', 'inf', '1e309', 'True', '[]',
                           '__import__("os").system("false")', 'pi/' , '1+'*100+'1'):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(['--joint', 'right_j0', '--position='+expression, '--time', '5'])

    def test_space_separated_negative_angles_and_real_cli_arguments(self):
        arguments = ['--position', '-pi/2', '--time', '5', '--joint', 'right_j2']
        self.assertAlmostEqual(parse_args(arguments).position, -math.pi/2)
        with patch.object(sys, 'argv', ['scripts/move_joint.py'] + arguments):
            self.assertAlmostEqual(parse_args().position, -math.pi/2)
        for option in ('--position', '--distance'):
            for expression, expected in (('-pi/2', -math.pi/2), ('-(pi / 4)', -math.pi/4),
                                         ('-2*pi/3', -2*math.pi/3), ('-1e-3', -.001)):
                args = parse_args([option, expression, '--time', '5', '--joint', 'right_j2'])
                self.assertAlmostEqual(getattr(args, option[2:]), expected)

    def test_angle_normalization_preserves_options_and_rejects_invalid_values(self):
        for arguments in (['--position', '--time', '5', '--joint', 'right_j2'],
                          ['--joint', 'right_j2', '--time', '5', '--position'],
                          ['--position', '-pi/0', '--time', '5', '--joint', 'right_j2'],
                          ['--position', '-__import__("os")', '--time', '5', '--joint', 'right_j2']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                parse_args(arguments)
            self.assertEqual(error.exception.code, 2)
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(SystemExit) as error:
            parse_args(['--position', '-pi/2', '-h'])
        self.assertEqual(error.exception.code, 0)

    def test_pi_target_still_obeys_joint_limits_before_publication(self):
        args = parse_args(['--joint', 'right_j0', '--position', 'pi/2', '--time', '5'])
        node = SimulatedMover(args, [0.])  # This fixture's upper limit is 1 rad.
        with patch('sawyer_ros2_control.move_joint.time.monotonic', return_value=0.), \
                patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                self.assertRaisesRegex(ValueError, 'outside joint limits'):
            node.run()
        self.assertIsNone(node.pub)
        self.assertEqual(node.messages, [])

    def test_end_options_and_relative_end_rejection(self):
        for end in ('-end', '+end'):
            args = parse_args(['--position', end, '--time', '5', '--joint', 'right_j5'])
            self.assertEqual(args.position, end)
            self.assertEqual(parse_args(['--position='+end, '--time', '5', '--joint', 'right_j5']).position, end)
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(['--distance', end, '--time', '5', '--joint', 'right_j5'])

    def test_end_targets_follow_live_and_urdf_limits_in_dry_run(self):
        for end, expected in (('-end', -.7), ('+end', .6)):
            args = parse_args(['--joint', 'right_j5', '--position', end, '--time', '2', '--dry-run'])
            node = SimulatedMover(args, [0.])
            node.model = dict(lower=-.8, upper=.9, velocity=2.)
            node.on_limits(JointLimits(joint_names=['right_j5'], position_lower=[-.7], position_upper=[.6],
                                      velocity=[2.], accel=[10.]))
            self.assertEqual(node.resolve_target(0.), expected)
            with patch('sawyer_ros2_control.move_joint.time.monotonic', return_value=0.), \
                    patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                    contextlib.redirect_stdout(io.StringIO()):
                node.run()
            self.assertIsNone(node.pub)
            node.on_limits(JointLimits(joint_names=['right_j5'], position_lower=[-.5], position_upper=[.4],
                                      velocity=[2.], accel=[10.]))
            self.assertEqual(node.resolve_target(0.), -.5 if end == '-end' else .4)

    def test_execute_both_limit_endpoints_without_overshoot_and_reject_short_time(self):
        for end, expected in (('-end', -.7), ('+end', .6)):
            clock = [0.]
            args = parse_args(['--joint', 'right_j5', '--position', end, '--time', '2'])
            node = SimulatedMover(args, clock)
            node.bounds = (-.7, .6)
            def spin(_node, timeout_sec):
                clock[0] += max(timeout_sec, .001)
                node.refresh_feedback()
            with patch('sawyer_ros2_control.move_joint.time.monotonic', side_effect=lambda: clock[0]), \
                    patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                    patch('sawyer_ros2_control.move_joint.rclpy.spin_once', side_effect=spin), \
                    contextlib.redirect_stdout(io.StringIO()):
                node.run()
                self.assertEqual(node.messages[-1].position[0], expected)
                self.assertTrue(all(-.7 <= m.position[0] <= .6 and m.names == ['right_j5'] for m in node.messages))
                count = len(node.messages)
                node.run()
                self.assertEqual(len(node.messages), count)
            args.duration = .01
            node = SimulatedMover(args, [0.])
            with patch('sawyer_ros2_control.move_joint.time.monotonic', return_value=0.), \
                    patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                    self.assertRaisesRegex(ValueError, 'minimum quintic time.*No commands sent'):
                node.run()
            self.assertIsNone(node.pub)

    def test_head_limit_endpoints_round_inward_to_float32(self):
        for bounds in ((-.1, .1), (.7, .9), (-.9, -.7)):
            for end in ('-end', '+end'):
                args = parse_args(['--joint', 'head_pan', '--position', end, '--time', '5'])
                node = SimulatedMover(args, [0.])
                node.bounds = bounds
                target = node.resolve_target(0.)
                self.assertGreaterEqual(target, bounds[0])
                self.assertLessEqual(target, bounds[1])
                self.assertAlmostEqual(target, bounds[0] if end == '-end' else bounds[1], places=6)
                self.assertEqual(node.wire_position(target), target)

    def test_operator_lower_limit_overshoot_completes_and_repeated_end_sends_nothing(self):
        clock = [0.]
        args = parse_args(['--joint', 'right_j6', '--position', '-end', '--time', '10'])
        node = SimulatedMover(args, clock)
        node.bounds = (-4.7124, 4.7124)
        node.velocity_limit = 4.545
        node.position = 1.567699
        overshoot = 4.7140419921875 - 4.7124
        def capture(msg):
            node.messages.append(msg)
            node.position = msg.position[0] - overshoot
        def spin(_node, timeout_sec):
            clock[0] += max(timeout_sec, .001)
            node.refresh_feedback()
        node.capture = capture
        output = io.StringIO()
        with patch('sawyer_ros2_control.move_joint.time.monotonic', side_effect=lambda: clock[0]), \
                patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                patch('sawyer_ros2_control.move_joint.rclpy.spin_once', side_effect=spin), \
                contextlib.redirect_stdout(output):
            node.run()
            self.assertAlmostEqual(node.position, -4.7140419921875)
            self.assertEqual(node.messages[-1].position[0], -4.7124)
            self.assertTrue(all(-4.7124 <= msg.position[0] <= 4.7124 for msg in node.messages))
            count = len(node.messages)
            node.run()
            self.assertEqual(len(node.messages), count)
        self.assertIn('"status": "reached"', output.getvalue())
        self.assertIn('"status": "already_at_target"', output.getvalue())
        self.assertIn('"limit_overshoot_rad":', output.getvalue())

    def test_recovery_from_small_measured_overshoot_keeps_all_commands_in_limits(self):
        for end, start, goal, bias in (('+end', -4.7140419921875, 4.7124, .0016419921875),
                                       ('-end', 4.7140419921875, -4.7124, -.0016419921875)):
            clock = [0.]
            args = parse_args(['--joint', 'right_j6', '--position', end, '--time', '10'])
            node = SimulatedMover(args, clock)
            node.bounds = (-4.7124, 4.7124)
            node.velocity_limit = 4.545
            node.position = start
            def capture(msg):
                node.messages.append(msg)
                node.position = msg.position[0] + bias
            def spin(_node, timeout_sec):
                clock[0] += max(timeout_sec, .001)
                node.refresh_feedback()
            node.capture = capture
            with patch('sawyer_ros2_control.move_joint.time.monotonic', side_effect=lambda: clock[0]), \
                    patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                    patch('sawyer_ros2_control.move_joint.rclpy.spin_once', side_effect=spin), \
                    contextlib.redirect_stdout(io.StringIO()):
                node.run()
            self.assertEqual(node.reference_start, -4.7124 if start < 0 else 4.7124)
            self.assertEqual(node.messages[-1].position[0], goal)
            self.assertTrue(all(-4.7124 <= msg.position[0] <= 4.7124 for msg in node.messages))

    def test_feedback_allowance_does_not_relax_command_limits_or_hide_large_overshoot(self):
        for options, position in ((['--position', '0', '--tolerance', '1'], -1.02),
                                  (['--position', '-1.001'], -1.001)):
            args = parse_args(['--joint', 'right_j6', '--time', '5'] + options)
            node = SimulatedMover(args, [0.])
            node.position = position
            with patch('sawyer_ros2_control.move_joint.time.monotonic', return_value=0.), \
                    patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                    self.assertRaisesRegex(ValueError, 'outside joint limits'):
                node.run()
            self.assertIsNone(node.pub)
            self.assertEqual(node.messages, [])

    def test_relative_recovery_uses_actual_feedback_and_rejects_further_outward_target(self):
        clock = [0.]
        args = parse_args(['--joint', 'right_j6', '--distance', '.1', '--time', '1'])
        node = SimulatedMover(args, clock)
        node.position = -1.001642
        expected = node.position + .1
        def spin(_node, timeout_sec):
            clock[0] += max(timeout_sec, .001)
            node.refresh_feedback()
        with patch('sawyer_ros2_control.move_joint.time.monotonic', side_effect=lambda: clock[0]), \
                patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                patch('sawyer_ros2_control.move_joint.rclpy.spin_once', side_effect=spin), \
                contextlib.redirect_stdout(io.StringIO()):
            node.run()
        self.assertEqual(node.messages[-1].position[0], expected)
        self.assertTrue(all(-1. <= msg.position[0] <= 1. for msg in node.messages))
        args.distance = -.01
        node = SimulatedMover(args, [0.])
        node.position = -1.001642
        with patch('sawyer_ros2_control.move_joint.time.monotonic', return_value=0.), \
                patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                self.assertRaisesRegex(ValueError, 'outside joint limits'):
            node.run()
        self.assertEqual(node.messages, [])

    def test_live_and_urdf_position_limits(self):
        args = parse_args(['--joint', 'head_pan', '--distance', '.1', '--time', '2'])
        node = SimulatedMover(args, [0.])
        node.model = dict(lower=-.8, upper=.9, velocity=2.)
        node.on_limits(JointLimits(joint_names=['head_pan'], position_lower=[-1.], position_upper=[.5],
                                  velocity=[1.5], accel=[3.]))
        self.assertEqual(node.bounds, (-.8, .5))
        self.assertEqual(node.velocity_limit, 1.5)
        self.assertEqual(node.acceleration_limit, 3.)
        for q in (-.81, .51, math.nan):
            with self.assertRaises(ValueError):
                node.check_position(q)
        node.on_limits(JointLimits(joint_names=['head_pan'], position_lower=[], position_upper=[.5]))
        with self.assertRaisesRegex(RuntimeError, 'Malformed'):
            node.check_position(0.)

    def test_arm_and_head_execute_without_approval_flags(self):
        for joint, distance in (('right_j3', .1), ('head_pan', .1), ('right_j4', .001)):
            clock = [0.]
            args = parse_args(['--joint', joint, '--distance', str(distance), '--time', '.6'])
            node = SimulatedMover(args, clock)
            def spin(_node, timeout_sec):
                clock[0] += max(timeout_sec, .001)
                node.refresh_feedback()
            with patch('sawyer_ros2_control.move_joint.time.monotonic', side_effect=lambda: clock[0]), \
                    patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                    patch('sawyer_ros2_control.move_joint.rclpy.spin_once', side_effect=spin), \
                    contextlib.redirect_stdout(io.StringIO()):
                node.run()
            self.assertGreater(len(node.messages), 40)
            values = []
            for msg in node.messages:
                if joint == 'head_pan':
                    self.assertEqual(msg.pan_mode, HeadPanCommand.SET_ACTIVE_MODE)
                    values.append(msg.target)
                else:
                    self.assertEqual(msg.mode, JointCommand.POSITION_MODE)
                    self.assertEqual(list(msg.names), [joint])
                    self.assertEqual(list(msg.velocity), [])
                    self.assertEqual(list(msg.acceleration), [])
                    self.assertEqual(list(msg.effort), [])
                    values.append(msg.position[0])
            self.assertAlmostEqual(values[-1], distance, places=7)
            self.assertTrue(all(a <= b+1e-10 for a, b in zip(values, values[1:])))

    def test_dry_run_and_invalid_target_do_not_publish(self):
        for options in (['--distance', '.1', '--dry-run'], ['--position', '2']):
            args = parse_args(['--joint', 'right_j0', '--time', '2'] + options)
            node = SimulatedMover(args, [0.])
            with patch('sawyer_ros2_control.move_joint.time.monotonic', return_value=0.), \
                    patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                    contextlib.redirect_stdout(io.StringIO()):
                if args.dry_run:
                    node.run()
                else:
                    with self.assertRaises(ValueError):
                        node.run()
            self.assertIsNone(node.pub)
            self.assertEqual(node.messages, [])

    def test_float32_head_target_and_new_live_limit(self):
        args = parse_args(['--joint', 'head_pan', '--position', '.1', '--time', '2'])
        node = SimulatedMover(args, [0.])
        node.bounds = (-1., .1)
        with self.assertRaises(ValueError):
            node.wire_position(.1)  # Float32 rounds this value above the exact limit.
        args = parse_args(['--joint', 'right_j0', '--distance', '.1', '--time', '2'])
        node = SimulatedMover(args, [0.])
        clock = [0.]
        at_limit_change = []
        def spin(_node, timeout_sec):
            clock[0] += max(timeout_sec, .001)
            node.refresh_feedback()
            if clock[0] > .3:
                if not at_limit_change:
                    at_limit_change.append(len(node.messages))
                node.bounds = (-1., .001)
        with patch('sawyer_ros2_control.move_joint.time.monotonic', side_effect=lambda: clock[0]), \
                patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                patch('sawyer_ros2_control.move_joint.rclpy.spin_once', side_effect=spin), \
                contextlib.redirect_stdout(io.StringIO()), self.assertRaises(ValueError):
            node.run()
        self.assertEqual(len(node.messages), at_limit_change[0])

    def test_slow_controller_reaches_absolute_zero_once(self):
        clock = [0.]
        args = parse_args(['--joint', 'right_j0', '--position', '0', '--time', '.6',
                           '--settle-timeout', '15', '--tolerance', '.002'])
        node = SimulatedMover(args, clock)
        node.position = .3
        commanded = [.3]
        def capture(msg):
            node.messages.append(msg)
            commanded[0] = msg.position[0]
        def spin(_node, timeout_sec):
            dt = max(timeout_sec, .001)
            clock[0] += dt
            error = commanded[0] - node.position
            node.position += max(-.05*dt, min(.05*dt, error))
            node.refresh_feedback()
        node.capture = capture
        with patch('sawyer_ros2_control.move_joint.time.monotonic', side_effect=lambda: clock[0]), \
                patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                patch('sawyer_ros2_control.move_joint.rclpy.spin_once', side_effect=spin), \
                contextlib.redirect_stdout(io.StringIO()):
            node.run()
            self.assertLessEqual(abs(node.position), args.tolerance)
            self.assertGreater(clock[0], 5.)  # A 0.6 s reference cannot finish this slow move.
            self.assertEqual(node.settings, [1.])
            count = len(node.messages)
            node.run()  # Same absolute zero, not another relative displacement.
            self.assertEqual(len(node.messages), count)
            self.assertEqual(node.settings, [1.])
        final = [msg.position[0] for msg in node.messages[-100:]]
        self.assertTrue(all(value == 0. for value in final))

    def test_operator_residual_finishes_on_time_and_repeating_zero_sends_nothing(self):
        clock = [0.]
        args = parse_args(['--joint', 'right_j1', '--position', '0', '--time', '5'])
        node = SimulatedMover(args, clock)
        node.position = -.0857783203125
        residual = .0042490234375  # Operator's measured residual, not an ideal actuator.
        def capture(msg):
            node.messages.append(msg)
            node.position = msg.position[0] - residual
        def spin(_node, timeout_sec):
            clock[0] += max(timeout_sec, .001)
            node.refresh_feedback()
        node.capture = capture
        output = io.StringIO()
        with patch('sawyer_ros2_control.move_joint.time.monotonic', side_effect=lambda: clock[0]), \
                patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                patch('sawyer_ros2_control.move_joint.rclpy.spin_once', side_effect=spin), \
                contextlib.redirect_stdout(output):
            node.run()
            count = len(node.messages)
            self.assertGreater(count, 450)
            self.assertEqual(node.messages[-1].position[0], 0.)
            # Allow one publication tick over the trajectory, plus startup settings.
            self.assertLess(clock[0], 5.25)
            node.run()
            self.assertEqual(len(node.messages), count)
        self.assertIn('"status": "reached"', output.getvalue())
        self.assertIn('"status": "already_at_target"', output.getvalue())
        self.assertIn('measured', output.getvalue())
        self.assertNotIn('not_reached', output.getvalue())

    def test_default_deadline_and_strict_tolerance_still_report_missed_targets(self):
        for residual, tolerance in ((.1, '.008726646'), (.0042490234375, '.002')):
            clock = [0.]
            args = parse_args(['--joint', 'right_j1', '--position', '0', '--time', '1',
                               '--tolerance', tolerance, '--settle-timeout', '0'])
            node = SimulatedMover(args, clock)
            node.position = -.2
            def capture(msg):
                node.messages.append(msg)
                node.position = msg.position[0] - residual
            def spin(_node, timeout_sec):
                clock[0] += max(timeout_sec, .001)
                node.refresh_feedback()
            node.capture = capture
            with patch('sawyer_ros2_control.move_joint.time.monotonic', side_effect=lambda: clock[0]), \
                    patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                    patch('sawyer_ros2_control.move_joint.rclpy.spin_once', side_effect=spin), \
                    contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(RuntimeError, 'not reached'):
                node.run()
            self.assertLess(clock[0], 1.25)

    def test_feedback_joint_names_and_out_of_order_samples(self):
        args = parse_args(['--joint', 'right_j0', '--position', '0', '--time', '1'])
        node = SimulatedMover(args, [0.])
        msg = JointState(name=['right_j6', 'right_j0'], position=[-.9, .5])
        msg.header.stamp = Time(sec=3)
        node.on_position(msg)
        self.assertEqual(node.position, .5)
        sequence = node.position_sequence
        msg.header.stamp = Time(sec=2)
        msg.position = [-.8, .8]
        node.on_position(msg)
        self.assertEqual(node.position, .5)
        self.assertEqual(node.position_sequence, sequence)
        msg.header.stamp = Time(sec=4)
        node.on_position(msg)
        self.assertEqual(node.position, .8)

    def test_goal_timeout_is_a_failure_and_stale_feedback_cannot_succeed(self):
        clock = [0.]
        args = parse_args(['--joint', 'right_j0', '--position', '0', '--time', '.3', '--settle-timeout', '.3'])
        node = SimulatedMover(args, clock)
        node.position = .1
        node.capture = lambda msg: node.messages.append(msg)  # Blocked actuator, fresh feedback.
        def spin(_node, timeout_sec):
            clock[0] += max(timeout_sec, .001)
            node.refresh_feedback()
        output = io.StringIO()
        with patch('sawyer_ros2_control.move_joint.time.monotonic', side_effect=lambda: clock[0]), \
                patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                patch('sawyer_ros2_control.move_joint.rclpy.spin_once', side_effect=spin), \
                contextlib.redirect_stdout(output), self.assertRaisesRegex(RuntimeError, 'not reached'):
            node.run()
        self.assertIn('"status": "not_reached"', output.getvalue())
        node.position = 0.
        clock[0] += 2.
        with patch('sawyer_ros2_control.move_joint.time.monotonic', side_effect=lambda: clock[0]), \
                self.assertRaisesRegex(RuntimeError, 'feedback stopped'):
            node.measured_position()

    def test_minimum_time_uses_exact_quintic_extrema(self):
        self.assertAlmostEqual(minimum_duration(.4, .5), 1.5)
        self.assertAlmostEqual(minimum_duration(-.4, .5), 1.5)
        acceleration_time = math.sqrt((10/math.sqrt(3))*.4/.2)
        self.assertAlmostEqual(minimum_duration(.4, 10., .2), acceleration_time)
        self.assertEqual(minimum_duration(0., 1., 1.), 0.)
        for velocity, acceleration in ((0., None), (math.nan, 1.), (1., -1.)):
            with self.assertRaises(ValueError):
                minimum_duration(.1, velocity, acceleration)

    def test_too_short_absolute_and_relative_plans_send_nothing(self):
        for destination, value in (('--position', '.1'), ('--distance', '.3')):
            args = parse_args(['--joint', 'right_j0', destination, value, '--time', '.01'])
            node = SimulatedMover(args, [0.])
            node.position = -.2
            with patch('sawyer_ros2_control.move_joint.time.monotonic', return_value=0.), \
                    patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                    self.assertRaisesRegex(ValueError, 'minimum quintic time.*No commands sent'):
                node.run()
            self.assertAlmostEqual(node.minimum_time, minimum_duration(.3, 2., 10.))
            self.assertIsNone(node.pub)
            self.assertEqual(node.messages, [])
            self.assertEqual(node.settings, [])
        args = parse_args(['--joint', 'right_j0', '--distance', '.3', '--time', '1'])
        node = SimulatedMover(args, [0.])
        node.args.duration = minimum_duration(.3, 2., 10.)
        node.check_duration(0., .3)  # Exact minimum is valid, within floating-point precision.

    def test_minimum_time_without_acceleration_data(self):
        args = parse_args(['--joint', 'head_pan', '--distance', '.1', '--time', '1'])
        node = SimulatedMover(args, [0.])
        node.on_limits(JointLimits(joint_names=['head_pan'], position_lower=[-1.], position_upper=[1.],
                                  velocity=[1.]))
        self.assertIsNone(node.acceleration_limit)
        node.check_duration(0., .1)
        self.assertAlmostEqual(node.minimum_time, .1875, places=7)

    def test_unmoving_passive_head_reports_controller_mode(self):
        clock = [0.]
        args = parse_args(['--joint', 'head_pan', '--position', '0', '--time', '.6', '--settle-timeout', '.3'])
        node = SimulatedMover(args, clock)
        node.position = -.3
        node.capture = lambda msg: node.messages.append(msg)
        def spin(_node, timeout_sec):
            clock[0] += max(timeout_sec, .001)
            node.refresh_feedback()
            node.on_head_state(HeadState(pan=-.3, pan_mode=HeadState.PASSIVE_MODE))
        output = io.StringIO()
        with patch('sawyer_ros2_control.move_joint.time.monotonic', side_effect=lambda: clock[0]), \
                patch('sawyer_ros2_control.move_joint.rclpy.ok', return_value=True), \
                patch('sawyer_ros2_control.move_joint.rclpy.spin_once', side_effect=spin), \
                contextlib.redirect_stdout(output), self.assertRaisesRegex(RuntimeError, 'not reached'):
            node.run()
        self.assertIn('requested ACTIVE_MODE (1) was not confirmed', output.getvalue())
        self.assertIn('"pan_mode": 0', output.getvalue())
        self.assertIn('"fresh": true', output.getvalue())
        self.assertTrue(all(msg.pan_mode == HeadPanCommand.SET_ACTIVE_MODE for msg in node.messages))


if __name__ == '__main__':
    unittest.main()
