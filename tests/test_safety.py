import math
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/sawyer_ros2_control'))
from sawyer_ros2_control.safety import JOINTS, Limit, Quintic, finite, live_limits, ready, validate_positions


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.start = dict.fromkeys(JOINTS, 0.)
        self.target = dict(self.start, right_j0=0.01)
        self.limits = dict.fromkeys(JOINTS, Limit(-1, 1, 0.1, 0.2))

    def test_quintic_endpoints_and_extrema(self):
        trajectory = Quintic(self.start, self.target, 1, self.limits)
        for elapsed, position in ((0, self.start), (1, self.target)):
            q, dq, ddq = trajectory.sample(elapsed)
            for j in JOINTS:
                self.assertAlmostEqual(q[j], position[j])
                self.assertAlmostEqual(dq[j], 0)
                self.assertAlmostEqual(ddq[j], 0)
        maximum_velocity, maximum_accel = 0., 0.
        previous = -1
        for i in range(10001):
            q, dq, ddq = trajectory.sample(i / 10000)
            self.assertGreaterEqual(q['right_j0'], previous - 1e-12)
            previous = q['right_j0']
            maximum_velocity = max(maximum_velocity, abs(dq['right_j0']))
            maximum_accel = max(maximum_accel, abs(ddq['right_j0']))
        self.assertAlmostEqual(maximum_velocity, 0.01 * 1.875)
        self.assertAlmostEqual(maximum_accel, 0.01 * 10 / math.sqrt(3), places=7)

    def test_unsafe_inputs_rejected(self):
        for value in (float('nan'), float('inf'), -float('inf'), True, '1'):
            with self.assertRaises(ValueError):
                finite(value)
        for duration in (0, -1, float('nan'), 0.01):
            with self.assertRaises(ValueError):
                Quintic(self.start, self.target, duration, self.limits)
        for target in (dict(self.target, unknown=0), dict(self.target, right_j0=2),
                       dict(self.target, right_j0=0.03), dict(self.target, right_j0=float('nan'))):
            with self.assertRaises(ValueError):
                Quintic(self.start, target, 1, self.limits)
        limited = dict.fromkeys(JOINTS, Limit(-1, 1, 1, 0.001))
        with self.assertRaisesRegex(ValueError, 'acceleration'):
            Quintic(self.start, self.target, 1, limited)

    def test_live_limit_intersection_and_malformed_arrays(self):
        model = {j: dict(lower=-1, upper=1, velocity=.3) for j in JOINTS}
        msg = NS(joint_names=list(JOINTS), position_lower=[-.8]*7,
                 position_upper=[.9]*7, velocity=[.5]*7, accel=[.1]*7)
        limits = live_limits(msg, model)
        self.assertEqual(limits['right_j0'], Limit(-.8, .9, .1, .1))
        msg.accel = []
        with self.assertRaises(ValueError):
            live_limits(msg, model)
        msg.accel = [-1]*7
        with self.assertRaises(ValueError):
            live_limits(msg, model)

    def test_readiness_fails_closed(self):
        values = dict(homed=True, ready=True, enabled=True, stopped=False, error=False,
                      low_voltage=False, estop_button=0, estop_source=0)
        collision = NS(collision_state=False)
        self.assertTrue(ready(NS(**values), collision))
        for key in ('homed', 'ready', 'enabled', 'stopped', 'error', 'low_voltage', 'estop_button', 'estop_source'):
            modified = dict(values)
            modified[key] = not values[key] if isinstance(values[key], bool) else 1
            self.assertFalse(ready(NS(**modified), collision))
        self.assertFalse(ready(NS(**values), NS(collision_state=True)))
        self.assertFalse(ready(None, collision))

    def test_stale_and_collision_feedback_abort(self):
        from sawyer_ros2_control.command import Controller
        healthy = NS(homed=True, ready=True, enabled=True, stopped=False, error=False,
                     low_voltage=False, estop_button=0, estop_source=0)
        fake = NS(fault=None, args=NS(feedback_timeout=1), received=dict.fromkeys(('joints','state','collision'), time.monotonic()),
                  data={'state':healthy, 'collision':NS(collision_state=True)})
        with self.assertRaisesRegex(RuntimeError, 'collision'):
            Controller.check(fake)
        fake.received['joints'] -= 2
        with self.assertRaisesRegex(RuntimeError, 'stale joints'):
            Controller.check(fake)

    def test_partial_joint_states_do_not_refresh_other_joints(self):
        from sawyer_ros2_control.command import Controller
        fake = NS(positions={}, velocities={}, joint_received={}, data={}, received={}, fault=None)
        for joint in JOINTS:
            Controller.on_joints(fake, NS(name=[joint], position=[.01], velocity=[0.]))
        self.assertEqual(set(fake.positions),set(JOINTS))
        old_receipt = fake.received['joints']
        Controller.on_joints(fake, NS(name=['gripper'], position=[0.], velocity=[]))
        self.assertEqual(fake.received['joints'],old_receipt)
        fake.joint_received['right_j0'] = time.monotonic()-5
        Controller.on_joints(fake, NS(name=['right_j1'], position=[.02], velocity=[0.]))
        self.assertLess(fake.received['joints'],time.monotonic()-4)


if __name__ == '__main__':
    unittest.main()
