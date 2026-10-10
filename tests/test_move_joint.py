import contextlib
import io
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/sawyer_ros2_control'))
from sawyer_ros2_control.safety import JOINTS, JointQuintic, Limit, single_joint_limit
from sawyer_ros2_control.move_joint import SingleJointController, parse_args
from intera_core_msgs.msg import JointLimits, HeadPanCommand, HeadState, JointCommand, RobotAssemblyState, CollisionDetectionState
from builtin_interfaces.msg import Time


class SimulatedController(SingleJointController):
    """Exercise actual publisher construction and trajectory loop without a ROS node or hardware."""
    def __init__(self, args, clock):
        self.args, self.clock = args, clock
        self.head = args.joint == 'head_pan'
        self.positions = dict.fromkeys(JOINTS, 0.)
        self.velocities = dict.fromkeys(JOINTS, 0.)
        self.head_position = self.head_velocity = 0.
        self.started_at, self.held_arm, self.fault, self.pub = None, None, None, None
        names = list(JOINTS) + ['head_pan']
        self.data = {'state': RobotAssemblyState(homed=True,ready=True,enabled=True),
                     'collision': CollisionDetectionState(),
                     'head': HeadState(pan=0.,pan_mode=HeadState.ACTIVE_MODE),
                     'limits': JointLimits(joint_names=names,position_lower=[-1.]*8,position_upper=[1.]*8,velocity=[1.]*8,accel=[1.]*8)}
        self.urdf = {j:dict(lower=-1.,upper=1.,velocity=1.) for j in names}
        self.received = {}
        self.messages = []
        self.refresh()

    def refresh(self):
        self.received = dict.fromkeys(('joints','state','collision','head','head_joint'),self.clock[0])

    def wait_for_feedback(self):
        self.refresh()

    def make_publishers(self):
        self.pub = NS(publish=self.capture)
        self.timeout_pub = self.speed_pub = NS(publish=lambda m:None)

    def get_parameter(self, name):
        return NS(value=False)

    def get_clock(self):
        return NS(now=lambda:NS(to_msg=lambda:Time(sec=123)))

    def capture(self, msg):
        self.messages.append((self.clock[0],msg))
        if self.head:
            self.head_position, self.head_velocity = float(msg.target), 0.
            self.data['head'].pan = msg.target
        else:
            self.positions.update(zip(msg.names,msg.position))
        self.refresh()


class MoveJointTests(unittest.TestCase):
    def test_argument_selection_and_approval(self):
        args = parse_args(['--joint','head_pan','--distance','-.01','--time','2'])
        self.assertEqual(args.distance,-.01)
        self.assertFalse(args.execute)
        for options in (['--joint','unknown','--distance','.01','--time','2'],
                        ['--joint','right_j0','--distance','.01','--position','0','--time','2'],
                        ['--joint','right_j0','--distance','nan','--time','2'],
                        ['--joint','right_j0','--distance','.01','--time','0'],
                        ['--joint','head_pan','--position','0','--time','2','--execute']):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                parse_args(options)

    def test_scalar_quintic_bounds_and_negative_distance(self):
        curve = JointQuintic(.1,.09,2,Limit(-1,1,.1,.2))
        for t,expected in ((0,.1),(2,.09)):
            q,v,a = curve.sample(t)
            self.assertAlmostEqual(q,expected)
            self.assertAlmostEqual(v,0)
            self.assertAlmostEqual(a,0)
        self.assertAlmostEqual(curve.sample(1)[0],.095)
        self.assertAlmostEqual(curve.peak_velocity,1.875*.01/2)
        self.assertAlmostEqual(curve.peak_acceleration,10/math.sqrt(3)*.01/4)
        for target,duration in ((2,2),(.2,2),(.09,.001)):
            with self.assertRaises(ValueError):
                JointQuintic(.1,target,duration,Limit(-1,1,.1,.2))

    def test_head_limit_respects_speed_ratio_and_live_acceleration(self):
        msg=JointLimits(joint_names=['head_pan'],position_lower=[-5.],position_upper=[.8],velocity=[1.8],accel=[.05])
        bound=single_joint_limit(msg,'head_pan',dict(lower=-5.0952,upper=.9064,velocity=1.8),.1,.2,.05)
        self.assertAlmostEqual(bound.velocity,.09)
        self.assertEqual(bound.acceleration,.05)
        self.assertEqual(bound.lower,-5.)

    def test_arm_and_head_execute_quintic_in_simulation(self):
        for joint in ('right_j3','head_pan'):
            clock=[0.]
            destination='--distance' if joint=='head_pan' else '--position'
            args=parse_args(['--joint',joint,destination,'.01','--time','.6','--execute','--operator-approved'])
            node=SimulatedController(args,clock)
            def spin(_node,timeout_sec):
                clock[0]+=max(timeout_sec,.001)
                node.refresh()
            with patch('sawyer_ros2_control.move_joint.time.monotonic',side_effect=lambda:clock[0]), \
                    patch('sawyer_ros2_control.command.time.monotonic',side_effect=lambda:clock[0]), \
                    patch('sawyer_ros2_control.move_joint.rclpy.ok',return_value=True), \
                    patch('sawyer_ros2_control.move_joint.rclpy.spin_once',side_effect=spin), \
                    contextlib.redirect_stdout(io.StringIO()):
                node.run()
            self.assertGreater(len(node.messages),40)
            values=[]
            for _,msg in node.messages:
                if joint=='head_pan':
                    self.assertIsInstance(msg,HeadPanCommand)
                    self.assertEqual(msg.pan_mode,HeadPanCommand.SET_ACTIVE_MODE)
                    self.assertAlmostEqual(msg.speed_ratio,.05)
                    values.append(msg.target)
                else:
                    self.assertIsInstance(msg,JointCommand)
                    self.assertEqual(msg.mode,JointCommand.POSITION_MODE)
                    self.assertEqual(list(msg.velocity),[])
                    self.assertEqual(list(msg.acceleration),[])
                    self.assertEqual(list(msg.effort),[])
                    for name,value in zip(msg.names,msg.position):
                        if name != joint:self.assertEqual(value,0.)
                    values.append(msg.position[msg.names.index(joint)])
            self.assertAlmostEqual(values[-1],.01,places=7)
            self.assertTrue(all(a<=b+1e-10 for a,b in zip(values,values[1:])))

    def test_head_faults_abort_and_dry_run_has_no_publisher(self):
        args=parse_args(['--joint','head_pan','--position','.01','--time','2'])
        node=SimulatedController(args,[0.])
        with patch('sawyer_ros2_control.move_joint.time.monotonic',return_value=0),contextlib.redirect_stdout(io.StringIO()):
            node.run()
        self.assertIsNone(node.pub)
        self.assertEqual(node.messages,[])
        for key in ('joints','state','collision'):
            node.received[key]=2.
        with patch('sawyer_ros2_control.move_joint.time.monotonic',return_value=2):
            with self.assertRaisesRegex(RuntimeError,'stale head'):
                node.check()
        node.refresh()
        node.data['head'].is_blocked=True
        with patch('sawyer_ros2_control.move_joint.time.monotonic',return_value=0):
            with self.assertRaisesRegex(RuntimeError,'blocked'):
                node.check()
        node.data['head'].is_blocked=False
        node.head_position=.1
        with patch('sawyer_ros2_control.move_joint.time.monotonic',return_value=0):
            with self.assertRaisesRegex(RuntimeError,'disagree'):
                node.check()


if __name__=='__main__':
    unittest.main()
