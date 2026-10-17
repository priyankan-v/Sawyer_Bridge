"""Preview or explicitly execute a freshly planned taught-pose demonstration."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from intera_core_msgs.msg import IODeviceStatus
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

from .command import Controller
from .gripper import BASE as GRIPPER_BASE, Gripper, decode_signals, parse_args as gripper_args
from .safety import JOINTS, Limit, finite
from .task_trajectory import retime, sample, waypoints

ORDER = ('open_before_pick', 'pre_pick', 'pick', 'grasp', 'lift', 'pre_place', 'place', 'release', 'retreat')


class ArmExecutor(Controller):
    def __init__(self, args, execute=False, expect_grasp=False, head_pan=None):
        self.head_pan, self.head_received, self.gripping = None, None, None
        self.gripper_fault = False
        self.expected_head = head_pan
        self.expect_grasp = expect_grasp
        settings = SimpleNamespace(execute=execute, command_timeout=.2, feedback_timeout=1.,
            max_velocity=args.max_velocity, max_acceleration=args.max_acceleration,
            wait=5., operation='configuration')
        super().__init__(settings)
        self.create_subscription(IODeviceStatus, GRIPPER_BASE + '/state', self.receive_gripper,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_joints(self, message):
        super().on_joints(message)
        if 'head_pan' in message.name and len(message.name) == len(message.position):
            try:
                self.head_pan = finite(message.position[list(message.name).index('head_pan')])
                self.head_received = time.monotonic()
            except ValueError:
                self.fault = 'Invalid head feedback'

    def receive_gripper(self, message):
        try:
            signals = decode_signals(message)
            signal = signals['is_gripping']
            self.gripping = signal['value'] if signal['status'] == 'ready' and type(signal['value']) is bool else None
            self.gripper_fault = signals['has_error']['value'] is not False or signals['has_error']['status'] != 'ready'
        except (KeyError, ValueError, TypeError):
            self.gripping = None
            self.gripper_fault = True

    def wait_for_feedback(self):
        super().wait_for_feedback()
        deadline = time.monotonic() + 5.
        while self.head_received is None or (self.expect_grasp and self.gripping is None):
            rclpy.spin_once(self, timeout_sec=.02)
            if time.monotonic() >= deadline:
                raise RuntimeError('Head/gripper feedback required before arm execution')

    def check(self):
        limits = super().check()
        if self.head_received is None or time.monotonic() - self.head_received > 1.:
            raise RuntimeError('Missing or stale head feedback for collision-checked execution')
        if self.expected_head is not None and abs(self.head_pan - self.expected_head) > .01:
            raise RuntimeError('Head moved relative to the collision-checked state; replan')
        if self.gripper_fault:
            raise RuntimeError('Gripper reports an error or invalid feedback')
        if self.expect_grasp and self.gripping is not True:
            raise RuntimeError('Grasp was lost or gripper feedback is unavailable; stopping transport')
        return limits

    def execute(self, points, rate=100.):
        if self.get_parameter('use_sim_time').value:
            raise RuntimeError('Hardware command execution rejects simulated time')
        self.wait_for_feedback()
        limits = self.check()
        if max(abs(self.positions[j] - points[0][j]) for j in JOINTS) > .002:
            raise RuntimeError('Arm moved from the validated trajectory start; replan')
        if max(abs(v) for v in self.velocities.values()) > .02:
            raise RuntimeError('Arm must be stationary before executing a stage')
        segments = retime(points, limits)
        if not segments:
            return dict(samples=0, duration=0.)
        deadline = time.monotonic() + 5.
        while not all(pub.get_subscription_count() for pub in (self.pub, self.speed_pub, self.timeout_pub)):
            rclpy.spin_once(self, timeout_sec=.02)
            self.check()
            if time.monotonic() >= deadline:
                raise RuntimeError('Arm command, speed and timeout bridge subscriptions are required')
        self.speed_pub.publish(Float64(data=.05))
        self.timeout_pub.publish(Float64(data=.2))
        settings_deadline = time.monotonic() + .3
        while time.monotonic() < settings_deadline:
            rclpy.spin_once(self, timeout_sec=.01)
            self.check()
        if max(abs(self.positions[j] - points[0][j]) for j in JOINTS) > .002:
            raise RuntimeError('Arm moved while controller settings were applied')
        duration = sum(segment.duration for segment in segments)
        begin = previous = next_tick = time.monotonic()
        count = 0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=max(0., min(.005, next_tick - time.monotonic())))
            now = time.monotonic()
            if now < next_tick:
                continue
            if now - previous > .2:
                raise RuntimeError('Arm command scheduling deadline missed')
            if self.check() != limits:
                raise RuntimeError('Controller limits changed during execution')
            target = sample(segments, now - begin)
            if max(abs(self.positions[j] - target[j]) for j in JOINTS) > .05:
                raise RuntimeError('Arm tracking error exceeds 0.05 rad')
            self.publish_positions(target)
            count += 1
            previous, next_tick = now, now + 1. / rate
            if now - begin >= duration:
                error = max(abs(self.positions[j] - points[-1][j]) for j in JOINTS)
                if error <= .008726646 and max(abs(v) for v in self.velocities.values()) < .01:
                    return dict(samples=count, duration=now - begin, final_error_rad=error)
                if now - begin > duration + 5.:
                    raise RuntimeError(f'Arm stage failed to settle: {error:.6f} rad')
        raise RuntimeError('ROS context shut down during execution')


def read_plan(args):
    plan = json.loads(args.plan.read_text())
    if plan.get('status') != 'planned' or tuple(stage['name'] for stage in plan['stages']) != ORDER:
        raise ValueError('Expected a complete successful nine-stage pick/place plan')
    manifest = json.loads((args.model / 'manifest.json').read_text())
    if plan.get('model_source_sha256') != manifest['source_sha256']:
        raise ValueError('Plan belongs to a different robot model; replan')
    if args.execute:
        if plan.get('state_source') != 'live_feedback':
            raise ValueError('Offline snapshot plans cannot execute; plan from live feedback')
        age = time.time() - finite(plan['planned_at_unix'])
        if not 0 <= age <= 120.:
            raise ValueError('Execution requires a plan less than 120 seconds old; replan')
    limits = {}
    model = ET.parse(args.model / 'sawyer.urdf').getroot()
    caps = json.loads((args.model / 'planning_limits.json').read_text())['joint_limits']
    for name in JOINTS:
        bound = model.find(f"joint[@name='{name}']/limit")
        limits[name] = Limit(float(bound.get('lower')), float(bound.get('upper')),
            min(args.max_velocity, caps[name]['max_velocity']), min(args.max_acceleration, caps[name]['max_acceleration']))
    previous = {name: finite(plan['start_state'][name]) for name in JOINTS}
    for index, stage in enumerate(plan['stages']):
        expected_operation = {0: 'open', 3: 'close', 7: 'open'}.get(index)
        if expected_operation is not None:
            if stage['kind'] != 'gripper' or stage.get('operation') != expected_operation:
                raise ValueError('Incorrect gripper operation in the task sequence')
            if index == 3 and stage.get('require_grasp') is not True:
                raise ValueError('Task must verify the grasp before transporting')
        elif stage['kind'] != 'arm':
            raise ValueError('Expected an arm trajectory in this task stage')
        if stage['kind'] == 'arm':
            points = waypoints(stage['trajectory'])
            if max(abs(previous[name] - points[0][name]) for name in JOINTS) > .002:
                raise ValueError('Discontinuous arm stages in the plan')
            segments = retime(points, limits)
            stage['waypoints'] = points
            stage['execution_duration_s'] = sum(segment.duration for segment in segments)
            previous = points[-1]
        elif stage['kind'] != 'gripper' or stage.get('operation') not in ('open', 'close'):
            raise ValueError('Unsupported task stage')
    return plan


def check_path(args, points, aperture):
    root = Path(os.environ['SAWYER_ROOT'])
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', dir=root / '.runtime') as stream:
        json.dump(dict(waypoints=points, expected_gripper_joint=aperture / 2.), stream, allow_nan=False)
        stream.flush()
        result = subprocess.run([str(root / 'scripts/env.sh'), 'system-ros2', 'python3',
            str(root / 'planning/validate_execution_path.py'), stream.name, '--model', str(args.model)],
            capture_output=True, text=True, timeout=120.)
    if result.returncode != 0:
        raise RuntimeError(result.stdout + result.stderr)
    return json.loads(result.stdout)


def gripper_operation(operation):
    node = Gripper(gripper_args([operation]))
    try:
        return node.run()
    finally:
        node.destroy_node()


def run(args, plan):
    if not args.execute:
        return dict(status='preview', execute=False, collision_scope=plan['collision_scope'],
            stages=[dict(name=stage['name'], kind=stage['kind'], duration_s=stage.get('execution_duration_s', 0.)) for stage in plan['stages']])
    arm = ArmExecutor(args, head_pan=plan['start_state']['head_pan'])
    try:
        arm.wait_for_feedback()
        arm.check()
        if max(abs(arm.positions[j] - plan['start_state'][j]) for j in JOINTS) > .008726646:
            raise RuntimeError('Robot no longer matches the planned start; replan before execution')
    finally:
        arm.destroy_node()
    initial = gripper_operation('status')['signals']
    if initial.get('has_error') is not False or initial.get('is_calibrated') is not True:
        raise RuntimeError('Gripper must be calibrated and error-free before the task')
    aperture = finite(initial['position_response_m'])
    first = next(stage for stage in plan['stages'] if stage['kind'] == 'arm')
    check_path(args, first['waypoints'], aperture)  # Refuse before the first gripper actuation if the start/path is invalid.
    holding, completed = False, []
    for stage in plan['stages']:
        print(f'Pick/place stage: {stage["name"]}', flush=True)
        if stage['kind'] == 'gripper':
            result = gripper_operation(stage['operation'])
            aperture = finite(result['signals']['position_response_m'])
            if stage.get('require_grasp') and result['signals'].get('is_gripping') is not True:
                raise RuntimeError('Gripper did not detect a grasp; transport was not started')
            holding = stage['operation'] == 'close'
        else:
            checked = check_path(args, stage['waypoints'], aperture)
            arm = ArmExecutor(args, execute=True, expect_grasp=holding, head_pan=checked['head_pan'])
            try:
                result = arm.execute(checked['waypoints'])
            finally:
                arm.safe_hold()
                arm.destroy_node()
        completed.append(dict(name=stage['name'], result=result))
    return dict(status='completed', execute=True, stages=completed)


def main(argv=None):
    root = Path(os.environ.get('SAWYER_ROOT', Path.cwd()))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, default=root / '.runtime/pick_place_plan.json')
    parser.add_argument('--model', type=Path, default=root / '.runtime/moveit_model')
    parser.add_argument('--execute', action='store_true', help='Execute the freshly planned sequence on the robot')
    parser.add_argument('--max-velocity', type=float, default=.015)
    parser.add_argument('--max-acceleration', type=float, default=.03)
    args = parser.parse_args(argv)
    for name, maximum in (('max_velocity', .05), ('max_acceleration', .1)):
        value = getattr(args, name)
        if not math.isfinite(value) or not 0 < value <= maximum:
            parser.error(f'{name} must be positive, finite and at most {maximum}')
    try:
        plan = read_plan(args)
    except (KeyError, ValueError, TypeError, OSError) as error:
        print(f'Pick/place: {error}', file=sys.stderr)
        return 1
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        print(json.dumps(run(args, plan), indent=2, allow_nan=False))
        return 0
    except KeyboardInterrupt:
        print('Pick/place interrupted; no automatic gripper opening was issued.', file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, KeyError, TypeError, OSError, subprocess.SubprocessError) as error:
        print(f'Pick/place aborted: {error}; gripper opening is not performed during failure handling.', file=sys.stderr)
        return 1
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
