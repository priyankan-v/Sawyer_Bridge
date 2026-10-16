"""Control Sawyer's electric end-effector gripper through the selective bridge."""
import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from intera_core_msgs.msg import IOComponentCommand, IODeviceStatus


# Travel limits from the bundled Intera Gripper interface, in metres.
MAX_POSITION = 0.041667
BASE = '/io/end_effector/right_gripper'


def decode_signals(message):
    """Decode Intera's scalar JSON arrays without retaining old signal values."""
    signals = {}
    for signal in message.signals:
        data = json.loads(signal.data)
        formatting = json.loads(signal.format)
        if not isinstance(data, list) or len(data) != 1:
            raise ValueError(f'{signal.name}: expected one scalar value')
        signals[signal.name] = dict(value=data[0], type=formatting['type'],
                                    status=signal.status.tag)
    return signals


class Gripper(Node):
    def __init__(self, args):
        super().__init__('sawyer_gripper')
        self.args = args
        self.message = None
        self.signals = {}
        self.error = None
        self.publisher = None
        self.create_subscription(IODeviceStatus, BASE + '/state', self.receive,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def receive(self, message):
        self.message = message
        try:
            self.signals = decode_signals(message)
            self.error = None
        except (ValueError, KeyError, TypeError) as error:
            self.signals = {}
            self.error = f'Invalid gripper feedback: {error}'

    def check_feedback(self):
        # State is latched and published on change, so silence is normal while idle.
        # Command timestamps and bounded acknowledgement/completion waits below
        # establish whether a new operation has been processed by the device.
        if self.message is None or self.count_publishers(BASE + '/state') == 0:
            raise RuntimeError('Gripper feedback is missing or its publisher is disconnected; check the bridge')
        if self.error:
            raise RuntimeError(self.error)
        if self.message.device.name != 'right_gripper':
            raise RuntimeError('Unexpected device on the gripper state topic')

    def value(self, name, kind):
        signal = self.signals.get(name)
        if signal is None or signal['status'] != 'ready':
            raise RuntimeError(f'Gripper signal {name} is missing or unavailable')
        value = signal['value']
        if kind is bool:
            valid = signal['type'] == 'bool' and type(value) is bool
        else:
            valid = signal['type'] == 'float' and type(value) in (int, float) and math.isfinite(value)
        if not valid:
            raise RuntimeError(f'Invalid gripper signal {name}')
        return value

    def check_ready(self):
        self.check_feedback()
        if self.args.operation == 'stop':
            return
        if self.args.operation != 'calibrate' and not self.value('is_calibrated', bool):
            raise RuntimeError('Gripper is not calibrated; run gripper calibrate explicitly')
        self.check_running()
        # The real electric gripper reports unready until calibrated.
        allowed = ('ready', 'unready') if self.args.operation == 'calibrate' else ('ready',)
        if self.message.device.status.tag not in allowed:
            raise RuntimeError('Gripper is not ready')

    def check_running(self):
        self.check_feedback()
        if self.args.operation == 'stop':
            return
        allowed = ('ready', 'busy', 'unready') if self.args.operation == 'calibrate' else ('ready', 'busy')
        if self.message.device.status.tag not in allowed or self.value('has_error', bool):
            raise RuntimeError('Gripper is not ready or reports an error')

    def wait_until(self, predicate, description, check=None):
        deadline = time.monotonic() + self.args.timeout
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if check:
                check()
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(f'Timed out waiting for {description}')
        raise RuntimeError('ROS context shut down')

    def command(self, values):
        signals = {}
        for name, value in values.items():
            kind = bool if type(value) is bool else float
            self.value(name, kind)  # Confirm the signal exists with the expected type.
            signals[name] = dict(format=dict(type=self.signals[name]['type']), data=[value])
        return IOComponentCommand(op='set', args=json.dumps(dict(signals=signals), allow_nan=False))

    def send(self, message):
        self.publisher = self.create_publisher(IOComponentCommand, BASE + '/command', 1)
        self.wait_until(lambda: self.publisher.get_subscription_count() > 0,
                        'bridge command subscription; restart with gripper_commands:=true', self.check_ready)
        message.time = self.get_clock().now().to_msg()
        next_publish = 0.

        def acknowledged():
            nonlocal next_publish
            if message.time in self.message.commands:
                return True
            if time.monotonic() >= next_publish:
                self.publisher.publish(message)
                next_publish = time.monotonic() + 0.1
            return False

        self.wait_until(acknowledged, 'gripper command acknowledgement', self.check_running)

    def position_tolerance(self):
        if self.args.tolerance is not None:
            return self.args.tolerance
        # The controller may stop anywhere inside its configured dead zone.
        # A tighter client default would incorrectly time out on successful moves.
        if 'dead_zone_m' not in self.signals:
            return 0.001
        tolerance = self.value('dead_zone_m', float)
        if not 0 <= tolerance < MAX_POSITION:
            raise RuntimeError('Invalid gripper dead_zone_m: expected less than full travel and nonnegative')
        return tolerance

    def completed(self, target):
        operation = self.args.operation
        if operation == 'calibrate':
            return self.value('is_calibrated', bool)
        if operation == 'stop':
            return not self.value('is_moving', bool)
        if self.value('is_moving', bool):
            return False
        if abs(self.value('position_response_m', float) - target) <= self.position_tolerance():
            return True
        # Closing against an object is a successful grasp even above zero aperture.
        return operation == 'close' and self.value('is_gripping', bool)

    def run(self):
        self.wait_until(lambda: self.message is not None,
                        f'gripper state on {BASE}/state; check the bridge forwards retained state '
                        '(durability: transient_local) and ROS_DOMAIN_ID matches')
        self.check_feedback()
        if self.args.operation == 'status':
            return self.report('status')
        self.check_ready()
        target = None
        if self.args.operation == 'calibrate':
            values = dict(calibrate=True)
        elif self.args.operation == 'stop':
            values = dict(go=False)
        else:
            target = {'open': MAX_POSITION, 'close': 0.}.get(self.args.operation, self.args.position)
            self.position_tolerance()  # Validate the device dead zone before publishing.
            values = dict(position_m=target, go=True)
        message = self.command(values)
        if self.args.dry_run:
            return dict(status='dry_run', topic=BASE + '/command', op=message.op,
                        args=json.loads(message.args))
        self.send(message)
        try:
            self.wait_until(lambda: self.completed(target), 'gripper operation to finish', self.check_running)
        except RuntimeError as error:
            if target is not None and str(error).startswith('Timed out waiting for'):
                actual = self.value('position_response_m', float)
                raise RuntimeError(f'{error}: target={target:.6f} m, measured={actual:.6f} m, '
                                   f'error={abs(actual - target):.6f} m, '
                                   f'tolerance={self.position_tolerance():.6f} m, '
                                   f'is_moving={self.value("is_moving", bool)}') from error
            raise
        report = self.report('completed')
        if target is not None:
            report.update(target_m=target, tolerance_m=self.position_tolerance(),
                          position_error_m=abs(self.value('position_response_m', float) - target))
        return report

    def report(self, status):
        return dict(status=status, device=self.message.device.name,
                    device_status=self.message.device.status.tag,
                    signals={name: signal['value'] for name, signal in self.signals.items()})


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', nargs='?', default='status',
                        choices=('status', 'open', 'close', 'position', 'calibrate', 'stop'))
    parser.add_argument('--position', type=float, help='Opening in metres (0 to 0.041667)')
    parser.add_argument('--dry-run', action='store_true', help='Check feedback and print command without publishing')
    parser.add_argument('--timeout', type=float, default=10., help='Timeout per wait, in seconds')
    parser.add_argument('--tolerance', type=float,
                        help='Position tolerance in metres (default: device dead_zone_m, or 0.001 if absent)')
    args = parser.parse_args(argv)
    if (args.operation == 'position') != (args.position is not None):
        parser.error('Use position --position METRES to specify an opening')
    if args.position is not None and (not math.isfinite(args.position) or not 0 <= args.position <= MAX_POSITION):
        parser.error('Position must be finite and within 0 to 0.041667 metres')
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('timeout must be positive and finite')
    if args.tolerance is not None and (not math.isfinite(args.tolerance) or not 0 < args.tolerance < MAX_POSITION):
        parser.error('Position tolerance must be positive, finite and less than 0.041667 metres')
    return args


def main(argv=None):
    args = parse_args(argv)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Gripper(args)
        print(json.dumps(node.run(), indent=2, allow_nan=False))
        return 0
    except KeyboardInterrupt:
        print('Interrupted; use gripper stop to stop an outstanding operation.', file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, TypeError) as error:
        print(f'Gripper: {error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
