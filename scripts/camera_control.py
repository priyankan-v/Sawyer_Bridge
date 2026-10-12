#!/usr/bin/env python3
"""Noetic camera stream control using the SDK's IO protocol; no motion commands."""
import argparse
import json
import math
from pathlib import Path
import threading
import time

import rospy
from intera_core_msgs.msg import IOComponentCommand, IODeviceConfiguration, IODeviceStatus
from sensor_msgs.msg import Image

CAMERAS = {'head': 'head_camera', 'hand': 'right_hand_camera'}
IMAGE_TOPICS = {'head': '/io/internal_camera/head_camera/image_rect_color',
                'hand': '/io/internal_camera/right_hand_camera/image_rect'}


def streaming_value(message):
    for signal in message.signals:
        if signal.name == 'camera_streaming':
            data = json.loads(signal.data)
            if json.loads(signal.format).get('type') != 'bool' or len(data) != 1 or type(data[0]) is not bool:
                raise ValueError('Invalid camera_streaming boolean signal')
            return data[0]
    raise ValueError('Missing camera_streaming signal')


class CameraControl:
    def __init__(self, timeout=5.):
        self.timeout = timeout
        self.condition = threading.Condition()
        self.states, self.publishers = {}, {}
        self.subscriptions = [
            rospy.Subscriber('/io/internal_camera/' + name + '/state', IODeviceStatus,
                             lambda message, key=key: self.receive(key, message), queue_size=1)
            for key, name in CAMERAS.items()
        ]
        self.wait(lambda: len(self.states) == len(CAMERAS), 'Camera status unavailable')

    def receive(self, key, message):
        with self.condition:
            self.states[key] = message
            self.condition.notify_all()

    def wait(self, predicate, description):
        deadline = time.monotonic() + self.timeout
        with self.condition:
            while not predicate():
                if rospy.is_shutdown():
                    raise RuntimeError('ROS shut down during camera operation')
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(description)
                self.condition.wait(min(.1, remaining))

    def status(self):
        with self.condition:
            return {key: streaming_value(message) for key, message in self.states.items()}

    def set_streaming(self, key, enabled):
        if self.status()[key] == enabled:
            return  # Do not send redundant commands.
        if key not in self.publishers:
            self.publishers[key] = rospy.Publisher(
                '/io/internal_camera/' + CAMERAS[key] + '/command', IOComponentCommand, queue_size=1)
        publisher = self.publishers[key]
        self.wait(lambda: publisher.get_num_connections() > 0, f'{key}: no camera command subscriber')
        command = IOComponentCommand(time=rospy.Time.now(), op='set', args=json.dumps({
            'signals': {'camera_streaming': {'format': {'type': 'bool'}, 'data': [enabled]}}}))
        deadline = time.monotonic() + self.timeout
        while not rospy.is_shutdown():
            publisher.publish(command)
            with self.condition:
                message = self.states[key]
                if command.time in message.commands and streaming_value(message) == enabled:
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.condition.wait(min(.1, remaining))
        raise RuntimeError(f'{key}: no acknowledged camera_streaming={enabled} state within {self.timeout:g} s')

    def start(self, selected):
        # Preserve the SDK's single-camera policy. Confirm the other is stopped
        # before enabling the selected source.
        for key in CAMERAS:
            if key != selected:
                self.set_streaming(key, False)
        self.set_streaming(selected, True)
        state = self.status()
        if not state[selected] or any(value for key, value in state.items() if key != selected):
            raise RuntimeError('Camera state changed concurrently; single-camera selection failed')

    def stop(self, selected):
        for key in CAMERAS if selected == 'all' else (selected,):
            self.set_streaming(key, False)

    def start_both(self):
        # Explicit opt-in to concurrent camera IO, verified on this controller.
        # The higher-level Cameras.start_streaming API enforces single selection.
        for key in CAMERAS:
            self.set_streaming(key, True)
        if not all(self.status().values()):
            raise RuntimeError('Controller did not keep both camera streams active')

    def info(self, seconds):
        details = rospy.get_param('/robot_config/camera_config')
        rows = {}
        for key, name in CAMERAS.items():
            config = rospy.wait_for_message('/io/internal_camera/' + name + '/config',
                                           IODeviceConfiguration, timeout=self.timeout)
            params = json.loads(config.commanded)['params']
            rows[key] = dict(camera=name, driver=details[name]['cameraType'],
                             configured_fps=params.get('fps'),
                             configured_width=params.get('width'), configured_height=params.get('height'),
                             topic=IMAGE_TOPICS[key], frames=0)
        image_lock = threading.Lock()
        times = {key: [] for key in CAMERAS}

        def received(key, message):
            with image_lock:
                times[key].append(time.monotonic())
                rows[key].update(frames=len(times[key]), width=message.width, height=message.height,
                                 encoding=message.encoding, step=message.step)

        subscriptions = [rospy.Subscriber(IMAGE_TOPICS[key], Image,
                            lambda msg, key=key: received(key, msg), queue_size=1, buff_size=2**23)
                         for key in CAMERAS]
        deadline = time.monotonic() + seconds
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                time.sleep(min(.05, max(0., deadline - time.monotonic())))
        finally:
            for subscription in subscriptions:
                subscription.unregister()
        with image_lock:
            state = self.status()
            for key, stamps in times.items():
                rows[key].update(streaming=state[key], measured_fps=(len(stamps)-1)/(stamps[-1]-stamps[0])
                                 if len(stamps) > 1 and stamps[-1] > stamps[0] else 0.)
            return dict(cameras=rows, sample_seconds=seconds,
                        maximum_fps='Not exposed by controller configuration',
                        maximum_resolution='Not exposed by controller configuration')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('start', 'start-both', 'stop', 'info'))
    parser.add_argument('camera', choices=('head', 'hand', 'all'), nargs='?', default='all')
    parser.add_argument('--timeout', type=float, default=5.)
    parser.add_argument('--sample-seconds', type=float, default=3.)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args(argv)
    if args.action == 'start' and args.camera == 'all':
        parser.error('Select head or hand; the SDK supports one active camera')
    if args.action == 'start-both' and args.camera != 'all':
        parser.error('start-both applies to both cameras')
    if not all(math.isfinite(value) and value > 0 for value in (args.timeout, args.sample_seconds)):
        parser.error('Timeout and sample duration must be positive and finite')
    try:
        rospy.init_node('sawyer_camera_control', anonymous=True)
        control = CameraControl(args.timeout)
        if args.action == 'start':
            control.start(args.camera)
        elif args.action == 'start-both':
            control.start_both()
        elif args.action == 'stop':
            control.stop(args.camera)
        result = control.info(args.sample_seconds) if args.action == 'info' else dict(
            action=args.action, selected=args.camera, streaming=control.status(), status='PASS')
        print(json.dumps(result, indent=2), flush=True)
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(result, indent=2) + '\n')
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(f'Camera control error: {error}', flush=True)
        return 1
    finally:
        if rospy.core.is_initialized():
            rospy.signal_shutdown('Camera command complete')


if __name__ == '__main__':
    raise SystemExit(main())
