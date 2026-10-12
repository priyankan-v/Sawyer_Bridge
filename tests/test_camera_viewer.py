"""Image decoding and stream health checks; no ROS graph or robot connection."""
from array import array
from collections import deque
from types import SimpleNamespace
import threading
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np
from sensor_msgs.msg import Image
from sawyer_ros2_control.camera_viewer import CameraViewer, CameraWindow, image_to_rgb, ppm_bytes, main, parse_args


def image(width, height, encoding, data, step=None, bigendian=False):
    message = Image(width=width, height=height, encoding=encoding,
                    is_bigendian=int(bigendian), step=step or len(data) // height)
    message.data = array('B', data)
    return message


class ImageTests(unittest.TestCase):
    def test_padded_bgr_rows(self):
        msg = image(2, 2, 'bgr8', [0, 0, 255, 0, 255, 0, 99, 99,
                                  255, 0, 0, 255, 255, 255, 99, 99], step=8)
        np.testing.assert_array_equal(image_to_rgb(msg),
                                      [[[255, 0, 0], [0, 255, 0]],
                                       [[0, 0, 255], [255, 255, 255]]])

    def test_color_and_alpha_encodings(self):
        for encoding, values in [('rgb8', [10, 20, 30]), ('bgr8', [30, 20, 10]),
                                 ('rgba8', [10, 20, 30, 0]), ('bgra8', [30, 20, 10, 0])]:
            with self.subTest(encoding=encoding):
                np.testing.assert_array_equal(image_to_rgb(image(1, 1, encoding, values)),
                                              [[[10, 20, 30]]])

    def test_mono8(self):
        for encoding in ('mono8', '8UC1'):
            np.testing.assert_array_equal(image_to_rgb(image(2, 1, encoding, [0, 255])),
                                          [[[0, 0, 0], [255, 255, 255]]])

    def test_mono16_byte_order_and_padding(self):
        for encoding in ('mono16', '16UC1'):
            for bigendian in (False, True):
                payload = [0x12, 0x34, 0xFF, 0xFF] if bigendian else [0x34, 0x12, 0xFF, 0xFF]
                payload += [99, 99]
                with self.subTest(encoding=encoding, bigendian=bigendian):
                    np.testing.assert_array_equal(
                        image_to_rgb(image(2, 1, encoding, payload, step=6, bigendian=bigendian)),
                        [[[0x12] * 3, [0xFF] * 3]])

    def test_preview_preserves_aspect_and_does_not_upscale(self):
        msg = image(8, 4, 'mono8', list(range(32)))
        rgb = image_to_rgb(msg, 4, 4)
        self.assertEqual(rgb.shape, (2, 4, 3))
        np.testing.assert_array_equal(rgb[:, :, 0], [[0, 2, 4, 6], [16, 18, 20, 22]])
        self.assertEqual(image_to_rgb(msg, 20, 20).shape, (4, 8, 3))
        self.assertTrue(rgb.flags.c_contiguous)

    def test_invalid_images_are_rejected(self):
        cases = [image(0, 1, 'mono8', [1]), image(2, 1, 'rgb8', [1, 2], step=2),
                 image(2, 1, 'mono8', [1], step=2), image(1, 1, 'bayer_rggb8', [1])]
        for msg in cases:
            with self.subTest(encoding=msg.encoding, width=msg.width, step=msg.step):
                with self.assertRaises(ValueError):
                    image_to_rgb(msg)

    def test_ppm_binary_pixels(self):
        rgb = np.array([[[0, 10, 255], [255, 0, 10]]], dtype=np.uint8)
        self.assertEqual(ppm_bytes(rgb), b'P6\n2 1\n255\n\x00\n\xff\xff\x00\n')


class StreamHealthTests(unittest.TestCase):
    def setUp(self):
        self.node = SimpleNamespace(
            args=SimpleNamespace(width=640, height=480, stale_after=2., duration=0., fps=15.,
                                 camera='both', view_only=False, stop_on_exit=False, start_both=False),
            lock=threading.Lock(),
            streams={name: dict(topic='/' + name, message=None, received=None,
                                frames=0, times=deque(maxlen=60)) for name in ('head', 'hand')},
            preview={}, decoded_sequence={}, errors={}, decode_errors={'head': 0, 'hand': 0},
            get_logger=Mock(return_value=Mock()))

    def report(self):
        snapshots = CameraViewer.snapshots(self.node)
        CameraViewer.update_previews(self.node, snapshots)
        return CameraViewer.report(self.node, snapshots)

    def test_waiting_live_stale_and_recovery(self):
        self.assertEqual(self.report()['status'], 'NO_LIVE_STREAMS')
        msg = image(1, 1, 'mono8', [32])
        CameraViewer.receive(self.node, 'hand', msg)
        report = self.report()
        self.assertEqual(report['status'], 'PARTIAL')
        self.assertEqual(report['streams']['head']['status'], 'waiting')
        self.node.streams['hand']['received'] = time.monotonic() - 3.
        self.assertEqual(self.report()['streams']['hand']['status'], 'stale')
        CameraViewer.receive(self.node, 'hand', msg)
        CameraViewer.receive(self.node, 'head', msg)
        self.assertEqual(self.report()['status'], 'PASS')

    def test_decode_error_and_recovery(self):
        CameraViewer.receive(self.node, 'hand', image(1, 1, 'bayer_rggb8', [32]))
        report = self.report()
        self.assertEqual(report['streams']['hand']['status'], 'error')
        self.assertEqual(report['streams']['hand']['decode_errors'], 1)
        self.assertNotIn('hand', self.node.preview)
        self.report()  # An unchanged bad frame is not decoded repeatedly.
        self.assertEqual(self.node.decode_errors['hand'], 1)
        CameraViewer.receive(self.node, 'hand', image(1, 1, 'mono8', [32]))
        self.assertEqual(self.report()['streams']['hand']['status'], 'live')

    def test_window_starts_without_frames_and_displays_one_active_camera(self):
        self.node.snapshots = lambda: CameraViewer.snapshots(self.node)
        self.node.update_previews = lambda snapshots, sizes=None: CameraViewer.update_previews(self.node, snapshots, sizes)
        self.node.report = lambda snapshots: CameraViewer.report(self.node, snapshots)
        window = SimpleNamespace(closed=False, refresh_job=None, node=self.node, root=Mock(), tk=Mock(),
                                 started=time.monotonic(), photos={}, rendered_sequence={},
                                 images={name: Mock() for name in ('head', 'hand')},
                                 labels={name: Mock() for name in ('head', 'hand')})
        window.refresh = lambda: CameraWindow.refresh(window)
        for canvas in window.images.values():
            canvas.winfo_width.return_value = 640
            canvas.winfo_height.return_value = 480
        with patch('sawyer_ros2_control.camera_viewer.rclpy.ok', return_value=True):
            CameraWindow.refresh(window)
            self.assertEqual(window.photos, {})
            for label in window.labels.values():
                self.assertTrue(label.configure.call_args.kwargs['text'].startswith('WAITING'))
            CameraViewer.receive(self.node, 'hand', image(1, 1, 'mono8', [32]))
            CameraWindow.refresh(window)
            self.assertIn('hand', window.photos)
            self.assertNotIn('head', window.photos)
            self.assertTrue(window.labels['head'].configure.call_args.kwargs['text'].startswith('WAITING'))
            self.assertTrue(window.labels['hand'].configure.call_args.kwargs['text'].startswith('LIVE'))

    def test_selected_camera_report_requires_only_selected_stream(self):
        self.node.args.camera = 'hand'
        self.node.streams.pop('head')
        CameraViewer.receive(self.node, 'hand', image(1, 1, 'mono8', [32]))
        self.assertEqual(self.report()['status'], 'PASS')

    def test_preview_resizes_without_a_new_frame(self):
        CameraViewer.receive(self.node, 'hand', image(4, 2, 'mono8', list(range(8))))
        snapshots = CameraViewer.snapshots(self.node)
        sizes = {'head': (320, 240), 'hand': (320, 100)}
        CameraViewer.update_previews(self.node, snapshots, sizes)
        self.assertEqual(self.node.preview['hand'].shape, (100, 200, 3))
        sizes['hand'] = (60, 60)
        CameraViewer.update_previews(self.node, snapshots, sizes)
        self.assertEqual(self.node.preview['hand'].shape, (30, 60, 3))
        self.assertEqual(self.node.streams['hand']['frames'], 1)


class CameraCommandTests(unittest.TestCase):
    @patch('sawyer_ros2_control.camera_viewer.camera_command')
    @patch('sawyer_ros2_control.camera_viewer.run_viewer', return_value=0)
    def test_default_and_view_only_do_not_control_cameras(self, viewer, command):
        self.assertEqual(main([]), 0)
        self.assertEqual(main(['--camera', 'head', '--view-only']), 0)
        command.assert_not_called()
        self.assertEqual(viewer.call_count, 2)

    @patch('sawyer_ros2_control.camera_viewer.camera_command')
    @patch('sawyer_ros2_control.camera_viewer.run_viewer', return_value=0)
    def test_select_and_stop_on_exit(self, viewer, command):
        self.assertEqual(main(['--camera', 'head', '--stop-on-exit']), 0)
        self.assertEqual(command.call_args_list, [unittest.mock.call('start', 'head'),
                                                 unittest.mock.call('stop', 'head')])

    @patch('sawyer_ros2_control.camera_viewer.camera_command')
    @patch('sawyer_ros2_control.camera_viewer.run_viewer', return_value=0)
    def test_both_streams_require_explicit_opt_in_and_stop_on_exit_stops_all(self, viewer, command):
        self.assertEqual(main(['--start-both', '--stop-on-exit']), 0)
        self.assertEqual(command.call_args_list, [unittest.mock.call('start-both'),
                                                 unittest.mock.call('stop', 'all')])

    @patch('sawyer_ros2_control.camera_viewer.camera_command')
    @patch('sawyer_ros2_control.camera_viewer.run_viewer')
    def test_switch_only_stop_and_info_do_not_open_window(self, viewer, command):
        self.assertEqual(main(['--camera', 'hand', '--select-only']), 0)
        command.assert_called_with('start', 'hand')
        self.assertEqual(main(['--stop-stream', 'all']), 0)
        command.assert_called_with('stop', 'all', None)
        self.assertEqual(main(['--stream-info']), 0)
        command.assert_called_with('info', report=None)
        viewer.assert_not_called()

    @patch('sawyer_ros2_control.camera_viewer.camera_command', side_effect=OSError('unavailable'))
    @patch('sawyer_ros2_control.camera_viewer.run_viewer')
    def test_control_failure_does_not_open_viewer(self, viewer, command):
        self.assertEqual(main(['--camera', 'head']), 1)
        viewer.assert_not_called()

    @patch('sawyer_ros2_control.camera_viewer.camera_command', side_effect=[None, OSError('stop failed')])
    @patch('sawyer_ros2_control.camera_viewer.run_viewer', return_value=0)
    def test_stop_failure_is_not_hidden(self, viewer, command):
        self.assertEqual(main(['--camera', 'head', '--stop-on-exit']), 1)

    def test_invalid_control_combinations(self):
        for argv in (['--camera', 'head', '--stop-stream', 'all'], ['--stop-on-exit'],
                     ['--select-only'], ['--camera', 'hand', '--require-both'],
                     ['--camera', 'head', '--view-only', '--stop-on-exit'],
                     ['--start-both', '--camera', 'head'], ['--start-both', '--view-only'],
                     ['--start-both', '--stop-stream', 'all']):
            with self.subTest(argv=argv), patch('sys.stderr'), self.assertRaises(SystemExit):
                parse_args(argv)


if __name__ == '__main__':
    unittest.main()
