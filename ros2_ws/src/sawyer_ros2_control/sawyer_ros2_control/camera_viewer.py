"""View Sawyer cameras; explicit selection/stop commands run in isolated Noetic."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image

HEAD_TOPIC = '/io/internal_camera/head_camera/image_rect_color'
HAND_TOPIC = '/io/internal_camera/right_hand_camera/image_rect'


def image_to_rgb(message, max_width=640, max_height=480, allow_upscale=False):
    """Decode common Image encodings, honoring row stride and byte order.

    Return an RGB uint8 preview fitted to the bounds while preserving aspect ratio.
    Rectified topics avoid a dependency on Bayer demosaicing or camera calibration.
    """
    encodings = {
        'rgb8': (3, 'u1'), 'bgr8': (3, 'u1'),
        'rgba8': (4, 'u1'), 'bgra8': (4, 'u1'),
        'mono8': (1, 'u1'), '8UC1': (1, 'u1'),
        'mono16': (1, 'u2'), '16UC1': (1, 'u2'),
    }
    if message.encoding not in encodings:
        raise ValueError(f'Unsupported encoding {message.encoding!r}; use a rectified RGB/mono topic')
    if message.width <= 0 or message.height <= 0 or max_width <= 0 or max_height <= 0:
        raise ValueError('Image and preview dimensions must be positive')
    channels, kind = encodings[message.encoding]
    dtype = np.dtype(kind).newbyteorder('>' if message.is_bigendian else '<')
    pixel_bytes = channels * dtype.itemsize
    if message.step < message.width * pixel_bytes:
        raise ValueError('Image row stride is smaller than its pixel data')
    if len(message.data) != message.step * message.height:
        raise ValueError('Image payload length does not match height and row stride')
    pixels = np.ndarray((message.height, message.width, channels), dtype=dtype,
                        buffer=memoryview(message.data),
                        strides=(message.step, pixel_bytes, dtype.itemsize))
    scale = min(max_width / message.width, max_height / message.height)
    if not allow_upscale:
        scale = min(1., scale)
    width, height = max(1, int(message.width * scale)), max(1, int(message.height * scale))
    if (width, height) != (message.width, message.height):
        rows = np.arange(height) * message.height // height
        columns = np.arange(width) * message.width // width
        pixels = pixels[rows[:, None], columns[None, :], :]
    if dtype.itemsize == 2:
        pixels = (pixels >> 8).astype(np.uint8)
    if channels == 1:
        pixels = np.repeat(pixels, 3, axis=2)
    else:
        pixels = pixels[:, :, :3]
        if message.encoding in ('bgr8', 'bgra8'):
            pixels = pixels[:, :, ::-1]
    return np.ascontiguousarray(pixels)


def ppm_bytes(rgb):
    """Tk's binary PPM input avoids OpenCV/cv_bridge dependencies."""
    height, width, _ = rgb.shape
    return f'P6\n{width} {height}\n255\n'.encode('ascii') + rgb.tobytes()


class CameraViewer(Node):
    def __init__(self, args):
        super().__init__('sawyer_camera_viewer' + ('' if args.camera == 'both' else '_' + args.camera))
        self.args = args
        self.lock = threading.Lock()
        self.streams = {
            name: dict(topic=topic, message=None, received=None, frames=0, times=deque(maxlen=60))
            for name, topic in (('head', args.head_topic), ('hand', args.hand_topic))
            if args.camera == 'both' or args.camera == name
        }
        self.preview = {}
        self.decoded_sequence = {}
        self.errors = {}
        self.decode_errors = dict.fromkeys(self.streams, 0)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions_owned = [
            self.create_subscription(Image, stream['topic'],
                                     lambda msg, key=name: self.receive(key, msg), qos)
            for name, stream in self.streams.items()
        ]
        for name, stream in self.streams.items():
            self.get_logger().info(f'{name}: subscribing to {stream["topic"]}')

    def receive(self, name, message):
        with self.lock:
            stream = self.streams[name]
            stream.update(message=message, received=time.monotonic(), frames=stream['frames'] + 1)
            stream['times'].append(stream['received'])

    def snapshots(self):
        with self.lock:
            return {name: dict(stream, times=list(stream['times']))
                    for name, stream in self.streams.items()}

    def update_previews(self, snapshots, sizes=None):
        # Runs only on the display/main thread; the ROS thread stores messages.
        for name, stream in snapshots.items():
            size = sizes[name] if sizes is not None else (self.args.width, self.args.height)
            key = (stream['frames'], *size)
            if stream['message'] is None or self.decoded_sequence.get(name) == key:
                continue
            self.decoded_sequence[name] = key
            try:
                self.preview[name] = image_to_rgb(stream['message'], *size, allow_upscale=sizes is not None)
                self.errors.pop(name, None)
            except (ValueError, TypeError, BufferError) as error:
                text = str(error)
                if self.errors.get(name) != text:
                    self.get_logger().error(f'{name}: {text}')
                self.errors[name] = text
                self.decode_errors[name] += 1
                self.preview.pop(name, None)

    def report(self, snapshots=None):
        snapshots = self.snapshots() if snapshots is None else snapshots
        now = time.monotonic()
        rows = {}
        for name, stream in snapshots.items():
            age = None if stream['received'] is None else max(0., now - stream['received'])
            if age is None:
                status = 'waiting'
            elif age > self.args.stale_after:
                status = 'stale'
            elif name in self.errors:
                status = 'error'
            else:
                status = 'live'
            times = stream['times']
            hz = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else 0.
            msg = stream['message']
            rows[name] = dict(topic=stream['topic'], status=status, frames=stream['frames'],
                              receive_hz=hz if status == 'live' else 0., age_seconds=age,
                              encoding=msg.encoding if msg else None,
                              width=msg.width if msg else None, height=msg.height if msg else None,
                              error=self.errors.get(name), decode_errors=self.decode_errors[name])
        live = sum(row['status'] == 'live' for row in rows.values())
        status = 'PASS' if live == len(rows) else ('PARTIAL' if live else 'NO_LIVE_STREAMS')
        control = ('both-started-via-ros1' if self.args.start_both else
                   'selected-via-ros1' if self.args.camera != 'both' and not self.args.view_only else 'read-only')
        return dict(status=status, streams=rows, camera_control=control)


class CameraWindow:
    def __init__(self, node, tk):
        self.node, self.tk = node, tk
        self.root = tk.Tk()
        self.root.title('Sawyer camera viewer')
        self.root.resizable(True, True)
        self.root.minsize(280, 240)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.report_callback_exception = self.callback_error
        self.root.bind('<Escape>', lambda event: self.close())
        self.root.bind('q', lambda event: self.close())
        self.closed = False
        self.error = None
        self.refresh_job = None
        self.started = time.monotonic()
        self.photos, self.rendered_sequence, self.images, self.labels = {}, {}, {}, {}
        self.topic_labels = {}
        stacked = node.args.layout == 'rows'
        for index, (name, stream) in enumerate(node.streams.items()):
            panel = tk.Frame(self.root, padx=8, pady=8)
            row, column = (index, 0) if stacked else (0, index)
            self.root.rowconfigure(row, weight=1, uniform='cameras')
            self.root.columnconfigure(column, weight=1, uniform='cameras')
            panel.grid(row=row, column=column, sticky='nsew')
            panel.columnconfigure(0, weight=1)
            panel.rowconfigure(2, weight=1, minsize=16)
            tk.Label(panel, text='Head camera' if name == 'head' else 'Right-hand camera',
                     font=('TkDefaultFont', 12, 'bold')).grid(row=0, column=0, sticky='ew')
            self.topic_labels[name] = tk.Label(panel, text=stream['topic'], wraplength=node.args.width)
            self.topic_labels[name].grid(row=1, column=0, sticky='ew')
            canvas = tk.Canvas(panel, width=node.args.width, height=node.args.height,
                               background='#181818', highlightthickness=0)
            canvas.grid(row=2, column=0, sticky='nsew', pady=6)
            self.images[name] = canvas
            self.labels[name] = tk.Label(panel, text='Waiting for frames', wraplength=node.args.width)
            self.labels[name].grid(row=3, column=0, sticky='ew')
            canvas.bind('<Configure>', lambda event, key=name: self.pane_resized(key, event))
        footer = ('Q / Esc / Ctrl-C to exit. Robot stream stops on exit.' if node.args.stop_on_exit else
                  'Q / Esc / Ctrl-C to exit. Closing this window leaves the robot stream running.')
        self.footer = tk.Label(self.root, text=footer,
                               wraplength=node.args.width * (1 if stacked else len(node.streams)))
        self.footer.grid(
            row=len(node.streams) if stacked else 1, column=0,
            columnspan=1 if stacked else len(node.streams), pady=8, sticky='ew')
        self.root.bind('<Configure>', self.window_resized)
        # Stacked panes should start inside the desktop rather than extend below it.
        self.root.update_idletasks()
        width = min(self.root.winfo_reqwidth(), int(self.root.winfo_screenwidth() * .9))
        height = min(self.root.winfo_reqheight(), int(self.root.winfo_screenheight() * .85))
        self.root.geometry(f'{max(280, width)}x{max(240, height)}')

    def pane_resized(self, name, event):
        width = max(1, event.width)
        self.topic_labels[name].configure(wraplength=width)
        self.labels[name].configure(wraplength=width)

    def window_resized(self, event):
        if event.widget is self.root:
            self.footer.configure(wraplength=max(1, event.width - 16))

    def refresh(self):
        if self.closed:
            return
        if self.refresh_job is not None:
            self.root.after_cancel(self.refresh_job)
            self.refresh_job = None
        if not rclpy.ok() or (self.node.args.duration and time.monotonic() - self.started >= self.node.args.duration):
            self.close()
            return
        snapshots = self.node.snapshots()
        sizes = {name: (max(1, canvas.winfo_width()), max(1, canvas.winfo_height()))
                 for name, canvas in self.images.items()}
        self.node.update_previews(snapshots, sizes)
        for name, row in self.node.report(snapshots)['streams'].items():
            canvas = self.images[name]
            width, height = sizes[name]
            key = (row['frames'], width, height)
            if row['status'] == 'live':
                if self.rendered_sequence.get(name) != key:
                    photo = self.tk.PhotoImage(data=ppm_bytes(self.node.preview[name]), format='PPM', master=self.root)
                    self.photos[name] = photo  # Tk images must retain a Python reference.
                    canvas.delete('all')
                    canvas.create_image(width / 2, height / 2, image=photo)
                    self.rendered_sequence[name] = key
                text = f'LIVE · {row["receive_hz"]:.1f} FPS · {row["width"]}×{row["height"]} · {row["encoding"]}'
            else:
                # Do not present an old picture as a live video stream.
                canvas.delete('all')
                self.photos.pop(name, None)
                self.rendered_sequence.pop(name, None)
                if row['status'] == 'waiting':
                    text = 'WAITING — camera may not be streaming'
                elif row['status'] == 'stale':
                    text = f'STALE — no frame for {row["age_seconds"]:.1f} s'
                else:
                    text = 'ERROR — ' + (row['error'] or 'invalid image')
                canvas.create_text(width / 2, height / 2, text=text,
                                   fill='white', width=max(1, width - 20))
            self.labels[name].configure(text=text)
        self.refresh_job = self.root.after(max(1, round(1000 / self.node.args.fps)), self.refresh)

    def close(self):
        if not self.closed:
            self.closed = True
            if self.refresh_job is not None:
                self.root.after_cancel(self.refresh_job)
                self.refresh_job = None
            # Release Tk image references on the GUI thread and break the root's
            # bound-method cycle. ROS-thread garbage collection must not destroy
            # a retired Tcl interpreter when another window is opened.
            self.photos.clear()
            self.root.report_callback_exception = None
            self.root.destroy()

    def callback_error(self, exception_type, error, traceback):
        # Tk otherwise prints callback failures and keeps an unusable window open.
        self.error = str(error)
        print(f'Camera display error: {exception_type.__name__}: {error}', flush=True)
        self.close()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    control = parser.add_mutually_exclusive_group()
    control.add_argument('--camera', choices=('head', 'hand', 'both'), default='both',
                         help='Head/hand switches the robot stream; both views existing streams only')
    control.add_argument('--stop-stream', choices=('head', 'hand', 'all'),
                         help='Stop robot camera streaming and exit without opening a window')
    control.add_argument('--stream-info', action='store_true',
                         help='Read camera configuration and sample image rates, then exit')
    parser.add_argument('--view-only', action='store_true', help='Show selected pane without camera control')
    parser.add_argument('--start-both', action='store_true',
                        help='Explicitly enable both camera IO streams; verified on this controller')
    parser.add_argument('--select-only', action='store_true', help='Switch camera and exit; keep an existing viewer open')
    parser.add_argument('--stop-on-exit', action='store_true', help='Stop the selected robot camera when this viewer exits')
    parser.add_argument('--head-topic', default=HEAD_TOPIC)
    parser.add_argument('--hand-topic', default=HAND_TOPIC)
    parser.add_argument('--layout', choices=('rows', 'columns'), default='rows',
                        help='Rows stacks cameras vertically (default); columns places them side by side')
    parser.add_argument('--width', type=int, default=640, help='Initial pane width; headless maximum preview width')
    parser.add_argument('--height', type=int, default=480, help='Initial pane height; headless maximum preview height')
    parser.add_argument('--fps', type=float, default=15., help='Maximum GUI refresh rate')
    parser.add_argument('--stale-after', type=float, default=2., help='Seconds without a frame before marking stale')
    parser.add_argument('--duration', type=float, default=0., help='Exit after this many seconds; 0 runs until closed')
    parser.add_argument('--headless', action='store_true', help='Validate streams without opening a window')
    parser.add_argument('--require-both', action='store_true', help='Exit with failure unless both streams are live')
    parser.add_argument('--report', type=Path, help='Write final frame statistics as JSON')
    args = parser.parse_args(argv)
    if args.start_both and (args.camera != 'both' or args.view_only or args.stop_stream or args.stream_info):
        parser.error('--start-both requires --camera both (the default) with camera control')
    if (args.select_only or args.stop_on_exit) and (
            (args.camera == 'both' and not args.start_both) or args.view_only or args.stop_stream or args.stream_info):
        parser.error('--select-only/--stop-on-exit require --camera head/hand or --start-both')
    if args.select_only and args.stop_on_exit:
        parser.error('--select-only and --stop-on-exit cannot be combined')
    if args.view_only and (args.stop_stream or args.stream_info):
        parser.error('--view-only applies to camera viewing')
    if args.require_both and args.camera != 'both':
        parser.error('--require-both requires the default two-pane viewer or --camera both')
    if not 16 <= args.width <= 1920 or not 16 <= args.height <= 1080:
        parser.error('Preview dimensions must be 16–1920 wide and 16–1080 high')
    if (not math.isfinite(args.fps) or not 1 <= args.fps <= 60
            or not math.isfinite(args.stale_after) or args.stale_after <= 0
            or not math.isfinite(args.duration) or args.duration < 0):
        parser.error('FPS must be 1–60; stale timeout positive; duration nonnegative and finite')
    if args.headless and args.duration == 0:
        parser.error('--headless requires a positive --duration')
    if not args.head_topic or not args.hand_topic or args.head_topic == args.hand_topic:
        parser.error('Specify two distinct, nonempty camera topics')
    return args


def run_viewer(args):
    node = executor = thread = window = None
    try:
        tk = None
        if not args.headless:
            import tkinter as tk
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = CameraViewer(args)
        if args.headless:
            deadline = time.monotonic() + args.duration
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=min(.05, max(0., deadline - time.monotonic())))
                node.update_previews(node.snapshots())
        else:
            window = CameraWindow(node, tk)
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            window.refresh()
            window.root.mainloop()
        node.update_previews(node.snapshots())
        report = node.report()
        print(json.dumps(report, indent=2), flush=True)
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + '\n')
        if window is not None and window.error is not None:
            return 1
        if args.require_both:
            return 0 if report['status'] == 'PASS' else 1
        if args.headless:
            return 0 if report['status'] in ('PASS', 'PARTIAL') else 1
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        # Display/library failures must reach the terminal rather than leave a
        # silently empty window. ROS callbacks never publish or control cameras.
        print(f'Camera viewer error: {error}', flush=True)
        return 1
    finally:
        if window is not None:
            window.close()
        if executor is not None:
            executor.shutdown(timeout_sec=2.)
        if thread is not None:
            thread.join(timeout=2.)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


def camera_command(action, camera=None, report=None):
    root = Path(os.environ['SAWYER_ROOT'])
    command = [str(root / 'scripts/env.sh'), 'ros1', 'python',
               str(root / 'scripts/camera_control.py'), action]
    if camera is not None:
        command.append(camera)
    if report is not None:
        command.extend(['--report', str(report)])
    # No Noetic imports or IO command publishers enter the ROS 2 process.
    subprocess.run(command, check=True, timeout=35)


def main(argv=None):
    args = parse_args(argv)
    started = False
    exit_code = 1
    try:
        if args.stop_stream:
            camera_command('stop', args.stop_stream, args.report)
            return 0
        if args.stream_info:
            camera_command('info', report=args.report)
            return 0
        if args.start_both:
            camera_command('start-both')
            started = True
        elif args.camera != 'both' and not args.view_only:
            camera_command('start', args.camera)
            started = True
        if args.select_only:
            return 0
        exit_code = run_viewer(args)
    except KeyboardInterrupt:
        exit_code = 130
    except (OSError, KeyError, subprocess.SubprocessError) as error:
        print(f'Camera command failed: {error}', flush=True)
    finally:
        if started and args.stop_on_exit:
            try:
                camera_command('stop', 'all' if args.start_both else args.camera)
            except (OSError, KeyError, subprocess.SubprocessError) as error:
                print(f'Stop-on-exit failed: {error}', flush=True)
                exit_code = 1
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
