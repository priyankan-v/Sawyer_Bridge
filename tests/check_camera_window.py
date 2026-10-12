"""Hidden-window rendering check; requires a desktop display, no robot traffic."""
from array import array
import os
import time
import tkinter as tk

# Isolate this node before importing ROS; no publishers or external subscriptions.
os.environ.update(ROS_DOMAIN_ID='89', ROS_LOCALHOST_ONLY='1')
import rclpy
from sensor_msgs.msg import Image
from sawyer_ros2_control.camera_viewer import CameraViewer, CameraWindow, main, parse_args

rclpy.init()
node = CameraViewer(parse_args([]))
window = None
try:
    window = CameraWindow(node, tk)
    window.root.attributes('-alpha', 0.)
    window.root.update()
    assert node.args.layout == 'rows'
    for name, row in (('head', 0), ('hand', 1)):
        grid = window.images[name].master.grid_info()
        assert (grid['row'], grid['column']) == (row, 0)
    footer = window.root.winfo_children()[-1].grid_info()
    assert (footer['row'], footer['columnspan']) == (2, 1)
    window.refresh()
    window.root.update_idletasks()
    assert not window.photos
    assert all(label.cget('text').startswith('WAITING') for label in window.labels.values())
    node.receive('hand', Image(width=2, height=1, encoding='mono8', step=2,
                               data=array('B', [0, 255])))
    window.refresh()
    assert window.labels['head'].cget('text').startswith('WAITING')
    assert window.photos['hand'].get(window.photos['hand'].width()-1, 0) == (255, 255, 255)
    node.receive('head', Image(width=2, height=1, encoding='rgb8', step=6,
                               data=array('B', [255, 0, 0, 0, 255, 0])))
    window.refresh()
    window.root.update_idletasks()
    assert window.photos['head'].get(0, 0) == (255, 0, 0)
    assert window.photos['head'].get(window.photos['head'].width()-1, 0) == (0, 255, 0)
    assert window.photos['hand'].get(window.photos['hand'].width()-1, 0) == (255, 255, 255)
    for geometry in ('420x600', '900x760', '320x420'):
        window.root.geometry(geometry)
        window.root.update()
        window.refresh()
        for name, canvas in window.images.items():
            photo = window.photos[name]
            assert photo.width() <= canvas.winfo_width() and photo.height() <= canvas.winfo_height()
            assert abs(photo.width()/photo.height()-2) < .05
            position = canvas.coords(canvas.find_all()[0])
            assert position == [canvas.winfo_width()/2, canvas.winfo_height()/2]
    assert window.root.resizable() == (1, 1)
    node.streams['hand']['received'] = time.monotonic() - 3.
    window.refresh()
    assert 'hand' not in window.photos
    assert window.labels['hand'].cget('text').startswith('STALE')
    assert 'head' in window.photos
    assert not window.error
    window.close()
    window.close()  # Cleanup remains safe after a user closes the window.
    print('PASS: startup without frames; one active camera; both images; stale frames removed; clean close')
    node.args.layout = 'columns'
    window = CameraWindow(node, tk)
    window.root.attributes('-alpha', 0.)
    window.root.update()
    for name, column in (('head', 0), ('hand', 1)):
        grid = window.images[name].master.grid_info()
        assert (grid['row'], grid['column']) == (0, column)
    footer = window.root.winfo_children()[-1].grid_info()
    assert (footer['row'], footer['columnspan']) == (1, 2)
    window.refresh()
    window.root.update_idletasks()
    assert window.photos['head'].get(0, 0) == (255, 0, 0)
    assert window.labels['hand'].cget('text').startswith('STALE')
    assert not window.error
    for geometry in ('500x420', '1100x700', '400x320'):
        window.root.geometry(geometry)
        window.root.update()
        window.refresh()
        canvas, photo = window.images['head'], window.photos['head']
        assert photo.width() <= canvas.winfo_width() and photo.height() <= canvas.winfo_height()
        assert window.labels['hand'].cget('text').startswith('STALE')
    print('PASS: rows/columns grow and shrink; aspect ratio, centering, footer and stale status')
finally:
    if window is not None:
        window.close()
    node.destroy_node()
    rclpy.shutdown()


# Exercise the complete production GUI startup, executor and timed shutdown
# with no publishers in our isolated domain. Keep this test window hidden.
class HiddenTk(tk.Tk):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.withdraw()


tk.Tk = HiddenTk
assert main(['--duration', '0.3']) == 0
print('PASS: complete GUI application starts and shuts down without camera frames')
assert main(['--camera', 'head', '--view-only', '--duration', '0.3']) == 0
assert main(['--camera', 'hand', '--view-only', '--duration', '0.3']) == 0
print('PASS: head-only and hand-only desktop panes without camera control')
