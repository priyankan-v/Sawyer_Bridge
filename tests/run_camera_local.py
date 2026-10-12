"""Two simulated ROS 1 cameras through the real bridge; localhost only.

Run with system Python. Owns only its subprocesses and uses no physical robot.
"""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '.runtime/camera-local'
OUT.mkdir(parents=True, exist_ok=True)
os.chdir(ROOT)
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11422', ROS_IP='127.0.0.1',
                  ROS_DOMAIN_ID='88', ROS_LOCALHOST_ONLY='1')
os.environ.pop('ROS_HOSTNAME', None)
socket.setdefaulttimeout(2)
owned, logs = [], []

FIXTURE = '''
import json, rospy
from intera_core_msgs.msg import IOComponentCommand, IODataStatus, IODeviceConfiguration, IODeviceStatus
from sensor_msgs.msg import Image
rospy.init_node('camera_fixture')
head = rospy.Publisher('/io/internal_camera/head_camera/image_rect_color', Image, queue_size=1)
hand = rospy.Publisher('/io/internal_camera/right_hand_camera/image_rect', Image, queue_size=1)
# Deliberately use padded color rows; hand pixels exercise the full byte range.
color = Image(width=2, height=2, encoding='rgb8', step=8,
              data=bytes([255,0,0,0,255,0,99,99,0,0,255,255,255,255,99,99]))
mono = Image(width=2, height=2, encoding='mono8', step=2, data=bytes([0,64,128,255]))
names = {'head': 'head_camera', 'hand': 'right_hand_camera'}
enabled = {'head': True, 'hand': True}  # Two independent simulated sources.
acknowledged = {key: [] for key in names}
states, configs, subscriptions = {}, {}, []
rospy.set_param('/robot_config/camera_config', {
    'head_camera': {'cameraType': 'ienso_ethernet'}, 'right_hand_camera': {'cameraType': 'cognex'}})
def command(key, msg):
    args = json.loads(msg.args)
    assert msg.op == 'set' and set(args) == {'signals'}
    assert set(args['signals']) == {'camera_streaming'}
    signal = args['signals']['camera_streaming']
    assert signal['format'] == {'type': 'bool'} and type(signal['data'][0]) is bool
    enabled[key] = signal['data'][0]
    acknowledged[key].append(msg.time)
    acknowledged[key] = acknowledged[key][-10:]
    with open('.runtime/camera-local/commands.jsonl', 'a') as output:
        output.write(json.dumps({'camera': key, 'streaming': enabled[key]}) + '\\n')
for key, name in names.items():
    prefix = '/io/internal_camera/' + name
    states[key] = rospy.Publisher(prefix+'/state', IODeviceStatus, queue_size=1, latch=True)
    configs[key] = rospy.Publisher(prefix+'/config', IODeviceConfiguration, queue_size=1, latch=True)
    configs[key].publish(IODeviceConfiguration(commanded=json.dumps({'params': {'fps': 10, 'width': 2, 'height': 2}})))
    subscriptions.append(rospy.Subscriber(prefix+'/command', IOComponentCommand,
                                         lambda msg, key=key: command(key, msg), queue_size=1))
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    color.header.stamp = mono.header.stamp = rospy.Time.now()
    if enabled['head']: head.publish(color)
    if enabled['hand']: hand.publish(mono)
    for key in names:
        states[key].publish(IODeviceStatus(time=rospy.Time.now(), commands=acknowledged[key],
            signals=[IODataStatus(name='camera_streaming', format='{"type":"bool"}',
                                   data=json.dumps([enabled[key]]))]))
    rate.sleep()
'''

PIXEL_CHECK = '''
import time
import numpy as np
import rclpy
from sawyer_ros2_control.camera_viewer import CameraViewer, parse_args
rclpy.init()
node = CameraViewer(parse_args(['--headless', '--duration', '6']))
try:
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
        node.update_previews(node.snapshots())
        if all(stream['frames'] >= 5 for stream in node.streams.values()):
            break
    assert node.report()['status'] == 'PASS', node.report()
    np.testing.assert_array_equal(node.preview['head'],
        [[[255,0,0],[0,255,0]], [[0,0,255],[255,255,255]]])
    np.testing.assert_array_equal(node.preview['hand'],
        [[[0,0,0],[64,64,64]], [[128,128,128],[255,255,255]]])
    assert all(publisher.topic_name in ('/parameter_events', '/rosout')
               for publisher in node.publishers), 'Viewer created an application publisher'
    print('PASS: both converted image payloads and decoded pixels; no camera or motion publishers')
finally:
    node.destroy_node()
    rclpy.shutdown()
'''


def start(name, command):
    log = (OUT / (name + '.log')).open('w')
    logs.append(log)
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    owned.append(process)
    return process


def run(command):
    result = subprocess.run(command, timeout=25, text=True, capture_output=True)
    print(result.stdout, end='', flush=True)
    print(result.stderr, end='', flush=True)
    if result.returncode:
        raise RuntimeError(f'{command}: exit {result.returncode}')


try:
    # Refuse to join somebody else's master.
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(('127.0.0.1', 11422))
    master = start('master', ['scripts/env.sh', 'ros1', 'roscore', '-p', '11422'])
    deadline = time.monotonic() + 20
    while True:
        try:
            assert xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getUri('/camera_test')[0] == 1
            break
        except (OSError, AssertionError):
            if master.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Private ROS master failed; see ' + str(OUT / 'master.log'))
            time.sleep(.2)
    (OUT / 'commands.jsonl').write_text('')
    start('fixture', ['scripts/env.sh', 'ros1', 'python', '-c', FIXTURE])
    bridge = start('bridge', ['scripts/env.sh', 'ros2', 'ros2', 'launch',
                             'sawyer_ros2_control', 'bridge.launch.py', 'cameras:=true'])
    run(['scripts/env.sh', 'ros2', 'python', '-c', PIXEL_CHECK])
    if bridge.poll() is not None:
        raise RuntimeError('Bridge exited; see ' + str(OUT / 'bridge.log'))
    run(['scripts/env.sh', 'ros2', 'ros2', 'run', 'sawyer_ros2_control', 'camera_viewer',
         '--headless', '--duration', '6', '--require-both', '--report', str(OUT / 'viewer.json')])
    report = json.loads((OUT / 'viewer.json').read_text())
    assert report['status'] == 'PASS'
    assert all(stream['frames'] >= 30 and 8 < stream['receive_hz'] < 12
               for stream in report['streams'].values()), report
    master_api = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
    topics = master_api.getParam('/camera_test', '/sawyer_bridge/topics')[2]
    enabled = [entry for entry in topics if entry.get('enabled', True)]
    assert all(entry['direction'] == '1to2' for entry in enabled)
    publishers = dict(master_api.getSystemState('/camera_test')[2][0])
    expected = {
        '/rosout', '/rosout_agg', '/io/internal_camera/head_camera/image_rect_color',
        '/io/internal_camera/right_hand_camera/image_rect'}
    expected.update('/io/internal_camera/'+name+'/'+suffix
                    for name in ('head_camera', 'right_hand_camera') for suffix in ('state', 'config'))
    assert set(publishers) == expected, publishers
    print('PASS: opt-in launch, both streams ~10 FPS, no camera-control or motion publishers', flush=True)
    cli = ['scripts/env.sh', 'ros2', 'ros2', 'run', 'sawyer_ros2_control', 'camera_viewer']
    run(cli + ['--camera', 'head', '--headless', '--duration', '2', '--stop-on-exit',
               '--report', str(OUT / 'head.json')])
    selected = json.loads((OUT / 'head.json').read_text())
    assert selected['status'] == 'PASS' and set(selected['streams']) == {'head'}, selected
    run(cli + ['--camera', 'hand', '--select-only'])
    before = (OUT / 'commands.jsonl').read_text()
    run(cli + ['--camera', 'hand', '--view-only', '--headless', '--duration', '2'])
    run(cli + ['--stream-info', '--report', str(OUT / 'info.json')])
    assert (OUT / 'commands.jsonl').read_text() == before, 'Read-only options sent a command'
    info = json.loads((OUT / 'info.json').read_text())
    assert info['cameras']['hand']['streaming'] and not info['cameras']['head']['streaming']
    assert info['cameras']['hand']['frames'] >= 20
    run(cli + ['--stop-stream', 'all'])
    run(cli + ['--start-both', '--select-only'])
    before = (OUT / 'commands.jsonl').read_text()
    viewers = [start(key+'-view-only', cli+['--camera',key,'--view-only','--headless','--duration','2',
                                          '--report',str(OUT/(key+'-view-only.json'))]) for key in ('head','hand')]
    for key, process in zip(('head','hand'),viewers):
        assert process.wait(timeout=20) == 0, key
        row=json.loads((OUT/(key+'-view-only.json')).read_text())
        assert row['status']=='PASS' and set(row['streams'])=={key}, row
    assert (OUT / 'commands.jsonl').read_text() == before, 'Two read-only viewers changed streams'
    run(cli + ['--start-both', '--headless', '--duration', '2', '--require-both', '--stop-on-exit',
               '--report', str(OUT / 'both-controlled.json')])
    assert json.loads((OUT / 'both-controlled.json').read_text())['status'] == 'PASS'
    observed = [json.loads(line) for line in (OUT / 'commands.jsonl').read_text().splitlines()]
    transitions = []
    for event in observed:
        if not transitions or transitions[-1] != event:
            transitions.append(event)
    assert transitions == [dict(camera='hand', streaming=False), dict(camera='head', streaming=False),
                           dict(camera='hand', streaming=True), dict(camera='hand', streaming=False),
                           dict(camera='head', streaming=True), dict(camera='hand', streaming=True),
                           dict(camera='head', streaming=False), dict(camera='hand', streaming=False)], transitions
    publishers = dict(master_api.getSystemState('/camera_test')[2][0])
    assert not any(topic.startswith('/robot/') for topic in publishers), publishers
    print('PASS: selection, switch-only, stop-on-exit, stop-all, two concurrent read-only viewer nodes, info and acknowledged camera-only commands', flush=True)
finally:
    for process in reversed(owned):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    for process in reversed(owned):
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
    for log in logs:
        log.close()
