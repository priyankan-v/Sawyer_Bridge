"""Exercise gripper commands through ROS 1/2 on a private localhost graph."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[1]
MASTER = 'http://127.0.0.1:11423'
DOMAIN = '89'
WORK = ROOT / '.runtime/gripper-local'


def fixture():
    import rospy
    from intera_core_msgs.msg import IOComponentCommand, IOComponentStatus, IODataStatus, IODeviceStatus, IOStatus

    assert os.environ['ROS_MASTER_URI'] == MASTER
    rospy.init_node('isolated_gripper_fixture')
    values = dict(has_error=False, is_calibrated=False, is_moving=False,
                  is_gripping=False, position_response_m=0., position_m=0., go=True, calibrate=False,
                  dead_zone_m=0.002)
    acknowledgements = []
    publisher = rospy.Publisher('/io/end_effector/right_gripper/state', IODeviceStatus, queue_size=1, latch=True)
    lock = threading.Lock()

    def publish_state():
        publisher.publish(IODeviceStatus(time=rospy.Time.now(),
            device=IOComponentStatus(name='right_gripper',
                                     status=IOStatus(tag='ready' if values['is_calibrated'] else 'unready')),
            signals=[IODataStatus(name=name, data=json.dumps([value]),
                                 format=json.dumps(dict(type='bool' if type(value) is bool else 'float')),
                                 status=IOStatus(tag='ready')) for name, value in values.items()],
            commands=acknowledgements[-20:]))

    def receive(message):
        with lock:
            assert message.op == 'set'
            if message.time in acknowledgements or (WORK / 'suppress-feedback').exists():
                return
            signals = json.loads(message.args)['signals']
            for name, component in signals.items():
                assert name in ('calibrate', 'position_m', 'go')
                value = component['data'][0]
                assert component['format']['type'] == ('bool' if type(value) is bool else 'float')
                values[name] = value
            if signals.get('calibrate', {}).get('data') == [True]:
                values['is_calibrated'] = True
            if 'position_m' in signals:
                assert values['is_calibrated'] and 0 <= values['position_m'] <= 0.041667
                # Match the measured open residual: outside 1 mm, inside the 2 mm device dead zone.
                values['position_response_m'] = max(0.01, values['position_m'] - 0.0011160803571429)
                if (WORK / 'blocked-motion').exists():
                    values['position_response_m'] = 0.01
                values['is_gripping'] = values['position_m'] == 0.
            acknowledgements.append(message.time)
            with (WORK / 'commands.jsonl').open('a') as stream:
                stream.write(json.dumps(signals) + '\n')
            publish_state()

    rospy.Subscriber('/io/end_effector/right_gripper/command', IOComponentCommand, receive, queue_size=1)
    # Match Sawyer: one latched initial state, then publish only on change.
    with lock:
        publish_state()
    rospy.spin()


def run():
    os.chdir(ROOT)
    os.environ.update(ROS_MASTER_URI=MASTER, ROS_IP='127.0.0.1',
                      ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='1')
    os.environ.pop('ROS_HOSTNAME', None)
    WORK.mkdir(parents=True, exist_ok=True)
    (WORK / 'commands.jsonl').write_text('')
    (WORK / 'suppress-feedback').unlink(missing_ok=True)
    (WORK / 'blocked-motion').unlink(missing_ok=True)
    processes, logs = [], []

    def start(name, command):
        log = (WORK / (name + '.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    def stop(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)

    def command(*args, expected=0):
        result = subprocess.run(['scripts/env.sh', 'ros2', 'ros2', 'run', 'sawyer_ros2_control',
                                 'gripper', *args, '--timeout', '3'], capture_output=True, text=True, timeout=15)
        assert result.returncode == expected, result.stdout + result.stderr
        return json.loads(result.stdout) if expected == 0 else result.stderr

    try:
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 11423))  # Refuse an existing ROS master.
        start('master', ['scripts/env.sh', 'ros1', 'roscore', '-p', '11423'])
        deadline = time.monotonic() + 15
        while True:
            try:
                assert xmlrpc.client.ServerProxy(MASTER).getUri('/gripper_test')[0] == 1
                break
            except (OSError, AssertionError):
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.2)
        start('fixture', ['scripts/env.sh', 'ros1', 'python', __file__, '--fixture'])
        launch = ['scripts/env.sh', 'ros2', 'ros2', 'launch', 'sawyer_ros2_control', 'bridge.launch.py']
        bridge = start('telemetry_bridge', launch)
        assert command()['signals']['is_calibrated'] is False
        time.sleep(1.2)
        assert command()['signals']['is_calibrated'] is False  # Late subscriber, no new state publication.
        assert command('calibrate', '--dry-run')['status'] == 'dry_run'
        assert 'not calibrated' in command('open', expected=1)
        assert 'subscription' in command('calibrate', expected=1)
        assert (WORK / 'commands.jsonl').read_text() == ''
        stop(bridge)
        start('command_bridge', launch + ['gripper_commands:=true'])
        for operation in [('calibrate',), ('open',), ('position', '--position', '0.02'), ('close',), ('stop',)]:
            result = command(*operation)
            assert result['status'] == 'completed', result
            if operation == ('close',):
                assert result['signals']['is_gripping'] and result['signals']['position_response_m'] == 0.01
            if operation[0] in ('open', 'position'):
                assert 0.001 < result['position_error_m'] < result['tolerance_m'] == 0.002
        commands = [json.loads(line) for line in (WORK / 'commands.jsonl').read_text().splitlines()]
        assert len(commands) == 5, commands
        (WORK / 'blocked-motion').touch()
        error = command('open', expected=1)
        assert 'measured=0.010000 m' in error and 'tolerance=0.002000 m' in error, error
        (WORK / 'suppress-feedback').touch()
        assert 'acknowledgement' in command('stop', expected=1)  # Cached state cannot acknowledge a new command.
        print('PASS: retained state, late clients, dry-run, disabled commands, calibration, device dead zone, open, position, object grasp, stop, blocked motion and missing acknowledgement; localhost only.')
    finally:
        for process in reversed(processes):
            stop(process)
        for log in logs:
            log.close()
        (WORK / 'suppress-feedback').unlink(missing_ok=True)
        (WORK / 'blocked-motion').unlink(missing_ok=True)


if __name__ == '__main__':
    if sys.argv[1:] == ['--fixture']:
        fixture()
    else:
        run()
