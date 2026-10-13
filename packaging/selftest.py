"""Test the installed bridge on isolated localhost graphs; never attach to Sawyer."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client

from sawyer_ros2_control.runtime import package_prefix, backend_environment, state_directory


def main():
    prefix = package_prefix()
    helper = prefix / 'libexec/sawyer'
    env = dict(os.environ)
    # Port is allocated locally; refuse collision rather than attaching to a master.
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        port = listener.getsockname()[1]
    env.update(ROS_MASTER_URI=f'http://127.0.0.1:{port}', ROS_IP='127.0.0.1',
               ROS_DOMAIN_ID=str(100 + port % 100), ROS_LOCALHOST_ONLY='1')
    env.pop('ROS_HOSTNAME', None)
    private = backend_environment(prefix)
    private.update({name: env[name] for name in ('ROS_MASTER_URI', 'ROS_IP', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY')})
    private.pop('ROS_HOSTNAME', None)
    owned = []
    with tempfile.TemporaryDirectory(prefix='sawyer-package-test-') as directory:
        handles = []
        def start(name, command, environment):
            log = open(Path(directory) / (name + '.log'), 'w')
            handles.append(log)
            process = subprocess.Popen(command, env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            owned.append(process)
            return process
        try:
            start('master', ['/usr/bin/python3', str(helper / 'test_fixture.py'), 'master', str(port)], private)
            socket.setdefaulttimeout(2)
            master = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
            deadline = time.monotonic() + 10
            while True:
                try:
                    assert master.getUri('/package_test')[0] == 1
                    break
                except (OSError, AssertionError):
                    if time.monotonic() > deadline:
                        raise RuntimeError('Private test master did not start')
                    time.sleep(.1)
            start('fixture', ['/usr/bin/python3', str(helper / 'test_fixture.py'), 'fixture'], private)
            topics = [
                ('joints', 'sensor_msgs/msg/JointState', '1to2'),
                ('image', 'sensor_msgs/msg/Image', '1to2'),
                ('command', 'intera_core_msgs/msg/JointCommand', '2to1'),
                ('head', 'intera_core_msgs/msg/HeadPanCommand', '2to1'),
                ('head_ack', 'intera_core_msgs/msg/HeadPanCommand', '1to2'),
                ('ack', 'std_msgs/msg/String', '1to2')]
            config = [dict(topic='/sawyer_pkg_test/' + name, type=typ, direction=direction,
                           queue_size=5, enabled=True) for name, typ, direction in topics]
            assert master.setParam('/package_test', '/sawyer_pkg_test/topics', config)[0] == 1
            bridge = start('bridge', [str(prefix / 'lib/ros1_bridge/parameter_bridge'),
                           '/sawyer_pkg_test/topics', '/sawyer_pkg_test/no_services', '/sawyer_pkg_test/no_services'], env)
            result = subprocess.run(['/usr/bin/python3', str(helper / 'test_client.py')], env=env,
                                    capture_output=True, text=True, timeout=30)
            if result.returncode:
                raise RuntimeError(result.stdout + result.stderr)
            assert bridge.poll() is None, 'Bridge exited unexpectedly'
            report = {'status': 'PASS', 'scope': 'Isolated localhost fixture; no physical robot commands',
                      'package_prefix': str(prefix), 'master': env['ROS_MASTER_URI'],
                      'domain': env['ROS_DOMAIN_ID'], 'checks': result.stdout.strip(), 'wall_time': time.time()}
            destination = state_directory() / 'local_transport.json'
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(report, indent=2))
            return 0
        except Exception:
            for log in Path(directory).glob('*.log'):
                print(log.name, log.read_text(), file=sys.stderr)
            raise
        finally:
            for process in reversed(owned):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
            for handle in handles:
                handle.close()


if __name__ == '__main__':
    raise SystemExit(main())
