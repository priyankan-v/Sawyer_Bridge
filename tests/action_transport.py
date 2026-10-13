"""Test both action directions against native actionlib/rclpy, on localhost only."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client
import yaml
from ament_index_python.packages import get_package_prefix

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--ros1-prefix', type=Path, required=True)
parser.add_argument('--report', type=Path)
args = parser.parse_args()
actionlib = ROOT / '.runtime/actionlib-fixture/actionlib-1.14.3/actionlib/src'
assert (actionlib / 'actionlib').exists(), 'Run tests/prepare_actionlib.py first'
prefix = Path(get_package_prefix('sawyer_bridge'))
with socket.socket() as listener:
    listener.bind(('127.0.0.1', 0)); port = listener.getsockname()[1]
env = dict(os.environ, ROS_MASTER_URI=f'http://127.0.0.1:{port}', ROS_IP='127.0.0.1',
           ROS_DOMAIN_ID=str(100 + port % 100), ROS_LOCALHOST_ONLY='1')
env.pop('ROS_HOSTNAME', None)
private = dict(env, PYTHONPATH=str(args.ros1_prefix.resolve() / 'lib/python3/dist-packages') + ':' + str(actionlib),
               LD_LIBRARY_PATH=str(args.ros1_prefix.resolve() / 'lib'),
               ROS_PYTHON_LOG_CONFIG_FILE=str(ROOT / '.runtime/clean-build/sources/ros_comm/ros_comm-1.17.4/tools/rosgraph/conf/python_logging.conf'))
owned = []
with tempfile.TemporaryDirectory(prefix='sawyer-actions-test-') as directory:
    logs = []
    def start(name, command, environment):
        log = open(Path(directory) / (name + '.log'), 'w'); logs.append(log)
        process = subprocess.Popen(command, env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        owned.append(process); return process
    try:
        start('master', ['/usr/bin/python3', str(ROOT / 'tests/ros1_fixture.py'), 'master', str(port)], private)
        socket.setdefaulttimeout(2)
        master = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        deadline = time.monotonic() + 10
        while True:
            try:
                assert master.getUri('/fixture')[0] == 1; break
            except (OSError, AssertionError):
                if time.monotonic() > deadline: raise RuntimeError('Fixture master unavailable')
                time.sleep(.1)
        server1 = start('ros1-server', ['/usr/bin/python3', str(ROOT / 'tests/action_ros1.py'), 'server'], private)
        server2 = start('ros2-server', ['/usr/bin/python3', str(ROOT / 'tests/action_ros2.py'), 'server'], env)
        # Wait for native fixtures, then let the package discover all endpoints.
        deadline = time.monotonic() + 10
        while '/sawyer_pkg_test/forward_io' not in dict(master.getSystemState('/fixture')[2][2]):
            if time.monotonic() > deadline: raise RuntimeError('ROS 1 server fixture failed')
            time.sleep(.1)
        deadline = time.monotonic() + 10
        while 'Native ROS 2 action servers ready' not in (Path(directory) / 'ros2-server.log').read_text():
            if time.monotonic() > deadline: raise RuntimeError('ROS 2 server fixture failed')
            time.sleep(.1)
        settings = Path(directory) / 'bridge.yaml'
        settings.write_text(yaml.safe_dump(dict(topics=[], actions=[], services_1_to_2=[], services_2_to_1=[])))
        launch = start('bridge', ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'sawyer_bridge',
                                  'bridge.launch.py', 'config:=' + str(settings), 'all_interfaces:=true'], env)
        checks = []
        for script, environment in [('action_ros2.py', env), ('action_ros1.py', private)]:
            result = subprocess.run(['/usr/bin/python3', str(ROOT / 'tests' / script), 'client'],
                                    env=environment, capture_output=True, text=True, timeout=60)
            if result.returncode: raise RuntimeError(result.stdout + result.stderr)
            checks.append(result.stdout.strip())
        assert launch.poll() is None
        actions = master.getParam('/fixture', '/sawyer_bridge/actions')[2]
        assert len(actions) == 4, actions
        assert {entry['direction'] for entry in actions} == {'1to2', '2to1'}, actions
        for key in ('services_1_to_2', 'services_2_to_1'):
            assert len(master.getParam('/fixture', '/sawyer_bridge/' + key)[2]) == 3, key
        for script, environment, server in [('action_ros1.py', private, server2), ('action_ros2.py', env, server1)]:
            ready = Path(directory) / (script + '.ready')
            client = start(script + '-loss', ['/usr/bin/python3', str(ROOT / 'tests' / script), 'loss', str(ready)], environment)
            deadline = time.monotonic() + 12
            while not ready.exists():
                if client.poll() is not None or time.monotonic() > deadline: raise RuntimeError('Loss-test goal did not activate: ' + script)
                time.sleep(.02)
            os.killpg(server.pid, signal.SIGKILL); server.wait(timeout=3)
            assert client.wait(timeout=50) == 0, 'Server-loss handling failed: ' + script
            checks.append((Path(directory) / (script + '-loss.log')).read_text().strip())
        report = dict(status='PASS', package_prefix=str(prefix), scope='Isolated native actionlib/rclpy fixtures; no physical robot commands', checks=checks,
                      actions=actions, services_per_direction=3)
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True); args.report.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    except Exception:
        for log in Path(directory).glob('*.log'): print(log.name + '\n' + log.read_text())
        raise
    finally:
        for process in reversed(owned):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait()
        for log in logs: log.close()
