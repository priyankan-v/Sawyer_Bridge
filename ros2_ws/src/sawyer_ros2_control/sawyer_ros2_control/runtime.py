"""Locate installed Sawyer files and isolate its private ROS 1 helpers."""
import argparse
import json
import os
from pathlib import Path
import platform
import subprocess
import sys


def package_prefix():
    from ament_index_python.packages import get_package_prefix
    return Path(get_package_prefix('sawyer_ros2_control'))


def state_directory():
    if os.environ.get('SAWYER_STATE_DIR'):
        return Path(os.environ['SAWYER_STATE_DIR'])
    if os.environ.get('SAWYER_ROOT'):
        return Path(os.environ['SAWYER_ROOT']) / '.runtime'
    base = Path(os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local/state')))
    return base / 'sawyer_ros2'


def transport_report():
    return state_directory() / 'local_transport.json'


def check_platform():
    if platform.machine() != 'x86_64' or 'VERSION_ID="22.04"' not in Path('/etc/os-release').read_text():
        raise RuntimeError('This package requires Ubuntu 22.04 x86_64')
    if sys.version_info[:2] != (3, 10):
        raise RuntimeError('Use system Humble with /usr/bin/python3 (Python 3.10)')
    if not Path('/opt/ros/humble/setup.bash').is_file():
        raise RuntimeError('Install ROS 2 Humble under /opt/ros/humble first')


def backend_environment(prefix=None):
    prefix = package_prefix() if prefix is None else Path(prefix)
    private = prefix / 'libexec/sawyer/ros1'
    if not (private / 'lib/libroscpp.so').exists():
        raise RuntimeError(f'Missing packaged ROS 1 client libraries: {private}')
    env = dict(os.environ)
    for name in ('PYTHONHOME', 'PYTHONPATH', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                 'CMAKE_PREFIX_PATH', 'LD_LIBRARY_PATH', 'ROS_PACKAGE_PATH'):
        env.pop(name, None)
    env.update(PATH='/usr/bin:/bin', PYTHONNOUSERSITE='1',
               PYTHONPATH=str(private / 'lib/python3/dist-packages'),
               LD_LIBRARY_PATH=str(private / 'lib'), ROS_PACKAGE_PATH=str(private / 'share'),
               ROS_VERSION='1', ROS_DISTRO='noetic', ROS_PYTHON_VERSION='3',
               ROS_ETC_DIR=str(private / 'etc/ros'))
    return env


def backend_command(name, arguments=()):
    prefix = package_prefix()
    script = prefix / 'libexec/sawyer' / (name + '.py')
    if not script.is_file():
        raise RuntimeError(f'Missing installed helper: {script}')
    return ['/usr/bin/python3', str(script), *map(str, arguments)], backend_environment(prefix)


def run_backend(name, arguments=(), timeout=35):
    command, env = backend_command(name, arguments)
    return subprocess.run(command, env=env, check=True, timeout=timeout)


def preflight_main():
    check_platform()
    command, env = backend_command('preflight', sys.argv[1:])
    os.execve(command[0], command, env)


def verify_main():
    check_platform()
    script = package_prefix() / 'libexec/sawyer/selftest.py'
    os.execve('/usr/bin/python3', ['/usr/bin/python3', str(script)], dict(os.environ))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--backend', choices=('camera_control', 'configure_bridge', 'preflight'))
    parser.add_argument('arguments', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    check_platform()
    if args.backend:
        command, env = backend_command(args.backend, args.arguments)
        os.execve(command[0], command, env)
    if args.check:
        private = backend_environment()
        subprocess.run(['/usr/bin/python3', '-c', 'import rospy,rosgraph; from intera_core_msgs.msg import JointCommand'],
                       env=private, check=True)
        import rclpy
        from intera_core_msgs.msg import JointCommand
        print(json.dumps({'python': sys.executable, 'prefix': str(package_prefix()),
                          'ros2': rclpy.__file__, 'private_ros1_imports': 'PASS',
                          'state_directory': str(state_directory())}, indent=2))
        return 0
    parser.error('Use --check or --backend')
