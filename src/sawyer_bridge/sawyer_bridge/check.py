"""Check package ABI and mappings without connecting to a robot."""
import json
from pathlib import Path
import platform
import subprocess
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def main():
    try:
        release = dict(line.split('=', 1) for line in Path('/etc/os-release').read_text().splitlines() if '=' in line)
        if (platform.machine() != 'x86_64' or sys.version_info[:2] != (3, 10)
                or release.get('ID') != 'ubuntu' or release.get('VERSION_ID') != '"22.04"'):
            raise RuntimeError('Requires Ubuntu 22.04 x86_64 and system Python 3.10')
        import rclpy
        from intera_core_msgs.msg import JointCommand
        from intera_motion_msgs.msg import MotionStatus
        for interface in (JointCommand, MotionStatus):
            interface.__class__.__import_type_support__()
        from intera_core_msgs.action import CalibrationCommand
        from intera_motion_msgs.action import MotionCommand
        for interface in (CalibrationCommand, MotionCommand):
            interface.__class__.__import_type_support__()
        prefix = Path(get_package_prefix('sawyer_bridge'))
        bridge = prefix / 'lib/sawyer_bridge/dynamic_bridge'
        linked = subprocess.run(['/usr/bin/ldd', str(bridge)], capture_output=True, text=True, check=True)
        if 'not found' in linked.stdout:
            raise RuntimeError(linked.stdout)
        pairs = subprocess.run([str(bridge), '--print-pairs'], capture_output=True, text=True, check=True, timeout=20)
        for required in ('sensor_msgs/msg/JointState', 'intera_core_msgs/msg/JointCommand',
                         'intera_core_msgs/msg/HeadPanCommand'):
            if required not in pairs.stdout:
                raise RuntimeError('Missing mapping: ' + required)
        messages = [package + '/msg/' + file.stem
                    for package in ('intera_core_msgs', 'intera_motion_msgs')
                    for file in (Path(get_package_share_directory(package)) / 'msg').glob('*.msg')]
        for interface in messages:
            if interface not in pairs.stdout:
                raise RuntimeError('Missing Intera mapping: ' + interface)
        print(json.dumps({'status': 'PASS', 'intera_messages': len(messages), 'intera_services': 3,
                          'actions': ['CalibrationCommand', 'MotionCommand'],
                          'action_directions': ['1to2', '2to1'], 'prefix': str(prefix), 'python': sys.executable,
                          'scope': 'Shared libraries, ROS 2 typesupport and required mappings; no robot connection'}, indent=2))
        return 0
    except (OSError, RuntimeError, ImportError, subprocess.SubprocessError) as error:
        print('Package check failed: ' + str(error), file=sys.stderr)
        return 1
