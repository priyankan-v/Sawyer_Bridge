"""Audit a fresh Conda installation for one bridge package and runtime-only payload."""
import argparse
import json
import os
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix', type=Path)
    args = parser.parse_args()
    prefix = args.prefix.resolve()
    records = list((prefix / 'conda-meta').glob('*.json'))
    assert len(records) == 1, 'Run this audit on a fresh package-only prefix'
    record = json.loads(records[0].read_text())
    assert record['name'] == 'sawyer_bridge', record['name']
    packages = {path.name for path in (prefix / 'share/ament_index/resource_index/packages').iterdir()}
    assert packages == {'sawyer_bridge', 'intera_core_msgs', 'intera_motion_msgs'}, packages
    assert not (prefix / 'share/ros1_bridge').exists()
    assert not (prefix / 'lib/ros1_bridge').exists()
    assert not (prefix / 'lib/libros1_bridge.so').exists()
    assert (prefix / 'lib/libsawyer_bridge.so').exists()
    assert {path.name for path in (prefix / 'lib/sawyer_bridge').iterdir()} == {
        'check', 'configure_bridge', 'parameter_bridge', 'dynamic_bridge'}
    runtime = prefix / 'libexec/sawyer/ros1'
    assert {path.name for path in runtime.iterdir()} == {'lib'}
    assert len(list((runtime / 'lib').iterdir())) == 8
    assert not any(name in path for path in record['files'] for name in (
        'move_joint', 'camera_viewer', 'preflight', 'sawyer_ros2_control', 'rospy', 'rosmaster'))
    count = 0
    environment = dict(os.environ, LD_LIBRARY_PATH='/opt/ros/humble/lib')
    for path in prefix.rglob('*'):
        if not path.is_file() or path.is_symlink():
            continue
        with path.open('rb') as stream:
            if stream.read(4) != b'\x7fELF':
                continue
        result = subprocess.run(['/usr/bin/ldd', str(path)], env=environment,
                                capture_output=True, text=True, check=True)
        assert 'not found' not in result.stdout, (path, result.stdout)
        assert '.runtime/clean-build/' not in result.stdout, (path, result.stdout)
        count += 1
    print(json.dumps({'status': 'PASS', 'ros_packages': sorted(packages),
                      'elf_files': count, 'private_ros1_libraries': 8,
                      'scope': 'One bridge package; no legacy apps or build-only dependencies'}, indent=2))


if __name__ == '__main__':
    main()
