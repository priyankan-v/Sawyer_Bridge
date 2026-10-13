#!/usr/bin/python3
"""Build a Sawyer overlay for system Humble, including private ROS 1 dependencies."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import tarfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = {
    'catkin': ['.'], 'genmsg': ['.'], 'gencpp': ['.'], 'genpy': ['.'],
    'message_generation': ['.'], 'message_runtime': ['.'],
    'roscpp_core': ['cpp_common', 'rostime', 'roscpp_traits', 'roscpp_serialization'],
    'ros_comm': ['clients/roscpp', 'clients/rospy', 'utilities/xmlrpcpp',
                 'tools/rosgraph', 'tools/rosparam', 'tools/rosmsg'],
    'ros_comm_msgs': ['rosgraph_msgs'],
    'rosconsole': ['.'], 'ros': ['core/roslib', 'core/roslang', 'tools/rosunit'],
    'std_msgs': ['.'], 'common_msgs': ['geometry_msgs', 'sensor_msgs', 'actionlib_msgs'],
    'rospack': ['.'], 'cmake_modules': ['.'], 'ros_environment': ['.'],
}


def run(command, env=None, cwd=None):
    print('+', ' '.join(map(str, command)), flush=True)
    subprocess.run(list(map(str, command)), env=env, cwd=cwd, check=True)


def clean_environment():
    env = {key: value for key, value in os.environ.items()
           if not key.startswith(('CONDA', 'ROS', 'AMENT', 'COLCON', 'CMAKE'))
           and key not in ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH', 'PKG_CONFIG_PATH',
                           'CC', 'CXX', 'CFLAGS', 'CXXFLAGS', 'LDFLAGS', 'CPATH', 'LIBRARY_PATH')}
    env.update(PATH='/usr/bin:/bin', PYTHONNOUSERSITE='1', ROS_PYTHON_VERSION='3')
    return env


def system_environment():
    result = subprocess.check_output(
        ['/bin/bash', '--noprofile', '--norc', '-c',
         'source /opt/ros/humble/setup.bash; /usr/bin/python3 -c "import os,json; print(json.dumps(dict(os.environ)))"'],
        env=clean_environment(), text=True)
    return json.loads(result)


def extract_sources(work, cache):
    manifest = json.loads((ROOT / 'packaging/ros1-sources.json').read_text())
    sources = work / 'sources'
    sources.mkdir(parents=True, exist_ok=True)
    cache.mkdir(parents=True, exist_ok=True)
    locations = {}
    for name, spec in manifest.items():
        archive = cache / (name + '.tar.gz')
        if not archive.exists():
            print('Downloading', spec['url'], flush=True)
            with urllib.request.urlopen(spec['url'], timeout=60) as response:
                archive.write_bytes(response.read())
        if hashlib.sha256(archive.read_bytes()).hexdigest() != spec['sha256']:
            raise RuntimeError(f'Source checksum mismatch: {archive}')
        destination = sources / name
        if not destination.exists():
            destination.mkdir()
            with tarfile.open(archive) as tar:
                # Upstream test fixtures contain escaping symlinks; no fixture
                # links are required to build these packages. Validate every path.
                for member in tar.getmembers():
                    if member.issym() or member.islnk():
                        continue
                    target = (destination / member.name).resolve()
                    if not target.is_relative_to(destination.resolve()):
                        raise RuntimeError(f'Unsafe archive path: {member.name}')
                    tar.extract(member, destination)
        locations[name] = next(path for path in destination.iterdir() if path.is_dir())
    # Only generate C++ and Python bindings; other language backends are unused.
    generation = locations['message_generation']
    cmake = generation / 'CMakeLists.txt'
    cmake.write_text(cmake.read_text().replace('gencpp geneus gennodejs genlisp genmsg genpy', 'gencpp genmsg genpy'))
    xml = generation / 'package.xml'
    text = xml.read_text()
    for backend in ('geneus', 'gennodejs', 'genlisp'):
        text = text.replace(f'  <run_depend>{backend}</run_depend>\n', '')
    xml.write_text(text)
    return locations


def collect_notices(locations, licenses):
    # Several ROS 1 repositories carry their BSD terms in source headers rather
    # than a top-level LICENSE. Retain those notices with the compiled binaries.
    notices = {}
    for name, location in locations.items():
        for file in location.rglob('*'):
            if not file.is_file() or file.suffix not in ('.cpp', '.h', '.hpp'):
                continue
            lines = []
            for line in file.read_text(errors='replace').splitlines():
                if line.strip() and not line.lstrip().startswith(('//', '/*', '*')):
                    break
                lines.append(line)
            block = '\n'.join(lines).strip()
            if 'copyright' in block.lower() and ('redistribution' in block.lower() or 'license' in block.lower()):
                notices.setdefault(block, name + '/' + str(file.relative_to(location)))
    (licenses / 'UPSTREAM_NOTICES.txt').write_text('\n\n'.join(
        origin + '\n' + notice for notice, origin in notices.items()) + '\n')


def build(args):
    if platform.machine() != 'x86_64' or platform.python_version_tuple()[:2] != ('3', '10'):
        raise RuntimeError('Build with /usr/bin/python3 on Ubuntu 22.04 x86_64 (Python 3.10)')
    if 'VERSION_ID="22.04"' not in Path('/etc/os-release').read_text():
        raise RuntimeError('This external-Humble package targets Ubuntu 22.04')
    prefix, work = args.prefix.resolve(), args.work.resolve()
    if prefix == Path('/opt/ros/humble') or prefix == ROOT or prefix == Path('/'):
        raise RuntimeError('Choose a dedicated writable package prefix')
    work.mkdir(parents=True, exist_ok=True)
    private = work / 'ros1-install'
    locations = extract_sources(work, args.cache.resolve())
    workspace = work / 'ros1'
    source = workspace / 'src'
    source.mkdir(parents=True, exist_ok=True)
    for repository, directories in PACKAGES.items():
        for directory in directories:
            package = locations[repository] / directory
            name = repository if directory == '.' else Path(directory).name
            link = source / name
            if link.is_symlink() and link.resolve() != package.resolve():
                link.unlink()
            if not link.exists():
                link.symlink_to(package, target_is_directory=True)
    for name in ('intera_core_msgs', 'intera_motion_msgs'):
        link = source / name
        if not link.exists():
            link.symlink_to(ROOT / 'interfaces/ros1' / name, target_is_directory=True)
    top = source / 'CMakeLists.txt'
    if not top.exists():
        top.symlink_to(locations['catkin'] / 'cmake/toplevel.cmake')
    env = clean_environment()
    env.update(ROS_DISTRO='noetic', ROS_VERSION='1')
    run(['/usr/bin/cmake', '-S', source, '-B', workspace / 'build',
         '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_INSTALL_PREFIX=' + str(private),
         '-DCATKIN_DEVEL_PREFIX=' + str(workspace / 'devel'),
         '-DPYTHON_EXECUTABLE=/usr/bin/python3', '-DCATKIN_ENABLE_TESTING=OFF',
         '-DCMAKE_C_COMPILER=/usr/bin/cc', '-DCMAKE_CXX_COMPILER=/usr/bin/c++',
         '-DROSCONSOLE_BACKEND=print'], env)
    run(['/usr/bin/cmake', '--build', workspace / 'build', '--parallel', args.jobs], env)
    run(['/usr/bin/cmake', '--install', workspace / 'build'], env)
    # This pure-Python master is used only by the explicitly invoked localhost
    # self-test; normal operation always connects to Sawyer's onboard master.
    shutil.copytree(locations['ros_comm'] / 'tools/rosmaster/src/rosmaster',
                    private / 'lib/python3/dist-packages/rosmaster', dirs_exist_ok=True)
    # Keep copyright/license notices for every bundled source repository.
    licenses = prefix / 'share/sawyer_bridge/licenses'
    licenses.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / 'LICENSE', licenses / 'project-LICENSE')
    for name, location in locations.items():
        for notice in location.rglob('*'):
            if notice.is_file() and notice.name.lower().startswith(('license', 'copying')):
                destination = licenses / name / notice.relative_to(location)
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(notice, destination)
    for name, location in [('intera_common', ROOT / 'interfaces/ros1'),
                           ('ros1_bridge', ROOT / 'src/sawyer_bridge')]:
        for notice in location.glob('*'):
            if notice.is_file() and notice.name.lower().startswith(('license', 'copying')):
                destination = licenses / name / notice.name
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(notice, destination)
    collect_notices(locations, licenses)
    env = system_environment()
    env['CMAKE_BUILD_PARALLEL_LEVEL'] = str(args.jobs)
    ros2 = ROOT / 'src'
    run(['/usr/bin/colcon', '--log-base', work / 'ros2-log', 'build',
         '--base-paths', ros2 / 'intera_core_msgs', ros2 / 'intera_motion_msgs',
         '--build-base', work / 'ros2-build', '--install-base', prefix, '--merge-install',
         '--parallel-workers', '1', '--cmake-force-configure', '--cmake-args', '-DBUILD_TESTING=OFF',
         '-DCMAKE_BUILD_TYPE=Release', '-DPython3_EXECUTABLE=/usr/bin/python3'], env, work)
    env['AMENT_PREFIX_PATH'] = str(prefix) + ':' + env.get('AMENT_PREFIX_PATH', '')
    env['CMAKE_PREFIX_PATH'] = str(prefix) + ':' + env.get('CMAKE_PREFIX_PATH', '') + ':' + str(private)
    env['PYTHONPATH'] = str(private / 'lib/python3/dist-packages') + ':' + str(prefix / 'local/lib/python3.10/dist-packages') + ':' + env.get('PYTHONPATH', '')
    env['PATH'] = str(private / 'bin') + ':' + env['PATH']
    env['PKG_CONFIG_PATH'] = str(private / 'lib/pkgconfig')
    env['ROS_PACKAGE_PATH'] = str(private / 'share')
    env['SAWYER_ROS1_PYTHONPATH'] = str(private / 'lib/python3/dist-packages')
    env['LD_LIBRARY_PATH'] = str(private / 'lib') + ':' + str(prefix / 'lib') + ':' + env.get('LD_LIBRARY_PATH', '')
    env['SAWYER_BRIDGE_PACKAGES'] = 'builtin_interfaces;std_msgs;geometry_msgs;sensor_msgs;intera_core_msgs;intera_motion_msgs'
    run(['/usr/bin/colcon', '--log-base', work / 'bridge-log', 'build',
         '--base-paths', ROOT / 'src/sawyer_bridge', '--build-base', work / 'bridge-build',
         '--install-base', prefix, '--merge-install', '--parallel-workers', '1',
         '--cmake-force-configure', '--cmake-args', '-DBUILD_TESTING=OFF', '-DCMAKE_BUILD_TYPE=Release',
         '-DPython3_EXECUTABLE=/usr/bin/python3',
         '-Dintera_core_msgs_DIR=' + str(prefix / 'share/intera_core_msgs/cmake'),
         '-Dintera_motion_msgs_DIR=' + str(prefix / 'share/intera_motion_msgs/cmake'),
         '-Dxmlrpcpp_DIR=' + str(private / 'share/xmlrpcpp/cmake'),
         '-DCMAKE_C_COMPILER=/usr/bin/cc', '-DCMAKE_CXX_COMPILER=/usr/bin/c++'], env, work)
    # Ship only the C++ runtime libraries, not ROS 1 tools, Python or build headers.
    runtime = prefix / 'libexec/sawyer/ros1/lib'
    runtime.mkdir(parents=True, exist_ok=True)
    for name in ('cpp_common', 'rostime', 'roscpp_serialization', 'rosconsole',
                 'rosconsole_backend_interface', 'rosconsole_print', 'roscpp', 'xmlrpcpp'):
        shutil.copy2(private / 'lib' / ('lib' + name + '.so'), runtime)
    share = prefix / 'share/sawyer_bridge'
    (share / 'package-build.json').write_text(json.dumps({
        'platform': 'ubuntu-22.04-linux-64', 'ros2': '/opt/ros/humble', 'python': '/usr/bin/python3',
        'private_ros1_sources': json.loads((ROOT / 'packaging/ros1-sources.json').read_text()),
    }, indent=2) + '\n')
    print('Built package prefix:', prefix, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--work', type=Path, default=ROOT / '.runtime/clean-build')
    parser.add_argument('--cache', type=Path, default=ROOT / '.runtime/pkg-sources')
    parser.add_argument('--jobs', type=int, default=2)
    build(parser.parse_args())
