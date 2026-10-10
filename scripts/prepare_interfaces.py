#!/usr/bin/env python3
"""Reproduce ROS 2 definitions and explicit bridge mappings from original sources."""
import hashlib
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILTINS = set('bool byte char float32 float64 int8 uint8 int16 uint16 int32 uint32 int64 uint64 string'.split())


def snake(name):
    name = re.sub(r'(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', name).lower()


def convert(text, package):
    result, fields = [], [[]]
    for line in text.splitlines():
        code, sep, comment = line.partition('#')
        if code.strip() == '---':
            fields.append([])
        match = re.match(r'^(\s*)(\S+)\s+(\w+)(.*)$', code)
        if match and code.strip() != '---':
            indent, typ, name, tail = match.groups()
            base = typ.split('[')[0]
            suffix = typ[len(base):]
            base = {'Header': 'std_msgs/Header', 'time': 'builtin_interfaces/Time',
                    'duration': 'builtin_interfaces/Duration'}.get(base, base)
            if base not in BUILTINS and '/' not in base:
                base = package + '/' + base
            new_name = name if '=' in tail else snake(name)
            if '=' not in tail:
                fields[-1].append((name, new_name))
            code = f'{indent}{base}{suffix} {new_name}{tail}'
        result.append(code.strip() + (sep + comment if sep else ''))
    return '\n'.join(result) + '\n', fields


def main():
    manifest = []
    (ROOT / 'ros1_ws/src').mkdir(parents=True, exist_ok=True)
    for package in ('intera_core_msgs', 'intera_motion_msgs'):
        source = ROOT / 'intera_common' / package
        if not source.is_dir():
            raise SystemExit(f'Missing original repository: {source}')
        link = ROOT / 'ros1_ws/src' / package
        if not link.exists():
            link.symlink_to(Path('../../intera_common') / package)
        target = ROOT / 'ros2_ws/src' / package
        target.mkdir(parents=True, exist_ok=True)
        files, mappings = [], []
        for kind in ('msg', 'srv', 'action'):
            for path in sorted((source / kind).glob('*.' + kind)):
                relative = path.relative_to(source)
                dest = target / relative
                dest.parent.mkdir(exist_ok=True)
                converted, fields = convert(path.read_text(), package)
                dest.write_text(converted)
                files.append(str(relative))
                manifest.append({'source': str(path.relative_to(ROOT)),
                                 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                                 'ros2': str(dest.relative_to(ROOT)),
                                 'renamed': {a: b for group in fields for a, b in group if a != b}})
                if kind == 'msg' and any(a != b for a, b in fields[0]):
                    mappings.extend([f'- ros1_package_name: {package}', f'  ros1_message_name: {path.stem}',
                                     f'  ros2_package_name: {package}', f'  ros2_message_name: {path.stem}',
                                     '  fields_1_to_2:'])
                    mappings.extend(f'    {a}: {b}' for a, b in fields[0])
                if kind == 'srv' and any(a != b for group in fields for a, b in group):
                    mappings.extend([f'- ros1_package_name: {package}', f'  ros1_service_name: {path.stem}',
                                     f'  ros2_package_name: {package}', f'  ros2_service_name: {path.stem}'])
                    for direction, group in zip(('request', 'response'), fields):
                        if group:
                            mappings.append(f'  {direction}_fields_1_to_2:')
                            mappings.extend(f'    {a}: {b}' for a, b in group)
        deps = ['std_msgs', 'geometry_msgs', 'sensor_msgs', 'builtin_interfaces', 'action_msgs']
        if package == 'intera_motion_msgs':
            deps.append('intera_core_msgs')
        (target / 'bridge_mapping.yaml').write_text('\n'.join(mappings) + '\n' if mappings else '[]\n')
        (target / 'package.xml').write_text(f'''<?xml version="1.0"?>
<package format="3">
  <name>{package}</name><version>5.3.0</version>
  <description>ROS 2 port of original Rethink Intera interfaces for ros1_bridge.</description>
  <maintainer email="maintainer@example.com">Sawyer workstation maintainers</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
{chr(10).join('  <depend>' + dep + '</depend>' for dep in deps)}
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export><build_type>ament_cmake</build_type>
    <ros1_bridge mapping_rules="bridge_mapping.yaml"/>
  </export>
</package>
''')
        (target / 'CMakeLists.txt').write_text(f'''cmake_minimum_required(VERSION 3.16)
project({package})
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
{chr(10).join('find_package(' + dep + ' REQUIRED)' for dep in deps)}
rosidl_generate_interfaces(${{PROJECT_NAME}}
{chr(10).join('  "' + file + '"' for file in files)}
  DEPENDENCIES {' '.join(deps)})
install(FILES bridge_mapping.yaml DESTINATION share/${{PROJECT_NAME}})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
''')
    (ROOT / 'docs').mkdir(exist_ok=True)
    (ROOT / 'docs/interface_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(f'Prepared {len(manifest)} original interfaces without changing ROS 1 sources.')


if __name__ == '__main__':
    main()
