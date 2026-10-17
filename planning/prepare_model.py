#!/usr/bin/env python3
"""Prepare a local MoveIt model from a read-only controller export."""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ARM = tuple(f'right_j{i}' for i in range(7))
PACKAGES = {'sawyer_description': ROOT / 'sawyer_robot/sawyer_description',
            'intera_tools_description': ROOT / 'intera_common/intera_tools_description'}


def number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'Expected a finite number: {value!r}')
    return float(value)


def prepare(directory):
    source = (directory / 'source.urdf').read_text()
    model = ET.fromstring(source)
    limits = json.loads((directory / 'live_limits.json').read_text())
    joints = {j.get('name'): j for j in model.findall('joint')}
    links = {link.get('name'): link for link in model.findall('link')}
    if len(joints) != len(model.findall('joint')) or len(links) != len(model.findall('link')):
        raise ValueError('Duplicate URDF joint/link names')
    if not {'base', 'right_gripper_tip', 'right_gripper_base', 'right_connector_plate_base',
            'right_electric_gripper_base', 'right_gripper_l_finger', 'right_gripper_r_finger'}.issubset(links):
        raise ValueError('Export must include the installed electric gripper and TCP')
    for name in ('right_electric_gripper_base', 'right_gripper_l_finger', 'right_gripper_r_finger'):
        if links[name].find('collision') is None:
            raise ValueError(f'Missing gripper collision geometry: {name}')
    names = limits['joint_names']
    if len(names) != len(set(names)) or not set(ARM).issubset(names):
        raise ValueError('Missing or duplicate controller arm limits')
    for field in ('position_lower', 'position_upper', 'velocity', 'accel'):
        if len(limits[field]) != len(names):
            raise ValueError(f'Malformed controller {field}')
    planning_limits = {}
    for name in ARM:
        joint = joints[name]
        if joint.get('type') != 'revolute':
            raise ValueError(f'Unexpected arm joint type: {name}')
        limit = joint.find('limit')
        index = names.index(name)
        lower = max(float(limit.get('lower')), number(limits['position_lower'][index]))
        upper = min(float(limit.get('upper')), number(limits['position_upper'][index]))
        velocity = min(float(limit.get('velocity')), number(limits['velocity'][index]), .15)
        acceleration = min(number(limits['accel'][index]), .3)
        if lower >= upper or velocity <= 0 or acceleration <= 0:
            raise ValueError(f'Invalid intersected limits: {name}')
        limit.set('lower', str(lower))
        limit.set('upper', str(upper))
        limit.set('velocity', str(velocity))
        planning_limits[name] = dict(has_velocity_limits=True, max_velocity=velocity,
                                    has_acceleration_limits=True, max_acceleration=acceleration)
    for mesh in model.findall('.//mesh'):
        uri = mesh.get('filename', '')
        if not uri.startswith('package://'):
            raise ValueError(f'Unexpected mesh URI: {uri}')
        package, relative = uri[len('package://'):].split('/', 1)
        if package not in PACKAGES:
            raise ValueError(f'Unmapped mesh package: {package}')
        base = PACKAGES[package].resolve()
        path = (base / relative).resolve()
        if not path.is_relative_to(base) or not path.is_file():
            raise ValueError(f'Missing or invalid mesh: {uri}')
        mesh.set('filename', path.as_uri())

    semantic = ET.Element('robot', name=model.get('name'))
    arm = ET.SubElement(semantic, 'group', name='right_arm')
    ET.SubElement(arm, 'chain', base_link='base', tip_link='right_gripper_tip')
    gripper = ET.SubElement(semantic, 'group', name='right_gripper')
    descendants = {'right_connector_plate_base'}
    while True:
        children = {j.find('child').get('link') for j in joints.values() if j.find('parent').get('link') in descendants}
        if children.issubset(descendants):
            break
        descendants.update(children)
    for link in sorted(descendants):
        ET.SubElement(gripper, 'link', name=link)
    ET.SubElement(semantic, 'end_effector', name='right_gripper', parent_link='right_gripper_base',
                  group='right_gripper', parent_group='right_arm')

    # Only structural exclusions: rigidly connected parts and directly adjacent
    # links. Never infer "never collides" from a handful of sampled poses.
    parent = {link: link for link in links}

    def find(link):
        while parent[link] != link:
            link = parent[link]
        return link

    for joint in joints.values():
        if joint.get('type') == 'fixed':
            parent[find(joint.find('child').get('link'))] = find(joint.find('parent').get('link'))
    disabled = {}
    for first, second in itertools.combinations(sorted(links), 2):
        if links[first].find('collision') is not None and links[second].find('collision') is not None and find(first) == find(second):
            disabled[first, second] = 'RigidlyConnected'
    for joint in joints.values():
        first, second = sorted((joint.find('parent').get('link'), joint.find('child').get('link')))
        disabled[first, second] = 'Adjacent'
    upstream = json.loads((ROOT / 'planning/collision_exclusions.json').read_text())
    for pair in upstream['pairs']:
        if pair['link1'] in links and pair['link2'] in links:
            disabled[tuple(sorted((pair['link1'], pair['link2'])))] = pair['reason']
    for (first, second), reason in sorted(disabled.items()):
        ET.SubElement(semantic, 'disable_collisions', link1=first, link2=second, reason=reason)
    ET.indent(model)
    ET.indent(semantic)
    (directory / 'sawyer.urdf').write_text(ET.tostring(model, encoding='unicode') + '\n')
    (directory / 'sawyer.srdf').write_text(ET.tostring(semantic, encoding='unicode') + '\n')
    (directory / 'planning_limits.json').write_text(json.dumps(dict(joint_limits=planning_limits), indent=2) + '\n')
    manifest = dict(source_sha256=hashlib.sha256(source.encode()).hexdigest(),
        robot=model.get('name'), group='right_arm', tip='right_gripper_tip',
        arm_joints=list(ARM), collision_links=[name for name, link in links.items() if link.find('collision') is not None],
        disabled_collision_pairs=[dict(link1=a, link2=b, reason=r) for (a, b), r in sorted(disabled.items())],
        documented_shoulder_exclusions=upstream['source'],
        documented_finger_contacts=upstream['gripper_source'],
        execution_enabled=False, workspace_obstacles_modelled=False)
    (directory / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=ROOT / '.runtime/moveit_model')
    args = parser.parse_args()
    print(json.dumps(prepare(args.model), indent=2))
