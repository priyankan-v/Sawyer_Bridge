"""Planning model preparation must preserve geometry and tighten hardware limits."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('prepare_moveit_model', ROOT / 'planning/prepare_model.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
adapter_spec = importlib.util.spec_from_file_location('state_adapter', ROOT / 'planning/state_adapter.py')
adapter = importlib.util.module_from_spec(adapter_spec)
adapter_spec.loader.exec_module(adapter)


def fixture(directory):
    model = ET.Element('robot', name='test_sawyer')

    def link(name):
        element = ET.SubElement(model, 'link', name=name)
        ET.SubElement(ET.SubElement(ET.SubElement(element, 'collision'), 'geometry'), 'sphere', radius='.01')
        return element

    link('base')
    parent = 'base'
    for index, name in enumerate(module.ARM):
        child = f'right_l{index}'
        link(child)
        joint = ET.SubElement(model, 'joint', name=name, type='revolute')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=child)
        ET.SubElement(joint, 'limit', lower='-1', upper='1', velocity='1', effort='1')
        parent = child
    for name in ('right_gripper_base', 'right_connector_plate_base', 'right_electric_gripper_base',
                 'right_gripper_tip', 'right_gripper_l_finger', 'right_gripper_r_finger'):
        link(name)
        joint = ET.SubElement(model, 'joint', name=name + '_joint', type='fixed')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=name)
        parent = name
    limits = dict(joint_names=list(module.ARM), position_lower=[-.9] * 7, position_upper=[.8] * 7,
                  velocity=[.3] * 7, accel=[.2] * 7)
    (directory / 'source.urdf').write_text(ET.tostring(model, encoding='unicode'))
    (directory / 'live_limits.json').write_text(json.dumps(limits))
    return model, limits


class MoveItModelTests(unittest.TestCase):
    def test_gripper_feedback_uses_half_aperture_and_arm_values_are_preserved(self):
        values = dict(right_j6=-4.714662109375, right_gripper_l_finger_joint=.04055091964285715)
        converted = adapter.model_positions(values)
        self.assertEqual(converted['right_j6'], values['right_j6'])
        self.assertEqual(converted['right_gripper_l_finger_joint'], .020275459821428575)
        self.assertEqual(values['right_gripper_l_finger_joint'], .04055091964285715)
        with self.assertRaises(ValueError):
            adapter.model_positions(dict(right_j0=float('nan')))

    def test_model_preserves_geometry_and_uses_intersection_without_widening(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            source, _ = fixture(directory)
            manifest = module.prepare(directory)
            prepared = ET.parse(directory / 'sawyer.urdf').getroot()
            self.assertEqual(len(source.findall('.//collision')), len(prepared.findall('.//collision')))
            for joint in prepared.findall('joint')[:7]:
                self.assertEqual(joint.find('limit').get('lower'), '-0.9')
                self.assertEqual(joint.find('limit').get('upper'), '0.8')
                self.assertEqual(joint.find('limit').get('velocity'), '0.15')
            self.assertFalse(manifest['execution_enabled'])
            semantic = ET.parse(directory / 'sawyer.srdf').getroot()
            pairs = {(entry.get('link1'), entry.get('link2')) for entry in semantic.findall('disable_collisions')}
            self.assertNotIn(('right_l0', 'right_l6'), pairs)
            self.assertEqual(semantic.find("group[@name='right_arm']/chain").get('tip_link'), 'right_gripper_tip')

    def test_missing_tool_collision_geometry_fails(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            model, _ = fixture(directory)
            finger = model.find("link[@name='right_gripper_l_finger']")
            finger.remove(finger.find('collision'))
            (directory / 'source.urdf').write_text(ET.tostring(model, encoding='unicode'))
            with self.assertRaisesRegex(ValueError, 'Missing gripper collision'):
                module.prepare(directory)

    def test_invalid_or_conflicting_live_limits_fail(self):
        for value in (float('nan'), 2.):
            with self.subTest(value=value), tempfile.TemporaryDirectory() as temp:
                directory = Path(temp)
                _, limits = fixture(directory)
                limits['position_lower'][0] = value
                (directory / 'live_limits.json').write_text(json.dumps(limits))
                with self.assertRaises(ValueError):
                    module.prepare(directory)

    def test_mesh_resolution_is_local_and_missing_packages_fail(self):
        for uri, valid in [('package://sawyer_description/meshes/sawyer_pv/base.DAE', True),
                           ('package://unknown/mesh.stl', False),
                           ('package://sawyer_description/../../README.md', False)]:
            with self.subTest(uri=uri), tempfile.TemporaryDirectory() as temp:
                directory = Path(temp)
                model, _ = fixture(directory)
                visual = ET.SubElement(model.find('link'), 'visual')
                ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh', filename=uri)
                (directory / 'source.urdf').write_text(ET.tostring(model, encoding='unicode'))
                if valid:
                    module.prepare(directory)
                    filename = ET.parse(directory / 'sawyer.urdf').find('.//mesh').get('filename')
                    self.assertTrue(filename.startswith('file:///'))
                else:
                    with self.assertRaises(ValueError):
                        module.prepare(directory)


if __name__ == '__main__':
    unittest.main()
