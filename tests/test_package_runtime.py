"""Check that packaged ROS 1 helpers stay isolated from system Humble."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from sawyer_ros2_control.runtime import backend_environment, state_directory, transport_report


class PackageRuntimeTests(unittest.TestCase):
    def test_backend_preserves_network_and_removes_ros2_python(self):
        with tempfile.TemporaryDirectory() as directory:
            prefix = Path(directory)
            library = prefix / 'libexec/sawyer/ros1/lib/libroscpp.so'
            library.parent.mkdir(parents=True)
            library.touch()
            with patch.dict(os.environ, {'ROS_MASTER_URI': 'http://127.0.0.1:11421',
                                        'ROS_IP': '127.0.0.1', 'ROS_DOMAIN_ID': '87',
                                        'PYTHONPATH': '/opt/ros/humble/python',
                                        'LD_LIBRARY_PATH': '/unrelated/conda/lib',
                                        'AMENT_PREFIX_PATH': '/opt/ros/humble'}, clear=True):
                environment = backend_environment(prefix)
            self.assertEqual(environment['ROS_MASTER_URI'], 'http://127.0.0.1:11421')
            self.assertEqual(environment['ROS_IP'], '127.0.0.1')
            self.assertNotIn('AMENT_PREFIX_PATH', environment)
            self.assertNotIn('/opt/ros', environment['PYTHONPATH'])
            self.assertNotIn('/unrelated', environment['LD_LIBRARY_PATH'])
            self.assertEqual(environment['ROS_DISTRO'], 'noetic')

    def test_user_state_location_is_separate_from_package(self):
        with patch.dict(os.environ, {'SAWYER_STATE_DIR': '/tmp/sawyer-state'}, clear=True):
            self.assertEqual(state_directory(), Path('/tmp/sawyer-state'))
            self.assertEqual(transport_report(), Path('/tmp/sawyer-state/local_transport.json'))

    def test_missing_private_runtime_is_an_error(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError, 'Missing packaged ROS 1'):
                backend_environment(directory)

    def test_legacy_root_uses_fresh_state_report(self):
        with patch.dict(os.environ, {'SAWYER_ROOT': '/tmp/legacy-checkout'}, clear=True):
            self.assertEqual(transport_report(), Path('/tmp/legacy-checkout/.runtime/local_transport.json'))
