"""Exercise launch configuration without ROS 1 Python or a robot connection."""
import contextlib
import io
import os
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import patch
from xmlrpc.server import SimpleXMLRPCServer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/sawyer_bridge'))
from sawyer_bridge.configure import main


class ConfigurationTest(unittest.TestCase):
    def setUp(self):
        self.parameters = {}
        self.server = SimpleXMLRPCServer(('127.0.0.1', 0), logRequests=False)
        self.uri = 'http://127.0.0.1:' + str(self.server.server_address[1])
        self.server.register_function(lambda caller: [1, '', self.uri], 'getUri')
        def set_param(caller, key, value):
            self.parameters[key] = value
            return [1, '', 0]
        self.server.register_function(set_param, 'setParam')
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.thread.join()
        self.server.server_close()

    def configure(self, *options):
        config = ROOT / 'src/sawyer_bridge/config/sawyer_bridge.yaml'
        with patch.dict(os.environ, ROS_MASTER_URI=self.uri), patch.object(sys, 'argv', ['configure', str(config), *options]), contextlib.redirect_stdout(io.StringIO()):
            return main()

    def test_readonly_defaults_and_optin_commands(self):
        self.assertEqual(self.configure(), 0)
        initial = self.parameters['/sawyer_bridge']
        self.assertTrue(all(not entry['enabled'] for entry in initial['topics'] if entry['direction'] == '2to1'))
        self.assertFalse(any('camera' in entry['topic'] for entry in initial['topics']))
        self.assertEqual(self.configure('--commands', '--head-commands'), 0)
        enabled = {entry['topic'] for entry in self.parameters['/sawyer_bridge']['topics'] if entry['enabled']}
        self.assertIn('/robot/limb/right/joint_command', enabled)
        self.assertIn('/robot/head/command_head_pan', enabled)
        self.assertNotIn('/robot/set_super_enable', enabled)

    def test_master_rejection_is_reported_without_upload(self):
        self.server.register_function(lambda caller: [-1, 'Master unavailable', 0], 'getUri')
        self.assertEqual(self.configure(), 1)
        self.assertEqual(self.parameters, {})


if __name__ == '__main__':
    unittest.main()
