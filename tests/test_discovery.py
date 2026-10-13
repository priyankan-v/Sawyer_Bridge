"""Check discovery selection and non-invoking ROS service type probes."""
import contextlib
import io
from pathlib import Path
import socket
import struct
import sys
import threading
import unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/sawyer_bridge'))
from sawyer_bridge.discovery import discover, read_exact, service_type

class DiscoveryTest(unittest.TestCase):
    def test_service_probe_exchanges_only_headers(self):
        listener = socket.socket(); listener.bind(('127.0.0.1', 0)); listener.listen(1)
        received = []
        def serve():
            with listener.accept()[0] as connection:
                length = struct.unpack('<I', read_exact(connection, 4))[0]
                data = read_exact(connection, length)
                received.append(data)
                field = b'type=intera_core_msgs/SolvePositionIK'
                header = struct.pack('<I', len(field)) + field
                connection.sendall(struct.pack('<I', len(header)) + header)
                connection.settimeout(2); received.append(connection.recv(1))
        thread = threading.Thread(target=serve); thread.start()
        try:
            self.assertEqual(service_type('rosrpc://127.0.0.1:' + str(listener.getsockname()[1]), '/ik'), 'intera_core_msgs/SolvePositionIK')
            thread.join(timeout=3); self.assertFalse(thread.is_alive())
            self.assertIn(b'probe=1', received[0]); self.assertEqual(received[1], b'')
        finally: listener.close()

    def test_intera_topics_and_both_action_directions(self):
        class Master:
            def getSystemState(self, caller):
                return [1, '', [[['/state', ['/robot']], ['/old_bridge', ['/sawyer_bridge']]],
                                [['/command', ['/robot']], ['/forward/goal', ['/robot']], ['/unknown/goal', ['/robot']]], []]]
            def getTopicTypes(self, caller):
                return [1, '', [['/state', 'intera_core_msgs/HeadState'], ['/command', 'intera_core_msgs/JointCommand'],
                                ['/forward/goal', 'intera_motion_msgs/MotionCommandActionGoal'],
                                ['/forward/result', 'intera_motion_msgs/MotionCommandActionResult'],
                                ['/unknown/goal', 'intera_core_msgs/CameraFieldCalibrationCommandActionGoal'],
                                ['/old_bridge', 'intera_core_msgs/HeadState']]]
        config = dict(topics=[], actions=[], services_1_to_2=[], services_2_to_1=[])
        ros2 = ({'/ros2_command': 'intera_core_msgs/msg/JointCommand'}, {},
                {'/ros2_ik': ['intera_core_msgs/srv/SolvePositionIK']},
                {'/reverse': ['intera_core_msgs/action/CalibrationCommand']})
        unsupported = discover(Master(), config, {'intera_core_msgs/HeadState', 'intera_core_msgs/JointCommand'}, ros2=ros2)
        topics = {entry['topic']: entry for entry in config['topics']}
        self.assertEqual(topics['/state']['direction'], '1to2')
        self.assertEqual(topics['/command']['direction'], '2to1')
        self.assertEqual(topics['/ros2_command']['direction'], '2to1')
        self.assertNotIn('/forward/result', topics); self.assertNotIn('/old_bridge', topics)
        self.assertEqual({entry['direction'] for entry in config['actions']}, {'1to2', '2to1'})
        self.assertEqual(config['services_1_to_2'][0]['service'], '/ros2_ik')
        self.assertEqual(config['services_1_to_2'][0]['type'], 'intera_core_msgs/srv/SolvePositionIK')
        self.assertEqual(unsupported, ['intera_core_msgs/action/CameraFieldCalibrationCommand'])

if __name__ == '__main__': unittest.main()
