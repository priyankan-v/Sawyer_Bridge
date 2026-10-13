"""Discover Intera endpoints; service probes exchange headers, never call services."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import socket
import struct
import time
from urllib.parse import urlparse
import xmlrpc.client

PACKAGES = ('intera_core_msgs', 'intera_motion_msgs')
ACTIONS = {'intera_core_msgs/action/CalibrationCommand', 'intera_motion_msgs/action/MotionCommand'}
SERVICES = {'intera_core_msgs/IOComponentCommandSrv', 'intera_core_msgs/SolvePositionIK', 'intera_core_msgs/SolvePositionFK'}


def checked(response):
    if response[0] != 1:
        raise RuntimeError(response[1])
    return response[2]


def read_exact(connection, length):
    data = b''
    while len(data) < length:
        chunk = connection.recv(length - len(data))
        if not chunk:
            raise OSError('Connection closed during ROS header')
        data += chunk
    return data


def service_type(uri, name):
    address = urlparse(uri)
    if address.scheme != 'rosrpc' or not address.hostname or not address.port:
        raise ValueError('Invalid ROS service URI: ' + uri)
    fields = ['callerid=/sawyer_bridge_discovery', 'service=' + name, 'md5sum=*', 'probe=1']
    body = b''.join(struct.pack('<I', len(field.encode())) + field.encode() for field in fields)
    with socket.create_connection((address.hostname, address.port), timeout=2) as connection:
        connection.sendall(struct.pack('<I', len(body)) + body)
        length = struct.unpack('<I', read_exact(connection, 4))[0]
        if length > 1048576:
            raise ValueError('ROS service header exceeds 1 MiB')
        data = read_exact(connection, length)
    header = {}
    while data:
        if len(data) < 4:
            raise ValueError('Truncated ROS service header')
        length = struct.unpack('<I', data[:4])[0]
        if length > len(data) - 4:
            raise ValueError('Truncated ROS service field')
        key, value = data[4:4 + length].decode().split('=', 1)
        header[key] = value
        data = data[4 + length:]
    if 'error' in header:
        raise RuntimeError(header['error'])
    return header['type']


def ros2_graph():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.action import get_action_server_names_and_types_by_node
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('sawyer_bridge_discovery', context=context)
    executor = SingleThreadedExecutor(context=context)
    pubs, subs, services, actions = {}, {}, {}, {}
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, executor=executor, timeout_sec=.1)
        for name, types in node.get_topic_names_and_types():
            for typ in types:
                if not typ.startswith(tuple(package + '/msg/' for package in PACKAGES)):
                    continue
                if any(info.node_name != 'sawyer_bridge' for info in node.get_publishers_info_by_topic(name)):
                    pubs[name] = typ
                if any(info.node_name != 'sawyer_bridge' for info in node.get_subscriptions_info_by_topic(name)):
                    subs[name] = typ
        for peer, namespace in node.get_node_names_and_namespaces():
            if peer in ('sawyer_bridge', node.get_name()):
                continue
            try:
                services.update(node.get_service_names_and_types_by_node(peer, namespace))
                actions.update(get_action_server_names_and_types_by_node(node, peer, namespace))
            except rclpy.node.NodeNameNonExistentError:
                continue  # The peer left the graph during this snapshot.
        return pubs, subs, services, actions
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


def discover(master, config, supported_messages, ros2=None):
    caller = '/sawyer_bridge_discovery'
    publishers, subscribers, services = checked(master.getSystemState(caller))
    topic_types = dict(checked(master.getTopicTypes(caller)))
    pubs = {name for name, nodes in publishers if any(node != '/sawyer_bridge' for node in nodes)}
    subs = {name for name, nodes in subscribers if any(node != '/sawyer_bridge' for node in nodes)}
    topics = {entry['topic']: dict(entry) for entry in config['topics']}
    actions = {entry['action']: dict(entry) for entry in config.get('actions', [])}
    unsupported = set()
    for name, typ in topic_types.items():
        if not typ.startswith(tuple(package + '/' for package in PACKAGES)):
            continue
        if typ.endswith('ActionGoal') and name in subs:
            action = typ.replace('/', '/action/', 1)[:-len('ActionGoal')]
            if action in ACTIONS:
                actions.setdefault(name.rsplit('/', 1)[0], dict(action=name.rsplit('/', 1)[0], type=action, direction='2to1', enabled=True))
            else:
                unsupported.add(action)
            continue
        if typ.endswith(('ActionGoal', 'ActionFeedback', 'ActionResult')):
            continue  # Action protocol handled as a lifecycle, never generic topic forwarding.
        if typ not in supported_messages:
            unsupported.add(typ)
            continue
        if name not in pubs and name not in subs:
            continue
        direction = 'bidirectional' if name in pubs and name in subs else '1to2' if name in pubs else '2to1'
        entry = topics.setdefault(name, dict(topic=name, queue_size=5))
        entry.update(type=typ.replace('/', '/msg/', 1), direction=direction, enabled=True)
    resolved = []
    for name, nodes in services:
        if any(node != '/sawyer_bridge' for node in nodes):
            resolved.append((name, checked(master.lookupService(caller, name))))
    def probe(item):
        name, uri = item
        try:
            return name, service_type(uri, name), None
        except (OSError, ValueError, KeyError, RuntimeError) as error:
            return name, None, str(error)
    forward = {entry['service']: dict(entry) for entry in config.get('services_2_to_1', [])}
    with ThreadPoolExecutor(max_workers=4) as pool:
        for name, typ, error in pool.map(probe, resolved):
            if error:
                print('Discovery warning: could not probe service ' + name + ': ' + error)
            elif typ in SERVICES:
                forward.setdefault(name, dict(service=name, type=typ, package=''))
            elif typ.startswith(tuple(package + '/' for package in PACKAGES)):
                unsupported.add(typ)
    pubs2, subs2, services2, actions2 = ros2 if ros2 is not None else ros2_graph()
    for name in set(pubs2) | set(subs2):
        typ = pubs2.get(name, subs2.get(name))
        ros1_type = typ.replace('/msg/', '/')
        if ros1_type not in supported_messages:
            unsupported.add(typ)
            continue
        entry = topics.setdefault(name, dict(topic=name, type=typ, queue_size=5, enabled=True))
        needs1 = name in pubs or name in subs2
        needs2 = name in subs or name in pubs2
        entry.update(enabled=True, direction='bidirectional' if needs1 and needs2 else '1to2' if needs1 else '2to1')
    reverse = {entry['service']: dict(entry) for entry in config.get('services_1_to_2', [])}
    for name, types in services2.items():
        for typ in types:
            typ1 = typ.replace('/srv/', '/')
            if typ1 in SERVICES and name not in forward:
                reverse.setdefault(name, dict(service=name, type=typ, package=''))
    for name, types in actions2.items():
        for typ in types:
            if typ in ACTIONS:
                actions.setdefault(name, dict(action=name, type=typ, direction='1to2', enabled=True))
            elif typ.startswith(tuple(package + '/action/' for package in PACKAGES)):
                unsupported.add(typ)
    config.update(topics=list(topics.values()), services_2_to_1=list(forward.values()),
                  services_1_to_2=list(reverse.values()), actions=list(actions.values()))
    return sorted(unsupported)


def supported_messages():
    from ament_index_python.packages import get_package_share_directory
    return {package + '/' + file.stem for package in PACKAGES
            for file in (Path(get_package_share_directory(package)) / 'msg').glob('*.msg')}
