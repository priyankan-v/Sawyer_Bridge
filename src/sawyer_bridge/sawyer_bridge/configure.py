"""Load selective bridge parameters using the ROS 1 master's XML-RPC API."""
import argparse
import os
from pathlib import Path
import xmlrpc.client

import yaml


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 5
        return connection


def checked(response):
    code, message, value = response
    if code != 1:
        raise RuntimeError(message)
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', type=Path)
    parser.add_argument('--commands', action='store_true')
    parser.add_argument('--head-commands', action='store_true')
    args = parser.parse_args()
    try:
        uri = os.environ.get('ROS_MASTER_URI')
        if not uri:
            raise RuntimeError('Set ROS_MASTER_URI to the Sawyer ROS master')
        config = yaml.safe_load(args.config.read_text())
        if not isinstance(config, dict) or not isinstance(config.get('topics'), list):
            raise ValueError('Configuration must contain a topics list')
        for entry in config['topics']:
            if args.commands and entry['topic'] in (
                '/robot/limb/right/joint_command', '/robot/limb/right/joint_command_timeout',
                '/robot/limb/right/set_speed_ratio'):
                entry['enabled'] = True
            if args.head_commands and entry['topic'] == '/robot/head/command_head_pan':
                entry['enabled'] = True
        caller = '/configure_sawyer_bridge'
        with xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport()) as master:
            checked(master.getUri(caller))
            # Upload one namespace atomically; the C++ bridge reads its three lists.
            checked(master.setParam(caller, '/sawyer_bridge', {
                key: config.get(key, []) for key in ('topics', 'services_1_to_2', 'services_2_to_1')}))
        print('Configured Sawyer bridge on ' + uri)
        return 0
    except (OSError, ValueError, RuntimeError, KeyError, yaml.YAMLError, xmlrpc.client.Error) as error:
        print('Bridge configuration error: ' + str(error))
        return 1
