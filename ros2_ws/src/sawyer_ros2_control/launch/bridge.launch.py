"""Configure ROS 1 parameters, then start the existing C++ bridge in isolation."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def start(context):
    root = Path(os.environ['SAWYER_ROOT'])
    config = LaunchConfiguration('config').perform(context)
    command = [str(root / 'scripts/env.sh'), 'ros1', 'python',
               str(root / 'scripts/configure_bridge.py'), config]
    for name in ('commands', 'head_commands', 'cameras'):
        value = LaunchConfiguration(name).perform(context).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        if value == 'true':
            command.append('--' + name.replace('_', '-'))
    configure = ExecuteProcess(cmd=command, output='screen')
    bridge = ExecuteProcess(cmd=[str(root / 'scripts/env.sh'), 'bridge', 'ros2', 'run',
                                 'ros1_bridge', 'parameter_bridge', '/sawyer_bridge/topics',
                                 '/sawyer_bridge/services_1_to_2', '/sawyer_bridge/services_2_to_1',
                                 '__name:=sawyer_selective_bridge'], output='screen')

    def configured(event, _context):
        if event.returncode != 0:
            raise RuntimeError('ROS 1 bridge configuration failed; bridge was not started.')
        return [bridge]

    def bridge_exited(event, context):
        if event.returncode != 0 and not context.is_shutdown:
            raise RuntimeError(f'Bridge exited with status {event.returncode}')
        return [EmitEvent(event=Shutdown(reason='Bridge exited'))]

    return [RegisterEventHandler(OnProcessExit(target_action=configure, on_exit=configured)),
            RegisterEventHandler(OnProcessExit(target_action=bridge, on_exit=bridge_exited)),
            configure]


def generate_launch_description():
    root = Path(os.environ['SAWYER_ROOT'])
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(root / 'config/sawyer_bridge.yaml')),
        DeclareLaunchArgument('commands', default_value='false'),
        DeclareLaunchArgument('head_commands', default_value='false'),
        DeclareLaunchArgument('cameras', default_value='false',
                              description='Forward existing head/hand images without controlling cameras'),
        OpaqueFunction(function=start),
    ])
