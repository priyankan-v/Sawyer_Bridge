"""Configure ROS 1 parameters, then start the existing C++ bridge in isolation."""
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def start(context):
    prefix = Path(get_package_prefix('sawyer_bridge'))
    config = LaunchConfiguration('config').perform(context)
    command = [str(prefix / 'lib/sawyer_bridge/configure_bridge'), config]
    for name in ('commands', 'head_commands'):
        value = LaunchConfiguration(name).perform(context).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        if value == 'true':
            command.append('--' + name.replace('_', '-'))
    configure = ExecuteProcess(cmd=command, output='screen')
    bridge = ExecuteProcess(cmd=[str(prefix / 'lib/sawyer_bridge/parameter_bridge'),
                                 '/sawyer_bridge/topics',
                                 '/sawyer_bridge/services_1_to_2', '/sawyer_bridge/services_2_to_1',
                                 '__name:=sawyer_bridge'], output='screen')

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
    share = Path(get_package_share_directory('sawyer_bridge'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config/sawyer_bridge.yaml')),
        DeclareLaunchArgument('commands', default_value='false'),
        DeclareLaunchArgument('head_commands', default_value='false'),
        OpaqueFunction(function=start),
    ])
