"""System Humble MoveIt planning and model services; trajectory execution disabled."""
import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ROOT = Path(__file__).resolve().parents[1]


def start(context):
    directory = Path(LaunchConfiguration('model').perform(context)).resolve()
    description = {'robot_description': ParameterValue((directory / 'sawyer.urdf').read_text(), value_type=str)}
    semantic = {'robot_description_semantic': ParameterValue((directory / 'sawyer.srdf').read_text(), value_type=str)}
    relay = ['/usr/bin/python3', str(ROOT / 'planning/joint_state_relay.py'), '--model', str(directory)]
    offline = LaunchConfiguration('offline_state').perform(context)
    if offline:
        relay.extend(['--offline-state', offline])
    parameters = [description, semantic,
        {'robot_description_kinematics': {'right_arm': {
            'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
            'kinematics_solver_search_resolution': .005, 'kinematics_solver_timeout': .1}}},
        {'robot_description_planning': json.loads((directory / 'planning_limits.json').read_text())},
        {'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
         'ompl': {'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization',
            'start_state_max_bounds_error': 0.,
            'planner_configs': {'RRTConnect': {'type': 'geometric::RRTConnect', 'range': 0.}},
            'right_arm': {'default_planner_config': 'RRTConnect', 'planner_configs': ['RRTConnect'],
                          'longest_valid_segment_fraction': .002}},
         'allow_trajectory_execution': False,
         'disable_capabilities': 'move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupExecuteService move_group/MoveGroupPickPlaceAction move_group/MoveGroupMoveAction',
         'publish_robot_description': True, 'publish_robot_description_semantic': True,
         'publish_planning_scene': True, 'publish_geometry_updates': True,
         'publish_state_updates': True, 'publish_transforms_updates': True}]
    return [
        ExecuteProcess(cmd=relay, output='screen'),
        Node(package='moveit_ros_move_group', executable='move_group', namespace='sawyer_planning',
             output='screen', parameters=parameters,
             # Humble 2.5.9 can unload the KDL plugin before its node callback
             # groups are destroyed. Keep this process's plugin mapped until exit.
             additional_env={'LD_PRELOAD': '/opt/ros/humble/lib/libmoveit_kdl_kinematics_plugin.so'},
             remappings=[('joint_states', '/sawyer_planning/joint_states')]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='sawyer_planning', name='robot_state_publisher', output='screen',
             parameters=[description], remappings=[('joint_states', '/sawyer_planning/joint_states')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=str(ROOT / '.runtime/moveit_model')),
        DeclareLaunchArgument('offline_state', default_value='', description='Test-only frozen state; never use for physical execution'),
        OpaqueFunction(function=start),
    ])
