#!/usr/bin/env python3
"""Bring up the Aerostack2 autonomy stack for a single drone.

Equivalent to the basics_robotics_functions, behaviors, mission_execution
and mission_monitoring windows of the aerostack2.yaml tmuxinator layout
(rosbag and MicroXRCEAgent are intentionally left out). The platform / PX4
bridge is launched separately via px4_ros_bridge.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def resolve_plugin_config_file(context):
    """Default motion_controller_plugin_config_file to config/<plugin>.yaml."""
    if LaunchConfiguration('motion_controller_plugin_config_file').perform(context) != '':
        return
    plugin_name = LaunchConfiguration('motion_controller_plugin').perform(context)
    pkg_config = os.path.join(get_package_share_directory('as2_bringup'), 'config')
    context.launch_configurations['motion_controller_plugin_config_file'] = os.path.join(
        pkg_config, f'{plugin_name}.yaml')


def generate_launch_description():
    """Entrypoint."""
    pkg_bringup = get_package_share_directory('as2_bringup')
    default_config_file = os.path.join(pkg_bringup, 'config', 'config.yaml')
    default_controller_config_file = os.path.join(pkg_bringup, 'config', 'pid_speed_controller.yaml')
    default_tree = os.path.join(pkg_bringup, 'trees', 'square.xml')

    namespace = LaunchConfiguration('namespace')
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Drone namespace'),
        DeclareLaunchArgument(
            'motion_controller_plugin',
            default_value='pid_speed_controller',
            description='Motion controller plugin: '
                        'pid_speed_controller or differential_flatness_controller'),
        DeclareLaunchArgument(
            'motion_controller_plugin_config_file',
            default_value='',
            description='Motion controller plugin config file. '
                        'If empty, defaults to config/<motion_controller_plugin>.yaml'),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Shared drone configuration file'),
        DeclareLaunchArgument(
            'tree',
            default_value=default_tree,
            description='Path to behavior tree XML'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        OpaqueFunction(function=resolve_plugin_config_file),

        # Each include is wrapped in a scoped GroupAction so that launch
        # arguments the included file declares under its own name (e.g.
        # 'plugin_name', shared by both the motion controller and the
        # trajectory generator) don't leak into sibling includes.

        # State estimator
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('as2_state_estimator'),
                    'launch', 'state_estimator_launch.py')),
                launch_arguments={
                    'namespace': namespace,
                    'config_file': config_file,
                    'use_sim_time': use_sim_time,
                }.items()
            ),
        ]),

        # Motion controller
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('as2_motion_controller'),
                    'launch', 'controller_launch.py')),
                launch_arguments={
                    'namespace': namespace,
                    'plugin_name': LaunchConfiguration('motion_controller_plugin'),
                    'plugin_config_file': LaunchConfiguration(
                        'motion_controller_plugin_config_file'),
                    'use_sim_time': use_sim_time,
                }.items()
            ),
        ]),

        # Motion behaviors (takeoff, land, go_to, follow_path)
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('as2_behaviors_motion'),
                    'launch', 'motion_behaviors_launch.py')),
                launch_arguments={
                    'namespace': namespace,
                    'config_file': config_file,
                    'use_sim_time': use_sim_time,
                }.items()
            ),
        ]),

        # Trajectory generation behavior
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('as2_behaviors_trajectory_generation'),
                    'generate_polynomial_trajectory_behavior', 'launch',
                    'generate_polynomial_trajectory_behavior_launch.py')),
                launch_arguments={
                    'namespace': namespace,
                    'config_file': config_file,
                    'use_sim_time': use_sim_time,
                }.items()
            ),
        ]),

        # Behavior tree (mission execution)
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('as2_behavior_tree'),
                    'launch', 'behavior_trees.launch.py')),
                launch_arguments={
                    'drone_id': namespace,
                    'tree': LaunchConfiguration('tree'),
                    'use_sim_time': use_sim_time,
                }.items()
            ),
        ]),

        # Mission monitoring (alphanumeric viewer)
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('as2_alphanumeric_viewer'),
                    'launch', 'alphanumeric_viewer_launch.py')),
                launch_arguments={
                    'drone_id': namespace,
                    'use_sim_time': use_sim_time,
                }.items()
            ),
        ]),
    ])
