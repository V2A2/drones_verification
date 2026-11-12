#!/usr/bin/env python3
"""Bring up the PX4 <-> ROS 2 bridge (the Pixhawk platform node).

Each platform parameter is exposed as its own launch argument instead of
being read from a config file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Entrypoint."""
    default_control_modes_file = os.path.join(
        get_package_share_directory('as2_platform_pixhawk'),
        'config', 'control_modes.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Drone namespace'),
        DeclareLaunchArgument(
            'control_modes_file',
            default_value=default_control_modes_file,
            description='Platform control modes file'),
        DeclareLaunchArgument(
            'cmd_freq',
            default_value='100.0',
            description='Hz of platform commands send'),
        DeclareLaunchArgument(
            'info_freq',
            default_value='10.0',
            description='Hz of platform info publish'),
        DeclareLaunchArgument(
            'max_thrust',
            default_value='29.0',
            description='Max thrust to send to platform (N)'),
        DeclareLaunchArgument(
            'min_thrust',
            default_value='0.15',
            description='Min thrust to send to platform (N)'),
        DeclareLaunchArgument(
            'external_odom',
            default_value='false',
            description='Flag to use external odometry source'),
        DeclareLaunchArgument(
            'fmu_prefix',
            default_value='',
            description='Prefix for FMU topics'),
        DeclareLaunchArgument(
            'target_system_id',
            default_value='1',
            description='Target system ID for mavros'),

        Node(
            package='as2_platform_pixhawk',
            executable='as2_platform_pixhawk_node',
            name='platform',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,
            parameters=[{
                'control_modes_file': LaunchConfiguration('control_modes_file'),
                'cmd_freq': LaunchConfiguration('cmd_freq'),
                'info_freq': LaunchConfiguration('info_freq'),
                'max_thrust': LaunchConfiguration('max_thrust'),
                'min_thrust': LaunchConfiguration('min_thrust'),
                'external_odom': LaunchConfiguration('external_odom'),
                'fmu_prefix': LaunchConfiguration('fmu_prefix'),
                'target_system_id': LaunchConfiguration('target_system_id'),
            }]
        ),
    ])
