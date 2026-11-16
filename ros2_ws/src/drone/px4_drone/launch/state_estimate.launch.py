import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Runs in the drone namespace: reads <ns>/fmu/out/*, publishes <ns>/state/pose and <ns>/state/twist
    earth_frame = os.path.join(get_package_share_directory('px4_drone'), 'config', 'earth_frame.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        # Prefix of the fmu topics, empty = relative to the namespace (PX4_UXRCE_DDS_NS = namespace)
        DeclareLaunchArgument('px4_namespace', default_value=''),
        Node(package='px4_drone', executable='state_estimate', name='state_estimate',
             namespace=LaunchConfiguration('namespace'), output='screen',
             parameters=[earth_frame, {'px4_namespace': LaunchConfiguration('px4_namespace')}]),
    ])
