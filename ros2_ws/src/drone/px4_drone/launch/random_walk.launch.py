import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Runs in the drone namespace: registers a PX4 external mode on <ns>/fmu/*, listens to <ns>/collision_detected
    earth_frame = os.path.join(get_package_share_directory('px4_drone'), 'config', 'earth_frame.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('px4_namespace', default_value=''),
        # 0: walk until stopped
        DeclareLaunchArgument('steps', default_value='0'),
        Node(package='px4_drone', executable='random_walk', name='random_walk',
             namespace=LaunchConfiguration('namespace'), output='screen',
             parameters=[earth_frame, {
                 'px4_namespace': LaunchConfiguration('px4_namespace'),
                 'steps': ParameterValue(LaunchConfiguration('steps'), value_type=int),
             }]),
    ])
