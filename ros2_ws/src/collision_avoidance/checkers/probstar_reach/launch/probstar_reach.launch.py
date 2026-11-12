import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Nodes run inside the agent namespace so that sensor_measurements/odom and the
    # relative topics of verifier / agent_comms resolve to this agent's topics.
    namespace = LaunchConfiguration('namespace')
    # Unique numeric agent id (0..255). No default on purpose.
    system_id = ParameterValue(LaunchConfiguration('system_id'), value_type=int)
    go_to_config = os.path.join(
        get_package_share_directory('collision_avoidance_behavior'), 'config', 'go_to_collision_avoidance.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('system_id'),
        DeclareLaunchArgument('dt', default_value='0.1'),
        DeclareLaunchArgument('nsteps', default_value='10'),
        DeclareLaunchArgument('period', default_value='0.1'),
        DeclareLaunchArgument('prob_threshold', default_value='0.5'),
        DeclareLaunchArgument('time_tolerance', default_value='0.01'),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(package='probstar_reach', executable='verifier', name='verifier',
             namespace=namespace, output='screen',
             # Only the verifier's own logger (<namespace>.verifier), not rclpy / message_filters
             ros_arguments=['--log-level', [namespace, '.verifier:=', LaunchConfiguration('log_level')]],
             parameters=[{
                 'SystemID': system_id,
                 'dt': ParameterValue(LaunchConfiguration('dt'), value_type=float),
                 'nsteps': ParameterValue(LaunchConfiguration('nsteps'), value_type=int),
                 'period': ParameterValue(LaunchConfiguration('period'), value_type=float),
                 'prob_threshold': ParameterValue(LaunchConfiguration('prob_threshold'), value_type=float),
                 'time_tolerance': ParameterValue(LaunchConfiguration('time_tolerance'), value_type=float),
             }]),
        # Node name must equal the action name: BehaviorServer puts its _behavior/* services under it
        Node(package='collision_avoidance_behavior', executable='go_to_collision_avoidance_behavior_node',
             name='GoToCollisionAvoidanceBehavior', namespace=namespace, output='screen',
             parameters=[go_to_config]),
        # Relays my_rs <-> received_rs, message type discovered from the verifier's publisher
        Node(package='agent_comms', executable='agent_comms', name='agent_comms',
             namespace=namespace, output='screen',
             parameters=[{'SystemID': system_id}],
             # All drones send and receive on the same radio topic
             remappings=[('radio_in', '/radio_traffic'), ('radio_out', '/radio_traffic')]),
    ])
