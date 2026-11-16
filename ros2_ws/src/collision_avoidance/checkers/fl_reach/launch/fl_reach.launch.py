from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Nodes run inside the drone namespace so that state/pose, state/twist (px4_drone state_estimate)
    # and the relative topics of agent_comms (my_rs, received_rs, ...) resolve to this agent's topics.
    namespace = LaunchConfiguration('namespace')
    # Unique numeric agent id (0..255). No default on purpose.
    system_id = ParameterValue(LaunchConfiguration('system_id'), value_type=int)
    # Lookahead horizon of the reachable set (s)
    reach_time = ParameterValue(LaunchConfiguration('reach_time'), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('system_id'),
        DeclareLaunchArgument('reach_time', default_value='1.0'),
        Node(package='fl_reach', executable='verifier', name='verifier',
             namespace=namespace, output='screen',
             parameters=[{'SystemID': system_id, 'reachTime': reach_time}]),
        Node(package='fl_reach', executable='collision_checker', name='collision_checker',
             namespace=namespace, output='screen'),
        # Relays my_rs <-> received_rs, message type discovered from the verifier's publisher
        Node(package='agent_comms', executable='agent_comms', name='agent_comms',
             namespace=namespace, output='screen',
             parameters=[{'SystemID': system_id}],
             # All drones send and receive on the same radio topic
             remappings=[('radio_in', '/radio_traffic'), ('radio_out', '/radio_traffic')]),
    ])
