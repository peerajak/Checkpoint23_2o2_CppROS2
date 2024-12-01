from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tortoisebot_waypoints',
            executable='tortoisebot_waypoint_action_client_node',
            output='screen'),
    ])