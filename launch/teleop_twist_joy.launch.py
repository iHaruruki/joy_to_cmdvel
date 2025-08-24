from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
        ),
        Node(
            package='joy_to_cmdvel',
            executable='joy_to_cmdvel_node',
            output='screen',
        )
    ])