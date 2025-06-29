from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="low_level_control",
            executable="low_level_control_with_hands"
        ),
        Node(
            package="wrist_control_package",
            executable="wrist_control_node"
        ),
        Node(
            package="hands_init",
            executable="hands_init"   
        )
    ])