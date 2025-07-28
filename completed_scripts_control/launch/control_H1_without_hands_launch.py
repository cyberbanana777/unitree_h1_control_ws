from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="low_level_control",
                executable="low_level_control_without_hands_node",
            ),
        ]
    )
