from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="low_level_control",
                executable="low_level_control_with_hands_node",
            ),
            Node(
                package="low_level_control",
                executable="wrist_control_node",
            ),
            Node(
                package="low_level_control",
                executable="hands_init_node",
            ),
            Node(
                package="cmd_to_high_level_control_package",
                executable="cmd_to_high_level_control_node",
            ),
        ]
    )
