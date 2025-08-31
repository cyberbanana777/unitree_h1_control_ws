# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='with_hands',
        description='Режим работы: with_hands или without_hands.',
        choices=['with_hands', 'without_hands']
    )

    target_topic_arg = DeclareLaunchArgument(
        'target_topic',
        default_value='arm_sdk',
        description='Topic for control commands.',
        choices=['arm_sdk', 'lowcmd'],
    )

    max_joint_velocity_arg = DeclareLaunchArgument(
        'max_joint_velocity',
        default_value='4.0',
        description='Maximum joint velocity.',
    )

    target_action_arg = DeclareLaunchArgument(
        'target_action',
        default_value='other',
        description='Target action for control commands.',
        choices=['other', 'teleoperation'],
    )

    controled_by_cmd_vel_arg = DeclareLaunchArgument(
        'controled_by_cmd_vel',
        default_value='False',
        description='Enable robot control by cmd_vel.',
        choices=['True', 'False'],
    )

    time_step_arg = DeclareLaunchArgument(
        'time_step',
        default_value='0.5',
        description='Time step delay for control of move ',
    )

    # Node parameters
    common_params = {
        'target_topic_param': LaunchConfiguration('target_topic'),
        'max_joint_velocity_param': LaunchConfiguration('max_joint_velocity'),
        'target_action_param': LaunchConfiguration('target_action'),
    }

    slam_params = {
        'time_step_param': LaunchConfiguration('time_step'),\
    }

    # Conditions for selecting the mode
    with_hands_condition = PythonExpression([
        '"', LaunchConfiguration('mode'), '" == "with_hands"'
    ])

    without_hands_condition = PythonExpression([
        '"', LaunchConfiguration('mode'), '" == "without_hands"'
    ])

    cmd_vel_condition = PythonExpression([
        '"', LaunchConfiguration('controled_by_cmd_vel'), '" == "True"'
    ])

    # Nodes for the mode with hands
    with_hands_nodes = [
        Node(
            package="low_level_control",
            executable="low_level_control_with_hands_node",
            parameters=[common_params],
            condition=IfCondition(with_hands_condition)
        ),
        Node(
            package="low_level_control",
            executable="wrist_control_node",
            condition=IfCondition(with_hands_condition)
        ),
        Node(
            package="low_level_control",
            executable="hands_init_node",
            condition=IfCondition(with_hands_condition)
        )
    ]

    # Nodes for the mode without hands
    without_hands_node = Node(
        package="low_level_control",
        executable="low_level_control_without_hands_node",
        parameters=[common_params],
        condition=IfCondition(without_hands_condition)
    )

    h1_move_node = Node(
        package="cmd_to_high_level_control_package",
        executable="cmd_to_high_level_control_node",
        parameters=[slam_params],
        condition=IfCondition(cmd_vel_condition),
    )

    return LaunchDescription([
        mode_arg,
        target_topic_arg,
        max_joint_velocity_arg,
        target_action_arg,
        controled_by_cmd_vel_arg,
        time_step_arg,

        *with_hands_nodes,
        without_hands_node,
        h1_move_node,
    ])