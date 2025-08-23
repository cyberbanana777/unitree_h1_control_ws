# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



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

    time_step_arg = DeclareLaunchArgument(
        'time_step',
        default_value='0.5',
        description='Time step delay for control of move ',
    )


    # params for nodes
    common_params = {
        'target_topic': LaunchConfiguration('target_topic'),
        'max_joint_velocity': LaunchConfiguration('max_joint_velocity'),
        'target_action': LaunchConfiguration('target_action'),
    }

    slam_params = {
        'time_step_param': LaunchConfiguration('time_step'),\
    }

    # Get paths into dir's with launch-files
    control_launch_dir = os.path.join(
        get_package_share_directory('completed_scripts_control'),
        'launch'
    )

    control_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_launch_dir, 'control_h1_base.launch.py')
        ),
        launch_arguments=common_params.items()
    )

    h1_move_node = Node(
        package="cmd_to_high_level_control_package",
        executable="cmd_to_high_level_control_node",
        parameters=[slam_params],
    )

    return LaunchDescription([
        mode_arg,
        target_topic_arg,
        max_joint_velocity_arg,
        target_action_arg,
        time_step_arg,

        h1_move_node,
        control_base_launch,
    ])