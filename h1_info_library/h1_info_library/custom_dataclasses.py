# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

"""
Module for managing robot motor data and displaying joint information.

This module defines data structures for motor information and pose,
and provides a class to manage and display robot joint data based on
configuration (e.g., teleoperation mode or vendor defaults).
"""

from dataclasses import dataclass
from typing import Tuple

from rich.console import Console
from rich.style import Style
from rich.table import Table



from h1_info_library.limits import (LIMITS_OF_JOINTS_WITH_HANDS_FOR_TELEOPERATION,
                                    LIMITS_OF_JOINTS_WITH_HANDS_FROM_VENDOR)
from h1_info_library.names_and_indexes import FROM_INDEXES_TO_NAMES, FROM_NAMES_TO_INDEXES


@dataclass
class MotorInfo:
    """
    Dataclass representing static information about a motor/joint.

    Attributes:
        abs_index (int): Absolute index of the motor in the system.
        type (str): Type/category of the motor (e.g., body, finger, wrist).
        name_joint (str): Human-readable name of the joint.
        limits (tuple[float, float]): Min and max allowed pose values (in radians).
        index_in_msg (int): Index of the motor in the communication message.
    """

    abs_index: int
    type_: str
    name_joint: str
    limits: Tuple[float, float]
    index_in_msg: int


@dataclass
class MotorPose:
    """
    Dataclass representing the current and target pose state of a motor.

    Attributes:
        abs_index (int): Absolute index of the motor.
        target_pose (float): Desired pose (radians).
        current_pose (float): Current measured pose (radians).
        temporary_pose (float): Temporary value used during intermediate calculations.
    """

    abs_index: int
    target_pose: float
    current_pose: float
    temporary_pose: float


# Mapping from integer codes to motor type names.
TYPES_OF_MOTOR = {
    0: "default_body_joint",
    1: "finger_joint",
    2: "wrist_joint",
}


class RobotData:
    """
    A class to manage and display information about robot motors.

    This class initializes motor data based on the target action mode
    (e.g., teleoperation or vendor defaults) and whether hands and fingers
    are included. It supports displaying joint information in a formatted table.

    Attributes:
        joints_info (list[MotorInfo]): List of motor metadata.
        joints_pose_status (list[MotorPose]): List of current motor states.
    """

    def __init__(self, target_action = "teleoperation", include_hands_with_fingers = True) -> None:
        """
        Initialize the RobotData instance.

        Args:
            target_action (str): The action mode. Must be 'teleoperation'
                to use teleoperation-specific limits, otherwise uses vendor defaults.
            include_hands_with_fingers (bool): Whether to include finger and wrist joints.

        Raises:
            ValueError: If an invalid joint name is encountered during initialization.
        """
        self.joints_info: list[MotorInfo] = []
        self.joints_pose_status: list[MotorPose] = []

        # Select the appropriate set of joint limits
        if target_action == "teleoperation":
            limits_set = LIMITS_OF_JOINTS_WITH_HANDS_FOR_TELEOPERATION
        else:
            limits_set = LIMITS_OF_JOINTS_WITH_HANDS_FROM_VENDOR

        # Initialize default body joints (0–19)
        base_index = 0
        for i in range(20):
            abs_index = base_index + i
            self.joints_info.append(
                MotorInfo(
                    abs_index=abs_index,
                    type_=TYPES_OF_MOTOR[0],
                    name_joint=FROM_INDEXES_TO_NAMES[abs_index],
                    limits=limits_set[abs_index],
                    index_in_msg=i,
                )
            )
            self.joints_pose_status.append(
                MotorPose(
                    abs_index=abs_index,
                    target_pose=0.0,
                    current_pose=0.0,
                    temporary_pose=0.0,
                )
            )

        # Optionally include fingers and wrists
        if include_hands_with_fingers:
            # Initialize finger joints (20–31)
            base_index = 20
            for i in range(12):
                abs_index = base_index + i
                self.joints_info.append(
                    MotorInfo(
                        abs_index=abs_index,
                        type_=TYPES_OF_MOTOR[1],
                        name_joint=FROM_INDEXES_TO_NAMES[abs_index],
                        limits=limits_set[abs_index],
                        index_in_msg=i,
                    )
                )
                self.joints_pose_status.append(
                    MotorPose(
                        abs_index=abs_index,
                        target_pose=0.0,
                        current_pose=0.0,
                        temporary_pose=0.0,
                    )
                )

            # Initialize wrist joints (32–33)
            base_index = 32
            for i in range(2):
                abs_index = base_index + i
                self.joints_info.append(
                    MotorInfo(
                        abs_index=abs_index,
                        type_=TYPES_OF_MOTOR[2],
                        name_joint=FROM_INDEXES_TO_NAMES[abs_index],
                        limits=limits_set[abs_index],
                        index_in_msg=i,
                    )
                )
                self.joints_pose_status.append(
                    MotorPose(
                        abs_index=abs_index,
                        target_pose=0.0,
                        current_pose=0.0,
                        temporary_pose=0.0,
                    )
                )

    def show_info(self) -> None:
        """
        Display robot motor information in a formatted table.

        The table includes columns for ID, type, joint name, limits, and message index.
        Color coding is applied based on motor type. Summary statistics are also printed.
        """
        console = Console()

        # Create styled table
        table = Table(
            title="[bold]Robot Motors Reference[/]",
            title_style="bold white on blue",
            header_style="bold cyan",
            border_style="bright_blue",
            show_lines=True,
            expand=True,
        )

        # Define columns
        table.add_column("ID", justify="center", style="cyan")
        table.add_column("Type", style="magenta")
        table.add_column("Joint Name", style="green")
        table.add_column("Limits", justify="center")
        table.add_column("Message Index", justify="center")

        # Add rows
        for motor in self.joints_info:
            type_color = {
                "default_body_joint": "bright_cyan",
                "finger_joint": "bright_magenta",
                "wrist_joint": "#ffa000",
            }.get(motor.type_, "white")
            type_style = Style(color=type_color)
            limits_str = f"[{motor.limits[0]:.2f}, {motor.limits[1]:.2f}]"

            table.add_row(
                f"[color(220)]{motor.abs_index}[/]",
                f"[{type_style}]{motor.type_}[/]",
                motor.name_joint,
                limits_str,
                str(motor.index_in_msg),
            )

        # Print table
        console.print(table)

        # Print summary
        console.print(
            f"\n[bold]Total motors:[/] {len(self.joints_info)}",
            style="bold green",
        )

        # Count motors by type
        type_counts = {}
        for motor in self.joints_info:
            type_counts[motor.type_] = type_counts.get(motor.type_, 0) + 1

        console.print("\n[bold]Distribution by types:[/]")
        for motor_type, count in type_counts.items():
            console.print(f"  {motor_type}: {count}", style="bright_blue")

    def get_joint_by_name(self, name_joint: str) -> MotorInfo:
        """
        Retrieve motor information by joint name.

        Args:
            name_joint (str): The name of the joint to look up.

        Returns:
            MotorInfo: The motor information associated with the given name.

        Raises:
            ValueError: If no joint with the given name exists.
        """
        try:
            index = FROM_NAMES_TO_INDEXES[name_joint]
            # Find the MotorInfo object with the matching absolute index
            for motor in self.joints_info:
                if motor.abs_index == index:
                    return motor
            raise ValueError(f"Joint with name '{name_joint}' not found in motor list.")
        except KeyError:
            raise ValueError(f"Joint with name '{name_joint}' not found.")
        
    def get_joint_info_by_index(self, abs_index: int) -> MotorInfo:
        """Returns information about a joint by its absolute index."""
        for joint in self.joints_info:
            if joint.abs_index == abs_index:
                return joint
        raise ValueError(f"Joint with abs_index {abs_index} not found")

    def get_joint_pose_by_index(self, abs_index: int) -> MotorPose:
        """Returns the current pose (state) of the joint by its absolute index."""
        for pose in self.joints_pose_status:
            if pose.abs_index == abs_index:
                return pose
        raise ValueError(f"Pose for joint with abs_index {abs_index} not found")
    
    def update_current_pose(self, abs_index: int, new_value: float) -> None:
        """Updates the current_pose for the joint with the specified abs_index."""
        joint_pose = next((p for p in self.joints_pose_status if p.abs_index == abs_index), None)
        if joint_pose is None:
            raise ValueError(f"Joint with index {abs_index} not found")
        joint_pose.current_pose = new_value

    def update_target_pose(self, abs_index: int, new_value: float) -> None:
        """Updates the target_pose for the joint with the specified abs_index."""
        joint_pose = next((p for p in self.joints_pose_status if p.abs_index == abs_index), None)
        if joint_pose is None:
            raise ValueError(f"Joint with index {abs_index} not found")
        joint_pose.target_pose = new_value

    def update_temporary_pose(self, abs_index: int, new_value: float) -> None:
        """Updates the temporary_pose for the joint with the specified abs_index."""
        joint_pose = next((p for p in self.joints_pose_status if p.abs_index == abs_index), None)
        if joint_pose is None:
            raise ValueError(f"Joint with index {abs_index} not found")
        joint_pose.temporary_pose = new_value

    def get_current_pose(self, abs_index: int) -> float:
        """Get the current_pose for the joint with the specified abs_index."""
        joint_pose = next((p for p in self.joints_pose_status if p.abs_index == abs_index), None)
        if joint_pose is None:
            raise ValueError(f"Joint with index {abs_index} not found")
        return joint_pose.current_pose

    def get_target_pose(self, abs_index: int) -> float:
        """Get the target_pose for the joint with the specified abs_index."""
        joint_pose = next((p for p in self.joints_pose_status if p.abs_index == abs_index), None)
        if joint_pose is None:
            raise ValueError(f"Joint with index {abs_index} not found")
        return joint_pose.target_pose

    def get_temporary_pose(self, abs_index: int) -> float:
        """Get the temporary_pose for the joint with the specified abs_index."""
        joint_pose = next((p for p in self.joints_pose_status if p.abs_index == abs_index), None)
        if joint_pose is None:
            raise ValueError(f"Joint with index {abs_index} not found")
        return joint_pose.temporary_pose

if __name__ == "__main__":
    robot = RobotData(
        target_action="teleoperation",
        include_hands_with_fingers=True,
    )
    robot.show_info()
    print(robot.joints_info[0].type_)
    print(robot.get_joint_by_name("left_ankle_joint"))
    # This will raise ValueError:
    # print(robot.get_joint_by_name("left_ankle_joint__"))
