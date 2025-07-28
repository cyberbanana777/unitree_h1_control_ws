from dataclasses import dataclass

from rich.console import Console
from rich.style import Style
from rich.table import Table

from .limits import *
from .names_and_indexes import *


@dataclass
class MotorInfo:
    abs_index: int
    type: str
    name_joint: str
    limits: tuple
    index_in_msg: int


@dataclass
class MotorPose:
    abs_index: int
    target_pose: float
    current_pose: float
    temporary_pose: float


TYPES_OF_MOTOR = {0: "default_body_joint", 1: "finger_joint", 2: "wrist_joint"}


class RobotData:
    def __init__(self, target_action: str, include_hands_with_fingers: bool):
        self._var_runner = 0
        self.joints_info = []
        self.joints_pose_status = []

        if target_action == "teleoperation":
            limits_set = LIMITS_OF_JOINTS_WITH_HANDS_FOR_TELEOPERATION
        else:
            limits_set = LIMITS_OF_JOINTS_WITH_HANDS_FROM_VENDOR

        # Initialize default joints
        for i in range(20):
            self.joints_info.append(
                MotorInfo(
                    abs_index=(self._var_runner + i),
                    type=TYPES_OF_MOTOR[0],
                    name_joint=FROM_INDEXES_TO_NAMES[self._var_runner + i],
                    limits=limits_set[self._var_runner + i],
                    index_in_msg=i,
                )
            )

            self.joints_pose_status.append(
                MotorPose(
                    abs_index=(self._var_runner + i),
                    target_pose=0.0,
                    current_pose=0.0,
                    temporary_pose=0.0,
                )
            )

        # Additional parts (hands and fingers)
        if include_hands_with_fingers:
            # Initialize fingers
            self._var_runner = 20
            for i in range(12):
                self.joints_info.append(
                    MotorInfo(
                        abs_index=(self._var_runner + i),
                        type=TYPES_OF_MOTOR[1],
                        name_joint=FROM_INDEXES_TO_NAMES[self._var_runner + i],
                        limits=limits_set[self._var_runner + i],
                        index_in_msg=i,
                    )
                )

                self.joints_pose_status.append(
                    MotorPose(
                        abs_index=(self._var_runner + i),
                        target_pose=0.0,
                        current_pose=0.0,
                        temporary_pose=0.0,
                    )
                )

            # Initialize wrists
            self._var_runner = 32
            for i in range(2):
                self.joints_info.append(
                    MotorInfo(
                        abs_index=(self._var_runner + i),
                        type=TYPES_OF_MOTOR[2],
                        name_joint=FROM_INDEXES_TO_NAMES[self._var_runner + i],
                        limits=limits_set[self._var_runner + i],
                        index_in_msg=i,
                    )
                )

                self.joints_pose_status.append(
                    MotorPose(
                        abs_index=(self._var_runner + i),
                        target_pose=0.0,
                        current_pose=0.0,
                        temporary_pose=0.0,
                    )
                )
        del self._var_runner

    def show_info(self):
        console = Console()

        # Create table with style settings
        table = Table(
            title="[bold]Robot Motors Reference[/]",
            title_style="bold white on blue",
            header_style="bold cyan",
            border_style="bright_blue",
            show_lines=True,
            expand=True,
        )

        # Add columns
        table.add_column("ID", justify="center", style="cyan")
        table.add_column("Type", style="magenta")
        table.add_column("Joint Name", style="green")
        table.add_column("Limits", justify="center")
        table.add_column("Message Index", justify="center")

        # Fill table with data
        for motor in self.joints_info:
            # Determine color based on motor type
            type_style = Style(
                color={
                    "default_body_joint": "bright_cyan",
                    "finger_joint": "bright_magenta",
                    "wrist_joint": "#ffa000",
                }.get(motor.type, "white")
            )

            # Format limits
            limits_str = f"[{motor.limits[0]:.2f}, {motor.limits[1]:.2f}]"

            table.add_row(
                f"[color(220)]{str(motor.abs_index)}[/]",
                f"[{type_style}]{motor.type}[/]",
                motor.name_joint,
                limits_str,
                str(motor.index_in_msg),
            )

        # Print the table
        console.print(table)

        # Additional statistics
        console.print(
            f"\n[bold]Total motors:[/] {len(self.joints_info)}",
            style="bold green",
        )

        # Group by motor types
        type_counts = {}
        for motor in self.joints_info:
            type_counts[motor.type] = type_counts.get(motor.type, 0) + 1

        console.print("\n[bold]Distribution by types:[/]")
        for motor_type, count in type_counts.items():
            console.print(f"  {motor_type}: {count}", style="bright_blue")

    def get_joint_by_name(self, name_joint: str) -> MotorInfo:
        """Get info about joint by name"""
        try:
            index = FROM_NAMES_TO_INDEXES[name_joint]
            return index
        except Exception:
            raise ValueError(f"Joint with name '{name_joint}' not found")


if __name__ == "__main__":
    robot = RobotData(
        target_action="teleoperation", include_hands_with_fingers=True
    )
    robot.show_info()
    print(robot.joints_info[0].type)
    print(robot.get_joint_by_name("left_ankle_joint"))
    print(robot.get_joint_by_name("left_ankle_joint__"))
