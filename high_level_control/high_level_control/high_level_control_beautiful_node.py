#!/usr/bin/env python3

# Copyright (c) 2016-2024 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")
# All rights reserved.
#
# This file is a modified version of the original Unitree Robotics SDK code,
# redistributed under the terms of the BSD 3-Clause License.
# Modifications (c) 2025 by Alexander Grachev and Alice Zenina from RTU MIREA
# (Russia), licensed under the same BSD 3-Clause License.
# See LICENSE in the project root for full license text.

# MODIFICATIONS:
# - Completely redesigned the interface (Rich library, interactive prompts,
#   command tables).
# - Added new commands (START, STOP_MOVE).
# - Implemented error handling and safe shutdown.
# - Improved code structure (Command, TestOption, UserInterface classes).
# - Added localization (Russian/English).

"""
АННОТАЦИЯ
Данный скрипт запускает high-level клиента для управления роботом Unitree H1 с помощью
high-level команд (демпфирование, готовность, балансирование и т.д.) как с пульта.
Команды вводятся поочерёдно. При вводе "list" выводятся все доступные команды.
Для работы требуется библиотека "unitree_sdk2py".

ANNOTATION
This script launches a high-level client for controlling Unitree H1 robot using
high-level commands (damping, readiness, balancing, etc.) like a remote control.
Commands are entered sequentially. Enter "list" to display all available commands.
Requires "unitree_sdk2py" python library.
"""

import sys
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from rich import print
from rich.console import Console
from rich.panel import Panel
from rich.prompt import Prompt
from rich.style import Style
from rich.table import Table
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient

# Interface settings
INTERFACE = "wlp0s20f3"
DOMAIN_ID = 0
console = Console()


class Command(Enum):
    """Enumeration of available commands"""

    DAMP = (0, "damp", "Switch to damping mode")
    STAND_UP = (1, "stand_up", "Stand in default position")
    START = (2, "start", "Start movement")
    MOVE_FORWARD = (3, "move forward", "Move forward")
    MOVE_LATERAL = (4, "move lateral", "Lateral movement")
    MOVE_ROTATE = (5, "move rotate", "Rotate in place")
    LOW_STAND = (6, "low stand", "Low stance")
    HIGH_STAND = (7, "high stand", "High stance")
    ZERO_TORQUE = (8, "zero torque", "Zero torque")
    STOP_MOVE = (9, "stop_move", "Stop movement")

    def __init__(self, id: int, cmd_name: str, description: str):
        self._id = id
        self._cmd_name = cmd_name
        self._description = description

    @property
    def id(self):
        return self._id

    @property
    def name(self):
        return self._cmd_name

    @property
    def description(self):
        return self._description


@dataclass
class TestOption:
    """Class for storing current command"""

    name: Optional[str] = None
    id: Optional[int] = None


def print_welcome():
    """Prints welcome message"""
    print(
        Panel.fit(
            "[bold cyan]Unitree H1 High-Level Control Client[/bold cyan]",
            subtitle="[yellow]Press Ctrl+C to exit[/yellow]",
        )
    )

    warning_style = Style(color="red", bold=True)
    console.print(
        "⚠ [bold red]WARNING:[/bold red] Please ensure there are no obstacles "
        "around the robot while running this example.",
        style=warning_style,
    )


def print_command_table():
    """Prints table of available commands"""
    table = Table(
        title="[bold]Available Commands[/bold]",
        show_header=True,
        header_style="bold magenta",
        row_styles=["none", "none"],  # Alternating row styles
        show_lines=True,  # Show lines between rows
    )

    table.add_column("ID", style="cyan", justify="center")
    table.add_column("Name", style="green")
    table.add_column("Description", style="white")

    for cmd in Command:
        table.add_row(
            str(cmd.id),
            cmd.name,
            cmd.description,
            end_section=True,  # Add line after row
        )

    console.print(table)


def initialize_network(interface: str = DOMAIN_ID, domain_id: int = DOMAIN_ID):
    """Initializes network connection"""
    if interface:
        ChannelFactoryInitialize(domain_id, interface)
    else:
        ChannelFactoryInitialize(domain_id)


class UserInterface:
    def __init__(self):
        self.test_option = TestOption()

    def convert_to_int(self, input_str: str) -> Optional[int]:
        """Attempts to convert string to integer"""
        try:
            return int(input_str)
        except ValueError:
            return None

    def handle_input(self) -> bool:
        """Handles user input"""
        input_str = (
            Prompt.ask(
                "[bold cyan]Enter command ID/name or 'list'/'exit'[/bold cyan]",
                default="list",
            )
            .lower()
            .strip()
        )

        if input_str == "exit":
            return False

        if input_str == "list":
            print_command_table()
            return True

        # Search for command by ID or name
        for cmd in Command:
            if (
                input_str == cmd.name
                or self.convert_to_int(input_str) == cmd.id
            ):
                self.test_option.name = cmd.name
                self.test_option.id = cmd.id
                console.print(
                    f"[green]✓ Selected:[/green] [bold]{cmd.name}[/bold] "
                    f"(ID: {cmd.id}) - {cmd.description}"
                )
                return True

        console.print(
            "[red]✖ Error:[/red] Unknown command. "
            "Type 'list' to see available commands."
        )
        return True


def execute_command(client: LocoClient, option: TestOption):
    """Executes selected command"""
    if option.id is None:
        return

    try:
        if option.id == Command.DAMP.id:
            client.Damp()
        elif option.id == Command.STAND_UP.id:
            client.StandUp()
        elif option.id == Command.START.id:
            client.Start()
        elif option.id == Command.MOVE_FORWARD.id:
            client.Move(0.3, 0, 0)
        elif option.id == Command.MOVE_LATERAL.id:
            client.Move(0, 0.3, 0)
        elif option.id == Command.MOVE_ROTATE.id:
            client.Move(0, 0, 0.3)
        elif option.id == Command.LOW_STAND.id:
            client.LowStand()
        elif option.id == Command.HIGH_STAND.id:
            client.HighStand()
        elif option.id == Command.ZERO_TORQUE.id:
            client.ZeroTorque()
        elif option.id == Command.STOP_MOVE.id:
            client.StopMove()

        time.sleep(1)
    except Exception as e:
        console.print(f"[red]✖ Command execution error:[/red] {e}")


def main():
    """Main program function"""
    print_welcome()
    print_command_table()

    # Initialize network
    initialize_network(INTERFACE if len(sys.argv) > 1 else None, DOMAIN_ID)

    # Initialize client
    sport_client = LocoClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    ui = UserInterface()

    try:
        while ui.handle_input():
            execute_command(sport_client, ui.test_option)
    except KeyboardInterrupt:
        console.print("\n[yellow]ℹ Shutting down...[/yellow]")
    except Exception as e:
        console.print(f"[red]✖ Fatal error:[/red] {e}")
    finally:
        console.print("[green]✔ Client stopped[/green]")


if __name__ == "__main__":
    main()
