#!/usr/bin/env python3

'''
АННОТАЦИЯ
Данный скрипт запускает high-level клиента, который позволяет управлять робоом Unitree H1 с помощью 
high-level команд (приеревести робота в состояние демфирования, готовности и балансирования и т.д.) как с пульта. 
Команды поочерёднно вводятся. 
Если ввести "list" - выведутся все возможные команды
Для корректной работы необходим python-библиотека "unitree_sdk2py".

ANNOTATION
This script launches a high-level client that allows you to control the Unitree H1 robot using 
high-level commands (such as damping, readiness, and balancing, etc.) as if it were a remote control. 
The commands are entered one by one. 
If you enter "list," all possible commands will be displayed.
To use this script correctly, you will need the unitree_sdk2py python library.
'''

import sys
import time
from dataclasses import dataclass

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient

INTERFACE = 'wlp0s20f3'

@dataclass
class TestOption:
    name: str
    id: int

option_list = [
    TestOption(name="damp", id=0),         
    TestOption(name="stand_up", id=1),     
    TestOption(name="Start", id=2),
    TestOption(name="move forward", id=3),         
    TestOption(name="move lateral", id=4),    
    TestOption(name="move rotate", id=5),  
    TestOption(name="low stand", id=6),  
    TestOption(name="high stand", id=7),    
    TestOption(name="zero torque", id=8),     
    TestOption(name="Stop_move", id=9)     
]


class UserInterface:
    def __init__(self):
        self.test_option_ = None


    def convert_to_int(self, input_str):
        try:
            return int(input_str)
        except ValueError:
            return None


    def terminal_handle(self):
        input_str = input("Enter id or name: \n")

        if input_str == "list":
            self.test_option_.name = None
            self.test_option_.id = None
            for option in option_list:
                print(f"{option.name}, id: {option.id}")
            return

        for option in option_list:
            if input_str == option.name or self.convert_to_int(input_str) == option.id:
                self.test_option_.name = option.name
                self.test_option_.id = option.id
                print(
                    f"Test: {self.test_option_.name}, "
                    f"test_id: {self.test_option_.id}"
                )
                return

        print("No matching test option found.")


def main():
    print(
        "WARNING: Please ensure there are no obstacles around the robot "
        "while running this example."
    )    
    print("Enter 'list' to list all available functions.")
    input("Press Enter to continue...")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, INTERFACE)
    else:
        ChannelFactoryInitialize(0)


    print("\nAvailable functions:")
    for i, option in enumerate(option_list):
        print(f"{i} - {option.name}, id: {option.id}")
    print()

    test_option = TestOption(name=None, id=None) 
    user_interface = UserInterface()
    user_interface.test_option_ = test_option

    sport_client = LocoClient() 
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    while True:
        try:
            user_interface.terminal_handle()

            print(
                f"Updated Test Option: Name = {test_option.name}, "
                f"ID = {test_option.id}\n"
            )

            if test_option.id == 0:
                sport_client.Damp()
            elif test_option.id == 1:
                sport_client.StandUp()
            elif test_option.id == 2:
                sport_client.Start()
            elif test_option.id == 3:
                sport_client.Move(0.3,0,0)
            elif test_option.id == 4:
                sport_client.Move(0,0.3,0)
            elif test_option.id == 5:
                sport_client.Move(0,0,0.3)
            elif test_option.id == 6:
                sport_client.LowStand()
            elif test_option.id == 7:
                sport_client.HighStand()
            elif test_option.id == 8:
                sport_client.ZeroTorque()
            elif test_option.id == 9:
                sport_client.StopMove()

            time.sleep(1)
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")
            break


if __name__ == "__main__":
    main()
