#!/usr/bin/env python3

"""
АННОТАЦИЯ
Запускает исполняемый файл inspire_hand для работы с манипулятором Inspire Hand
через UART.
Автоматически передает sudo-пароль, обрабатывает вывод процесса и корректно
завершает работу по сигналу Ctrl+C. Используется в ROS 2 пакете low_level_control
для управления роботизированными устройствами. Требует Linux и прав sudo
(пароль указан в коде).

ANNOTATION
It launches the inspire_hand executable file to work with the Inspire Hand manipulator
via UART.
Automatically passes sudo password, handles process output and implements graceful Ctrl+C
termination. Designed for ROS 2 low_level_control package to control robotic devices.
Requires Linux environment and sudo privileges (password hardcoded).
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory


def main():
    """
    Main function for launching and managing the inspire_hand process.
    Handles sudo launch, password passing, and proper Ctrl+C termination.
    """
    package_share_dir = get_package_share_directory("low_level_control")
    executable_path = os.path.join(package_share_dir, "inspire_hand")

    # Command to launch with sudo and device specification
    cmd = ["sudo", "-S", executable_path, "-s", "/dev/ttyUSB0"]

    sudo_password = "Unitree0408\n"  # password with newline

    try:
        # Launch process with I/O handling
        with subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        ) as proc:
            # Send password to stdin
            proc.stdin.write(sudo_password)
            proc.stdin.flush()

            # Output process output in real time
            for line in proc.stdout:
                print(line, end="")

            proc.wait()
            print(f"\nProcess finished with exit code {proc.returncode}")

    except KeyboardInterrupt:
        print("\nReceived Ctrl+C! Terminating process...")
        _terminate_process(proc, sudo_password)


def _terminate_process(proc, sudo_password):
    """Properly terminates process using sudo kill."""
    try:
        kill_cmd = ["sudo", "-S", "kill", "-TERM", str(proc.pid)]

        kill_proc = subprocess.Popen(
            kill_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        out, _ = kill_proc.communicate(sudo_password)

        if kill_proc.returncode == 0:
            print("Process successfully terminated via sudo kill.")
        else:
            print(
                f"Error terminating process, exit code: {kill_proc.returncode}"
            )
            print(out)

    except Exception as e:
        print(f"Error while attempting to terminate process: {e}")


if __name__ == "__main__":
    main()
