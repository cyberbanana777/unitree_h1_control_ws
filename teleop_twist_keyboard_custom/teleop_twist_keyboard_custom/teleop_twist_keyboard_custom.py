#!/usr/bin/env python3

# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# Modifications by Alice Zenina and Alexander Grachev from RTU MIREA (Russia):
# - Ported from ROS1 to ROS2
# - Added Y-axis control support
# - Added speed limits (min/max)
# - Improved parameter handling
# - Code structure optimization

'''
АННОТАЦИЯ
Реализует телеоперационное управление роботом через клавиатуру, 
публикуя сообщения geometry_msgs/msg/Twist в топик ROS2 cmd_vel. 
Поддерживает линейное/угловое движение, режим страффинга и динамическую регулировку скоростей. 
Зависит от ОС (Windows/Linux) для обработки ввода.

ANNOTATION
Implements teleoperation control for robots via keyboard, 
publishing geometry_msgs/msg/Twist messages in ROS2 topic cmd_vel. Supports linear/angular motion, 
strafing mode, and dynamic speed adjustment. OS-dependent (Windows/Linux) for input handling.
'''

import sys
import threading
import rclpy
import geometry_msgs.msg

# Platform-specific imports for keyboard input
if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

# ====================== CONSTANTS ======================
# Speed parameters (m/s for linear, rad/s for angular)
MAX_LINEAR_SPEED = 1.5
MIN_LINEAR_SPEED = 0.1
MAX_ANGULAR_SPEED = 2.0
MIN_ANGULAR_SPEED = 0.1
DEFAULT_LINEAR_SPEED = 0.5
DEFAULT_ANGULAR_SPEED = 1.0
DEFAULT_Y_SPEED = 0.5
SPEED_INCREMENT = 0.1  # 10% speed adjustment

# Help message for keyboard controls
HELP_MESSAGE = """
Keyboard Teleoperation Node
Publishes Twist/TwistStamped messages based on keyboard input.
Works best with US keyboard layout.
---------------------------------------------------------
Movement Controls:
   u    i    o
   j    k    l
   m    ,    .

Holonomic Mode (strafing) - hold Shift:
---------------------------------------------------------
   U    I    O
   J    K    L
   M    <    >

t : move up (+z)
b : move down (-z)

any other key : stop

Speed Controls:
q/z : increase/decrease all speeds by 10%
w/x : increase/decrease linear speed only
e/c : increase/decrease angular speed only
r/v : increase/decrease y-axis speed only

CTRL-C to quit
"""

# Key bindings for movement control
MOVE_BINDINGS = {
    # Standard movement
    'i': (1, 0, 0, 0),    # forward
    'o': (1, 0, 0, -1),    # forward + right turn
    'j': (0, 0, 0, 1),     # left turn
    'l': (0, 0, 0, -1),    # right turn
    'u': (1, 0, 0, 1),     # forward + left turn
    ',': (-1, 0, 0, 0),    # backward
    '.': (-1, 0, 0, 1),    # backward + left turn
    'm': (-1, 0, 0, -1),   # backward + right turn
    
    # Holonomic movement (with Shift)
    'O': (1, -1, 0, 0),    # forward + left strafe
    'I': (1, 0, 0, 0),     # forward
    'J': (0, 1, 0, 0),     # left strafe
    'L': (0, -1, 0, 0),    # right strafe
    'U': (1, 1, 0, 0),     # forward + right strafe
    '<': (-1, 0, 0, 0),    # backward
    '>': (-1, -1, 0, 0),   # backward + right strafe
    'M': (-1, 1, 0, 0),    # backward + left strafe
    
    # Vertical movement
    't': (0, 0, 1, 0),     # up
    'b': (0, 0, -1, 0),    # down
}

# Key bindings for speed adjustment
SPEED_BINDINGS = {
    'q': (1 + SPEED_INCREMENT, 1 + SPEED_INCREMENT, 1 + SPEED_INCREMENT),  # all +
    'z': (1 - SPEED_INCREMENT, 1 - SPEED_INCREMENT, 1 - SPEED_INCREMENT),  # all -
    'w': (1 + SPEED_INCREMENT, 1, 1),  # linear +
    'x': (1 - SPEED_INCREMENT, 1, 1),  # linear -
    'e': (1, 1 + SPEED_INCREMENT, 1),  # angular +
    'c': (1, 1 - SPEED_INCREMENT, 1),  # angular -
    'r': (1, 1, 1 + SPEED_INCREMENT),  # y-axis +
    'v': (1, 1, 1 - SPEED_INCREMENT),  # y-axis -
}


def clamp(value, min_value, max_value):
    """Constrain a value between minimum and maximum bounds."""
    return max(min(value, max_value), min_value)


def get_key(settings):
    """Get a single key press from the keyboard."""
    if sys.platform == 'win32':
        return msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


def save_terminal_settings():
    """Save the current terminal settings for restoration later."""
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(old_settings):
    """Restore previously saved terminal settings."""
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def format_speeds(linear_speed, angular_speed, y_speed):
    """Format current speeds into a readable string."""
    return (f"Current speeds:\t"
            f"Linear: {round(clamp(linear_speed, MIN_LINEAR_SPEED, MAX_LINEAR_SPEED), 3)} m/s\t"
            f"Angular: {round(clamp(angular_speed, MIN_ANGULAR_SPEED, MAX_ANGULAR_SPEED), 3)} rad/s\t"
            f"Y-axis: {round(clamp(y_speed, MIN_LINEAR_SPEED, MAX_LINEAR_SPEED), 3)} m/s")


def main():
    """Main function to handle keyboard teleoperation."""
    # Initialize terminal settings and ROS2 node
    settings = save_terminal_settings()
    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard_custom')

    # Setup message type based on parameters
    stamped = node.declare_parameter('stamped', False).value
    frame_id = node.declare_parameter('frame_id', '').value
    
    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    TwistMsg = geometry_msgs.msg.TwistStamped if stamped else geometry_msgs.msg.Twist
    pub = node.create_publisher(TwistMsg, 'cmd_vel', 10)

    # Start ROS2 spinner in separate thread
    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.daemon = True
    spinner.start()

    # Initialize movement parameters
    speed = DEFAULT_LINEAR_SPEED
    turn = DEFAULT_ANGULAR_SPEED
    y_speed = DEFAULT_Y_SPEED
    x, y, z, th = 0.0, 0.0, 0.0, 0.0  # Movement components
    status_counter = 0

    # Prepare Twist message
    twist_msg = TwistMsg()
    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    try:
        print(HELP_MESSAGE)
        print(format_speeds(speed, turn, y_speed))
        
        while True:
            key = get_key(settings)
            
            # Handle movement keys
            if key in MOVE_BINDINGS:
                x, y, z, th = MOVE_BINDINGS[key]
                
            # Handle speed adjustment keys
            elif key in SPEED_BINDINGS:
                speed = clamp(speed * SPEED_BINDINGS[key][0], MIN_LINEAR_SPEED, MAX_LINEAR_SPEED)
                turn = clamp(turn * SPEED_BINDINGS[key][1], MIN_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
                y_speed = clamp(y_speed * SPEED_BINDINGS[key][2], MIN_LINEAR_SPEED, MAX_LINEAR_SPEED)

                print(format_speeds(speed, turn, y_speed))
                if status_counter == 14:
                    print(HELP_MESSAGE)
                status_counter = (status_counter + 1) % 15
                
            # Handle stop or exit
            else:
                x, y, z, th = 0.0, 0.0, 0.0, 0.0
                if key == '\x03':  # CTRL-C
                    break

            # Update and publish message
            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()

            twist.linear.x = x * speed
            twist.linear.y = y * y_speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            
            pub.publish(twist_msg)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # Clean up: stop movement and restore terminal
        if stamped:
            twist_msg.header.stamp = node.get_clock().now().to_msg()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        pub.publish(twist_msg)
        rclpy.shutdown()
        spinner.join()
        restore_terminal_settings(settings)


if __name__ == '__main__':
    main()