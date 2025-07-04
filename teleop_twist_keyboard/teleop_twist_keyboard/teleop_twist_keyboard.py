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

import sys
import threading

import geometry_msgs.msg
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

# ========== SPEED CONSTANTS ==========
MAX_LINEAR_SPEED = 1.5      # m/s
MIN_LINEAR_SPEED = 0.1     # m/s
MAX_ANGULAR_SPEED = 2.0     # rad/s
MIN_ANGULAR_SPEED = 0.1     # rad/s
DEFAULT_LINEAR_SPEED = 0.5  # m/s
DEFAULT_ANGULAR_SPEED = 1.0 # rad/s
DEFAULT_Y_SPEED = 0.5       # m/s
SPEED_INCREMENT = 0.1       # 10%

msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
r/v : increase/decrease only y-axis speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1 + SPEED_INCREMENT, 1 + SPEED_INCREMENT, 1 + SPEED_INCREMENT),
    'z': (1 - SPEED_INCREMENT, 1 - SPEED_INCREMENT, 1 - SPEED_INCREMENT),
    'w': (1 + SPEED_INCREMENT, 1, 1),
    'x': (1 - SPEED_INCREMENT, 1, 1),
    'e': (1, 1 + SPEED_INCREMENT, 1),
    'c': (1, 1 - SPEED_INCREMENT, 1),
    'r': (1, 1, 1 + SPEED_INCREMENT),
    'v': (1, 1, 1 - SPEED_INCREMENT),
}


def clamp(value, min_value, max_value):
    """Ограничение значения в заданных пределах"""
    return max(min(value, max_value), min_value)


def getKey(settings):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn, y_speed):
    return 'currently:\tspeed %s\tturn %s \ty_speed %s' % (
        round(clamp(speed, MIN_LINEAR_SPEED, MAX_LINEAR_SPEED), 3),
        round(clamp(turn, MIN_ANGULAR_SPEED, MAX_ANGULAR_SPEED), 3),
        round(clamp(y_speed, MIN_LINEAR_SPEED, MAX_LINEAR_SPEED), 3)
    )


def main():
    settings = saveTerminalSettings()
    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')

    # parameters
    stamped = node.declare_parameter('stamped', False).value
    frame_id = node.declare_parameter('frame_id', '').value
    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        TwistMsg = geometry_msgs.msg.TwistStamped
    else:
        TwistMsg = geometry_msgs.msg.Twist

    pub = node.create_publisher(TwistMsg, 'cmd_vel', 10)
    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = DEFAULT_LINEAR_SPEED
    turn = DEFAULT_ANGULAR_SPEED
    y_speed = DEFAULT_Y_SPEED
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    twist_msg = TwistMsg()

    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    try:
        print(msg)
        print(vels(speed, turn, y_speed))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = clamp(
                    speed * speedBindings[key][0],
                    MIN_LINEAR_SPEED,
                    MAX_LINEAR_SPEED
                )
                turn = clamp(
                    turn * speedBindings[key][1],
                    MIN_ANGULAR_SPEED,
                    MAX_ANGULAR_SPEED
                )
                y_speed = clamp(
                    y_speed * speedBindings[key][2],
                    MIN_LINEAR_SPEED,
                    MAX_LINEAR_SPEED
                )

                print(vels(speed, turn, y_speed))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

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
        print(e)

    finally:
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
        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()