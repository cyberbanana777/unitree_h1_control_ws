#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

"""
АННОТАЦИЯ
Узел wrist_control_node ROS2 управляет двигателями DM4310 через интерфейс CAN.
Он получает целевые параметры (положение, скорость, крутящий момент) через топик wrist/cmds
и публикует состояния двигателя (wrist/states) на частоте 333 Гц. Узел автоматически ограничивает
положение двигателя безопасными диапазонами и работает в режиме MIT.

Подключение инициализируется через адаптер UART-CAN (/dev/ttyACM0). Использует сообщения ROS2:
- MotorCmds для команд
- MotorStates для текущего состояния

Узел корректно завершает работу, отключая двигатели и закрывая соединение.

ANNOTATION
The wrist_control_node ROS2 node controls DM4310 motors via CAN interface.
It receives target parameters (position, speed, torque) via the wrist/cmds topic
and publishes motor states (wrist/states) at 333 Hz. The node automatically limits
motor positions to safe ranges and operates in MIT mode.

Connection is initialized via UART-CAN adapter (/dev/ttyACM0). Uses ROS2 messages:
- MotorCmds for commands
- MotorStates for current state

Node gracefully terminates by disabling motors and closing connection.
"""

import numpy as np
import rclpy
import serial
from rclpy.node import Node
from unitree_go.msg import MotorCmds, MotorState, MotorStates

from .DM_CAN import DM_Motor_Type, Motor, MotorControl

# Constants
TOPIC_CMD = "wrist/cmds"
TOPIC_STATES = "wrist/states"
FREQUENCY = 333  # Hz

# Motor aliases
LEFT_ALIAS = 0
RIGHT_ALIAS = 1

# Position limits [min, max] in radians
# Left limit: counter-clockwise rotation limit (from robot's perspective)
# Right limit: clockwise rotation limit (from robot's perspective)
MOTOR_LIMITS = {
    RIGHT_ALIAS: [-6.0, -0.23],
    LEFT_ALIAS: [-1.1, 4.58],
}


class PubSubNode(Node):
    def __init__(self):
        super().__init__("wrist_control_node")

        # Motor initialization
        self.motor_left = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
        self.motor_right = Motor(DM_Motor_Type.DM4310, 0x02, 0x12)

        # CAN connection setup
        try:
            self.serial_device = serial.Serial(
                "/dev/ttyACM0", 921600, timeout=0.5
            )
            self.motor_controler = MotorControl(self.serial_device)
            self.motor_controler.addMotor(self.motor_left)
            self.motor_controler.addMotor(self.motor_right)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection error: {e}")
            raise

        # Enable motors
        self.motor_controler.enable(self.motor_left)
        self.get_logger().info("Left motor enabled")
        self.motor_controler.enable(self.motor_right)
        self.get_logger().info("Right motor enabled")

        # ROS2 communication setup
        self.subscription = self.create_subscription(
            MotorCmds, TOPIC_CMD, self.listener_callback, 10
        )

        self.publisher = self.create_publisher(MotorStates, TOPIC_STATES, 10)

        # Timer for periodic state publishing
        self.timer = self.create_timer(
            1 / FREQUENCY, self.timer_callback  # Period in seconds
        )

        self.get_logger().info(
            f'Node started: subscribes to "{TOPIC_CMD}", publishes to "{TOPIC_STATES}"'
        )

        # Initial motor commands
        self.motor_controler.controlMIT(
            self.motor_left,
            10.0,  # kp
            2.0,  # kd
            0.0,  # target position
            0.0,  # target velocity
            0.0,  # torque
        )

        self.motor_controler.controlMIT(
            self.motor_right,
            10.0,  # kp
            2.0,  # kd
            -2.0,  # target position
            0.0,  # target velocity
            0.0,  # torque
        )

    def listener_callback(self, msg):
        """Handle incoming motor commands."""
        self.get_logger().debug(f"Received command: {msg}")

        left_motor_cmd = msg.cmds[LEFT_ALIAS]
        right_motor_cmd = msg.cmds[RIGHT_ALIAS]

        # Apply position limits
        target_position_left = np.clip(
            float(left_motor_cmd.q),
            MOTOR_LIMITS[LEFT_ALIAS][0],
            MOTOR_LIMITS[LEFT_ALIAS][1],
        )
        target_position_right = np.clip(
            float(right_motor_cmd.q),
            MOTOR_LIMITS[RIGHT_ALIAS][0],
            MOTOR_LIMITS[RIGHT_ALIAS][1],
        )

        # Send commands to motors
        self.motor_controler.controlMIT(
            self.motor_left,
            float(left_motor_cmd.kp),
            float(left_motor_cmd.kd),
            target_position_left,
            float(left_motor_cmd.dq),
            float(left_motor_cmd.tau),
        )

        self.motor_controler.controlMIT(
            self.motor_right,
            float(right_motor_cmd.kp),
            float(right_motor_cmd.kd),
            target_position_right,
            float(right_motor_cmd.dq),
            float(right_motor_cmd.tau),
        )

    def timer_callback(self):
        """Publish current motor states periodically."""
        motor_states_msg = MotorStates()
        motor_left_state_msg = MotorState()
        motor_right_state_msg = MotorState()

        # Populate left motor state
        motor_left_state_msg.mode = 0x01
        motor_left_state_msg.q = float(self.motor_left.getPosition())
        motor_left_state_msg.dq = float(self.motor_left.getVelocity())
        motor_left_state_msg.tau_est = float(self.motor_left.getTorque())

        # Populate right motor state
        motor_right_state_msg.mode = 0x01
        motor_right_state_msg.q = float(self.motor_right.getPosition())
        motor_right_state_msg.dq = float(self.motor_right.getVelocity())
        motor_right_state_msg.tau_est = float(self.motor_right.getTorque())

        # Publish combined states
        motor_states_msg.states.append(motor_left_state_msg)
        motor_states_msg.states.append(motor_right_state_msg)
        self.publisher.publish(motor_states_msg)

        self.get_logger().debug(f"Published states: {motor_states_msg}")

    def shutdown(self):
        """Gracefully shutdown node by disabling motors and closing connection."""
        self.motor_controler.disable(self.motor_left)
        self.motor_controler.disable(self.motor_right)
        self.serial_device.close()
        self.get_logger().info("Motors disabled, connection closed")


def main(args=None):
    rclpy.init(args=args)
    node = PubSubNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    except Exception as e:
        node.get_logger().error(f"Exception: {str(e)}")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
