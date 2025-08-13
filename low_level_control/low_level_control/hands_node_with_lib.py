#!/usr/bin/env python3

"""
АННОТАЦИЯ
ROS 2-узел для управления двумя роботизированными кистями Inspire Hand. 
Принимает командные сообщения MotorCmds для установки углов пальцев, 
публикует состояния MotorStates с углами, скоростями, ускорениями и 
температурами. Ключевые ограничения: жёстко заданный UART-порт
(/dev/ttyUSB0), фиксированная конфигурация (2 кисти, 12 моторов), требует
ОС Linux и специфичного оборудования.

ANNOTATION
ROS 2 node for controlling two Inspire Hand robotic hands. 
Subscribes to MotorCmds messages to set finger angles, 
publishes MotorStates with angles, velocities, accelerations and temperatures. 
Key limitations: hardcoded UART port (/dev/ttyUSB0), fixed configuration 
(2 hands, 12 motors), requires Linux OS and specific hardware.
"""

import rclpy
import numpy as np
from rclpy.node import Node
from inspire_hand import InspireHand
from unitree_go.msg import MotorCmds, MotorState, MotorStates

TOPIC_CMD = "inspire/cmd"
TOPIC_STATES = "inspire/state"
FREQUENCY = 100  # Reduced frequency 100 Hz
NORMALIZE_CONST = 1000
MAX_ANGLE = 1000  # Maximum device angle
MIN_ANGLE = 0  # Minimum device angle


class ControlNode(Node):
    def __init__(self):
        super().__init__("fingers_control_node")

        self.declare_parameter("velocity_limit", 1000)
        self.velocity_limit = self.get_parameter("velocity_limit").value

        # Initialize hands with error handling
        try:
            self.left_hand = InspireHand(port="/dev/ttyUSB0", baudrate=115200, slave_id=1)
            self.right_hand = InspireHand(port="/dev/ttyUSB0", baudrate=115200, slave_id=2)
            self.left_hand.open()
            self.right_hand.open()
            self.left_hand.open_all_fingers()
            self.right_hand.open_all_fingers()
            self.left_hand.set_all_finger_speeds(self.velocity_limit)
            self.right_hand.set_all_finger_speeds(self.velocity_limit)
            self.get_logger().info("Hands initialized")
        except Exception as e:
            self.get_logger().error(f"Hand initialization failed: {str(e)}")
            raise

        # State initialization
        self._last_q = self._read_current_angles()
        self._last_dq = np.zeros(12)
        self.dq = np.zeros(12)
        self.ddq = np.zeros(12)

        # ROS communication setup
        self.subscription = self.create_subscription(
            MotorCmds, TOPIC_CMD, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(MotorStates, TOPIC_STATES, 10)
        self.timer = self.create_timer(1.0 / FREQUENCY, self.timer_callback)
        self.get_logger().info("Node started")

    def _read_current_angles(self):
        """Read current angles with initialization"""
        angles_left = self.left_hand.get_finger_angles() or [0] * 6
        angles_right = self.right_hand.get_finger_angles() or [0] * 6
        return np.array(angles_left + angles_right)

    def _clamp_angle(self, angle):
        """Clamp angle to valid range"""
        return max(MIN_ANGLE, min(angle, MAX_ANGLE))

    def listener_callback(self, msg):
        """Command processing handler"""
        try:
            # Split commands for left and right hands
            left_cmds = [cmd.q * NORMALIZE_CONST for cmd in msg.cmds[:6]]
            right_cmds = [cmd.q * NORMALIZE_CONST for cmd in msg.cmds[6:]]

            # Set angles with boundary checking
            for i, angle in enumerate(left_cmds):
                clamped = self._clamp_angle(int(angle))
                self.left_hand.set_finger_angle(i, clamped)

            for i, angle in enumerate(right_cmds):
                clamped = self._clamp_angle(int(angle))
                self.right_hand.set_finger_angle(i, clamped)

        except Exception as e:
            self.get_logger().error(f"Command error: {str(e)}", throttle_duration_sec=5)

    def timer_callback(self):
        """Periodic state update handler"""
        try:
            # Read current angles
            angles_left = self.left_hand.get_finger_angles() or [0] * 6
            angles_right = self.right_hand.get_finger_angles() or [0] * 6
            current_q = np.array(angles_left + angles_right)

            # Calculate velocity and acceleration
            self.dq = (current_q - self._last_q) / NORMALIZE_CONST * FREQUENCY
            self.ddq = (self.dq - self._last_dq) * FREQUENCY

            # Update previous values
            self._last_q = current_q.copy()
            self._last_dq = self.dq.copy()

            # Read temperatures
            temp_left = self.left_hand.get_finger_temperatures() or [0] * 6
            temp_right = self.right_hand.get_finger_temperatures() or [0] * 6
            temperatures = temp_left + temp_right

            # Create message
            msg = MotorStates()
            for i in range(12):
                state = MotorState()
                state.q = current_q[i] / NORMALIZE_CONST
                state.dq = self.dq[i]
                state.ddq = self.ddq[i]
                state.tau_est = 0.0
                state.temperature = int(temperatures[i])
                state.mode = 0x01
                msg.states.append(state)

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(
                f"State update error: {str(e)}", throttle_duration_sec=5
            )

    def shutdown(self):
        """Safe shutdown procedure"""
        try:
            self.left_hand.open_all_fingers()
            self.right_hand.open_all_fingers()
            self.left_hand.close()
            self.right_hand.close()

            self.get_logger().info("Hands opened")
        except Exception:
            self.get_logger().warning("Error during shutdown")


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    except Exception as e:
        node.get_logger().error(f"Critical error: {str(e)}")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
