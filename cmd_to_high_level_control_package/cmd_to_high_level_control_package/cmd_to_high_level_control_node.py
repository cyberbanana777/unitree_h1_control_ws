#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient

# Constants
FREQUENCY = 100
TIME_STEP = 0.5  # seconds


class CmdToHighLevelControlNode(Node):
    """Node for converting command velocity to high-level control commands."""

    def __init__(self):
        super().__init__("cmd_to_high_level_control_node")

        # Initialize DDS for high_level_control
        ChannelFactoryInitialize(0)

        # Setup locomotion client
        self.sport_client = LocoClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

        # Parameters
        self.declare_parameter("time_step", TIME_STEP)
        self.get_logger().info(
            f"time_step = {self.get_parameter('time_step').value} Hz"
        )

        # State variables
        self.dt = self.get_parameter("time_step").value
        self.last_msg_time = self.get_clock().now()
        self.last_msg = ()

        # Subscribers
        self.subscription = self.create_subscription(
            Twist, "cmd_vel", self.listener_callback, 10
        )

        # Publishers
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Timers
        self.timer = self.create_timer(1.0 / FREQUENCY, self.timer_callback)

        self.subscription  # Prevent unused variable warning
        self.get_logger().info(
            "cmd_to_high_level_control_node has been started"
        )

    def listener_callback(self, msg):
        """Handle incoming velocity commands."""
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        # Update state
        self.last_msg = (vx, vy, omega)
        self.dt = self.get_parameter("time_step").value
        self.sport_client.SetVelocity(vx, vy, omega, self.dt)

        # Record message reception time using ROS 2 clock
        self.last_msg_time = self.get_clock().now()

        # Log colored movement command
        self.get_logger().info(
            f"\033[1;32mMOVEMENT COMMAND:\033[0m "
            f"\033[33mvx={vx:.2f}\033[0m, "
            f"\033[34mvy={vy:.2f}\033[0m, "
            f"\033[35mω={omega:.2f}\033[0m",
            throttle_duration_sec=0.3,
        )

    def timer_callback(self):
        """Handle periodic safety checks and zero-velocity timeouts."""
        now = self.get_clock().now()
        time_since_last_msg = (now - self.last_msg_time).nanoseconds / 1e9

        if time_since_last_msg > self.dt and self.last_msg != (0.0, 0.0, 0.0):
            # Send zero velocity command if timeout reached
            zero_msg = Twist()
            self.publisher.publish(zero_msg)
            self.sport_client.SetVelocity(0.0, 0.0, 0.0, self.dt)
            self.last_msg = (0.0, 0.0, 0.0)  # Update last message


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    node = CmdToHighLevelControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nExiting...")
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
