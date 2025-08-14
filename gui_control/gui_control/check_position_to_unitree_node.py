#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ChekNode(Node):
    def __init__(self):
        super().__init__("check_position_to_unitree_node")      
        self.get_logger().info("Node started")

        self.subscription_positions_to_unitree = self.create_subscription(
            String, "positions_to_unitree",
            self.listener_callback_positions_to_unitree, 10
        )

    def listener_callback_positions_to_unitree(self, msg):
        print(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = ChekNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping node...")
    except Exception as e:
        node.get_logger().error(f"Node error: {e}")
    finally:
        node.get_logger().info("Node stopped")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

