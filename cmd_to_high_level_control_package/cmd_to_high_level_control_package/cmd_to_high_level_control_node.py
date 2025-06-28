#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient


FREQUENCY = 20

class CmdToHighLevelControlNode(Node):
    def __init__(self):
        super().__init__('cmd_to_high_level_control_node')
        
        # DDS init for high_level_control
        ChannelFactoryInitialize(0)
        
        self.sport_client = LocoClient() 
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

        self.declare_parameter('frequency', FREQUENCY)
        self.get_logger().info(
            f"Frequency = {self.get_parameter('frequency').value} Hz"
        )

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('cmd_to_high_level_control node has been started')


    def listener_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z
        dt = 1 / self.get_parameter('frequency').value
        self.sport_client.SetVelocity(vx, vy, omega, dt)
        print(f"\033[1;32mMOVEMENT COMMAND:\033[0m \033[33mvx={vx:.2f}\033[0m, \033[34mvy={vy:.2f}\033[0m, \033[35mω={omega:.2f}\033[0m")


def main(args=None):
    rclpy.init(args=args)
    cmd_to_high_level_control_node = CmdToHighLevelControlNode()
    try:
        rclpy.spin(cmd_to_high_level_control_node)
    except KeyboardInterrupt:
        cmd_to_high_level_control_node.get_logger().info("\nExiting...")
        
    except Exception as e:
        cmd_to_high_level_control_node.get_logger().error(f"Error: {e}")
        
    finally:
        # Destroy the node explicitly
        cmd_to_high_level_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()