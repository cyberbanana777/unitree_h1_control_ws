#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
Реализует низкоуровневый ROS 2 узел для управления роботом Unitree H1, 
включая руки c пальцами и запястьями. Основные функции: обработка целевых поз
из JSON-сообщений через топик positions_to_unitree, плавное изменение 
скорости суставов с ограничением максимального угла поворота за цикл, 
контроль температуры приводов. Ключевые технологии: ROS 2 (rclpy), 
Unitree SDK, CRC-проверка команд. Важные ограничения: жестко заданные 
PID-коэффициенты, зависимость от проприетарных библиотек unitree_sdk2py 
и h1_info_library.

ANNOTATION
Implements a low-level ROS 2 node for controlling Unitree H1 robot's arms with
fingers and wrists. Core functionality: processing target poses from JSON 
messages via positions_to_unitree topic, smooth joint velocity ramping with 
per-cycle angle delta limits, motor temperature monitoring. Key technologies: 
ROS 2 (rclpy), Unitree SDK, CRC command validation. Critical constraints: 
hardcoded PID coefficients, dependency on proprietary unitree_sdk2py 
and h1_info_library.
'''

import json
import time
import math
import numpy as np

import rclpy
import h1_info_library as h1
from rclpy.node import Node
from std_msgs.msg import Float32, String
from unitree_go.msg import (
    LowCmd, LowState, 
    MotorCmd, MotorCmds,
    MotorState, MotorStates
)
from unitree_sdk2py.utils.crc import CRC


# Node configuration constants
FREQUENCY = 333.33                       # Hz
START_JOINT_VELOCITY = 0.5               # Initial joint velocity multiplier
MAX_JOINT_VELOCITY = 7.0                 # Max joint velocity
MAX_FINGER_VELOCITY = 0.6                # Max finger velocity
TARGET_TOPIC = "arm_sdk"                 # Default target topic
TIME_TO_CHANGE_VELOCITY = 20.0            # Seconds to reach max velocity
TARGET_ACTION = 'teleoperation'          # Default robot action mode
WRIST_SCALE = 5                          # Wrist velocity scaling factor


class LowLevelControlNode(Node):
    def __init__(self):
        super().__init__("low_level_control_with_hands_node")

        # Initialize parameters
        self.declare_parameter("target_action", TARGET_ACTION)
        self.target_action_param = self.get_parameter("target_action").value

        # Initialize robot data
        self.robot = h1.RobotData(
            target_action=self.target_action_param,
            include_hands_with_fingers=True
        )

        # Define active joints
        self.active_joints_H1 = [
            
            # arms
            h1.FROM_NAMES_TO_INDEXES["right_shoulder_roll_joint"],
            h1.FROM_NAMES_TO_INDEXES["right_shoulder_pitch_joint"],
            h1.FROM_NAMES_TO_INDEXES["right_shoulder_yaw_joint"],
            h1.FROM_NAMES_TO_INDEXES["right_elbow_joint"],
            h1.FROM_NAMES_TO_INDEXES["left_shoulder_pitch_joint"],
            h1.FROM_NAMES_TO_INDEXES["left_shoulder_roll_joint"],
            h1.FROM_NAMES_TO_INDEXES["left_shoulder_yaw_joint"],
            h1.FROM_NAMES_TO_INDEXES["left_elbow_joint"],
            h1.FROM_NAMES_TO_INDEXES["torso_joint"],

            # legs
            h1.FROM_NAMES_TO_INDEXES["right_hip_roll_joint"],
            h1.FROM_NAMES_TO_INDEXES["right_hip_pitch_joint"],
            h1.FROM_NAMES_TO_INDEXES["right_knee_joint"],
            h1.FROM_NAMES_TO_INDEXES["left_hip_roll_joint"],
            h1.FROM_NAMES_TO_INDEXES["left_hip_pitch_joint"],
            h1.FROM_NAMES_TO_INDEXES["left_knee_joint"],
            h1.FROM_NAMES_TO_INDEXES["left_hip_yaw_joint"],
            h1.FROM_NAMES_TO_INDEXES["right_hip_yaw_joint"],
            h1.FROM_NAMES_TO_INDEXES["left_ankle_joint"],
            h1.FROM_NAMES_TO_INDEXES["right_ankle_joint"],
        ]

        self.active_joints_hands = [
            h1.FROM_NAMES_TO_INDEXES["right_pinky"],
            h1.FROM_NAMES_TO_INDEXES["right_ring"],
            h1.FROM_NAMES_TO_INDEXES["right_middle"],
            h1.FROM_NAMES_TO_INDEXES["right_index"],
            h1.FROM_NAMES_TO_INDEXES["right_thumb_bend"],
            h1.FROM_NAMES_TO_INDEXES["right_thumb_rotation"],
            h1.FROM_NAMES_TO_INDEXES["left_pinky"],
            h1.FROM_NAMES_TO_INDEXES["left_ring"],
            h1.FROM_NAMES_TO_INDEXES["left_middle"],
            h1.FROM_NAMES_TO_INDEXES["left_index"],
            h1.FROM_NAMES_TO_INDEXES["left_thumb_bend"],
            h1.FROM_NAMES_TO_INDEXES["left_thumb_rotation"],
        ]

        self.active_joints_wrists = [
            h1.FROM_NAMES_TO_INDEXES["left_wrist"],
            h1.FROM_NAMES_TO_INDEXES["right_wrist"],
        ]

        # Initialize control messages
        self._initialize_control_messages()
        
        # Control parameters
        self.impact = 0.0                          # Control influence factor
        self.max_temperature = -1.0                # Maximum joint temperature
        self.velocity_changed = False              # Velocity ramp-up flag
        self.wrist_scale = WRIST_SCALE             # Wrist velocity multiplier
        
        # Declare and get parameters
        self.declare_parameter("max_joint_velocity_param", MAX_JOINT_VELOCITY)
        self.max_joint_velocity = START_JOINT_VELOCITY
        self.declare_parameter("target_topic_param", TARGET_TOPIC)
        self.target_topic = self.get_parameter("target_topic_param").value

        # Timing control
        self.control_dt = 1 / FREQUENCY
        self.time_for_return_control = 2.0         # Time to return control (seconds)
        
        # Calculate maximum joint deltas
        self.max_joint_delta_H1 = self.max_joint_velocity * self.control_dt
        self.max_joint_delta_hands = MAX_FINGER_VELOCITY
        self.max_joint_delta_wrists = self.max_joint_velocity * 5 * self.control_dt

        # Initialize counters
        self.timer_call_count = 0
        self.start_time = self.get_clock().now()

        # Setup publishers and subscribers
        self._setup_communication()

        self.get_logger().info("Node started")

    def _initialize_control_messages(self):
        """Initialize all control message structures"""
        # Hands control message
        self.cmd_msg_hands = MotorCmds()
        for _ in self.active_joints_hands:
            self.cmd_msg_hands.cmds.append(MotorCmd())
        self.get_logger().debug(f"cmd_msg_hands = {str(self.cmd_msg_hands)}")

        # Wrists control message
        self.cmd_msg_wrists = MotorCmds()
        for _ in self.active_joints_wrists:
            self.cmd_msg_wrists.cmds.append(MotorCmd())
        self.get_logger().debug(f"cmd_msg_wrists = {str(self.cmd_msg_wrists)}")

        # Feedback message
        self.feedback_msg = MotorStates()
        for _ in self.active_joints_hands:
            self.feedback_msg.states.append(MotorState())
        self.get_logger().debug(f"feedback_msg = {str(self.feedback_msg)}")

        # H1 control message
        self.create_cmd_msg = LowCmd
        # Workaround for missing field in LowCmd message
        setattr(self.create_cmd_msg, "__idl_typename__", "unitree_go.msg.dds_.LowCmd_")
        self.cmd_msg_H1 = self.create_cmd_msg()
        
        # Set constant message fields
        self.cmd_msg_H1.head[0] = 254
        self.cmd_msg_H1.head[1] = 239
        self.cmd_msg_H1.level_flag = 255
        self.cmd_msg_H1.gpio = 0

    def _setup_communication(self):
        """Setup all publishers and subscribers"""
        # Hands publishers/subscribers
        self.publisher_cmd = self.create_publisher(MotorCmds, "inspire/cmd", 10)
        
        self.subscription_fingers_states = self.create_subscription(
            MotorStates, "inspire/state", 
            self.listener_callback_fingers_states, 10
        )

        # Wrists publishers/subscribers
        self.publisher_wrist_cmds = self.create_publisher(MotorCmds, "wrist/cmds", 10)
        
        self.subscription_wrist_states = self.create_subscription(
            MotorStates, "wrist/states",
            self.listener_callback_wrist_states, 10
        )

        # H1 publishers/subscribers
        self.publisher_arm_sdk = self.create_publisher(LowCmd, self.target_topic, 10)
        
        self.subscription_LowCmd = self.create_subscription(
            LowState, "lowstate",
            self.listener_callback_LowCmd, 10
        )
        
        self.subscription_positions_to_unitree = self.create_subscription(
            String, "positions_to_unitree",
            self.listener_callback_positions_to_unitree, 10
        )

        # Timers
        self.timer_arm_sdk = self.create_timer(
            self.control_dt, self.timer_callback_arm_sdk
        )
        
        self.publisher_temperature = self.create_publisher(Float32, "max_temperature", 10)
        
        self.timer_temperature = self.create_timer(
            1.0, self.timer_callback_temperature
        )

    def listener_callback_positions_to_unitree(self, msg):
        """
        Process incoming position commands from the 'positions_to_unitree' topic.
        Message format should be: JSON_POSITIONS$IMPACT_VALUE
        """
        raw_data = msg.data

        # Split message into position data and impact value
        parts = raw_data.split("$")
        if len(parts) != 2:
            self.get_logger().error(f"Invalid message format: {raw_data}")
            return

        data_part, impact_part = parts

        # Process impact value
        try:
            self.impact = np.clip(float(impact_part), 0.0, 1.0)
        except ValueError:
            self.get_logger().error(f"Invalid impact value: {impact_part}")
            return 

        # Skip if no position data
        if not data_part or data_part == "{}":
            self.get_logger().debug("Empty pose data received, skipping processing")
            return

        # Process position data
        try:
            pose = json.loads(data_part)
            self.get_logger().debug(f"data = {pose}")
            self.get_logger().debug(f"impact = {self.impact}")

            for key, value in pose.items():
                if key.isdigit():  # Verify key is a numeric string
                    index = int(key)
                    limits = self.robot.get_joint_info_by_index(index).limits
                    clipped_value = np.clip(value, limits[0], limits[1])
                    self.robot.update_target_pose(index, clipped_value)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def listener_callback_LowCmd(self, msg):
        """Update current joint positions from H1 low state message"""
        temps = []
        for i in self.active_joints_H1:
            message_index = self.robot.get_joint_info_by_index(i).index_in_msg
            self.robot.update_current_pose(i, msg.motor_state[message_index].q)
            temps.append(msg.motor_state[i].temperature)
        self.max_temperature = max(temps)

    def listener_callback_fingers_states(self, msg):
        """Update current finger positions from hand state message"""
        for i in self.active_joints_hands:
            message_index = self.robot.get_joint_info_by_index(i).index_in_msg
            self.robot.update_current_pose(i, msg.states[message_index].q)

    def listener_callback_wrist_states(self, msg):
        """Update current wrist positions from wrist state message"""
        for i in self.active_joints_wrists:
            message_index = self.robot.get_joint_info_by_index(i).index_in_msg
            self.robot.update_current_pose(i, msg.states[message_index].q)

    def timer_callback_temperature(self):
        """Publish maximum joint temperature"""
        msg = Float32()
        msg.data = float(self.max_temperature)
        self.publisher_temperature.publish(msg)

    def timer_callback_arm_sdk(self):
        """
        Main control loop callback. Updates all joint positions based on:
        - Current and target positions
        - Maximum allowed velocities
        - Control influence factor
        """
        if self.timer_call_count <= 10 or self.impact == 0.0:
            # Initialization phase or zero impact - maintain current positions
            for i in self.active_joints_hands:
                message_index = self.robot.get_joint_info_by_index(i).index_in_msg
                self.cmd_msg_hands.cmds[message_index].q = 1.0
                self.publisher_cmd.publish(self.cmd_msg_hands)
            
            for i in (self.active_joints_H1 + 
                     self.active_joints_hands + 
                     self.active_joints_wrists):
                current_pose = self.robot.get_current_pose(i)
                self.robot.update_temporary_pose(i, current_pose)
        else:
            # Active control phase - update all joints
            self._update_H1_joints()
            self._update_hand_joints()
            self._update_wrist_joints()

        # Publish all commands
        self.cmd_msg_H1.crc = CRC().Crc(self.cmd_msg_H1)
        self.publisher_arm_sdk.publish(self.cmd_msg_H1)
        self.publisher_cmd.publish(self.cmd_msg_hands)
        self.publisher_wrist_cmds.publish(self.cmd_msg_wrists)
        
        self.timer_call_count += 1
        self._update_velocity_profile()

    def _update_H1_joints(self):
        """Update H1 joint commands with velocity-limited movement"""
        # Set control influence
        self.cmd_msg_H1.motor_cmd[h1.FROM_NAMES_TO_INDEXES["IMPACT"]].q = self.impact
        
        # Calculate and apply limited deltas for H1 joints
        for i in self.active_joints_H1:
            delta = (self.robot.get_target_pose(i) - 
                    self.robot.get_temporary_pose(i))
            clamped_delta = np.clip(delta, -self.max_joint_delta_H1, self.max_joint_delta_H1)
            new_temp_pose = (self.robot.get_temporary_pose(i) + 
                           clamped_delta)
            self.robot.update_temporary_pose(i, new_temp_pose)

        # Prepare motor commands for H1
        for j in self.active_joints_H1:
            mes_index = self.robot.get_joint_info_by_index(j).index_in_msg
            coeff_and_mode = h1.determine_coeff_and_mode(j)  # (Kp, Kd, mode)
            
            self.cmd_msg_H1.motor_cmd[mes_index].q = self.robot.get_temporary_pose(j)
            self.cmd_msg_H1.motor_cmd[mes_index].dq = 0.0
            self.cmd_msg_H1.motor_cmd[mes_index].tau = 0.0
            self.cmd_msg_H1.motor_cmd[mes_index].kp = coeff_and_mode[0]
            self.cmd_msg_H1.motor_cmd[mes_index].kd = coeff_and_mode[1]
            self.cmd_msg_H1.motor_cmd[mes_index].mode = coeff_and_mode[2]

    def _update_hand_joints(self):
        """Update hand joint commands with velocity-limited movement"""
        for i in self.active_joints_hands:
            delta = (self.robot.get_target_pose(i) - 
                    self.robot.get_temporary_pose(i))
            clamped_delta = np.clip(
                delta,
                -self.max_joint_delta_hands,
                self.max_joint_delta_hands,
            )
            clamped_delta = round(clamped_delta, 3)
            new_temp_pose = (self.robot.get_temporary_pose(i) + 
                           clamped_delta)
            self.robot.update_temporary_pose(i, new_temp_pose)

        for j in self.active_joints_hands:
            mes_index = self.robot.get_joint_info_by_index(j).index_in_msg
            self.cmd_msg_hands.cmds[mes_index].q = self.robot.get_temporary_pose(j)

    def _update_wrist_joints(self):
        """Update wrist joint commands with velocity-limited movement"""
        for i in self.active_joints_wrists:
            delta = (self.robot.get_target_pose(i) - 
                    self.robot.get_temporary_pose(i))
            clamped_delta = np.clip(
                delta, -self.max_joint_delta_wrists, self.max_joint_delta_wrists
            )
            clamped_delta = round(clamped_delta, 3)
            new_temp_pose = (self.robot.get_temporary_pose(i) + 
                           clamped_delta)
            self.robot.update_temporary_pose(i, new_temp_pose)

        for j in self.active_joints_wrists:
            mes_index = self.robot.get_joint_info_by_index(j).index_in_msg
            coeff_and_mode = h1.determine_coeff_and_mode(j)  # (Kp, Kd, mode)
            
            self.cmd_msg_wrists.cmds[mes_index].q = self.robot.get_temporary_pose(j)
            self.cmd_msg_wrists.cmds[mes_index].dq = 0.0
            self.cmd_msg_wrists.cmds[mes_index].tau = 0.0
            self.cmd_msg_wrists.cmds[mes_index].kp = coeff_and_mode[0]
            self.cmd_msg_wrists.cmds[mes_index].kd = coeff_and_mode[1]

    def _update_velocity_profile(self):
        """Gradually increase joint velocity to target over time"""
        if self.velocity_changed:
            return

        current_time = self.get_clock().now()
        duration = (current_time - self.start_time).nanoseconds / 1e9  # в секундах

        if duration >= TIME_TO_CHANGE_VELOCITY:
            self.max_joint_velocity = self.get_parameter("max_joint_velocity_param").value
            self.max_joint_delta_H1 = self.max_joint_velocity * self.control_dt
            self.max_joint_delta_wrists = self.max_joint_velocity * 5 * self.control_dt
            self.velocity_changed = True
            return

        # Smooth speed increase
        target_velocity = self.get_parameter("max_joint_velocity_param").value
        remaining_time = TIME_TO_CHANGE_VELOCITY - duration
        remaining_steps = remaining_time / self.control_dt

        if remaining_steps > 0:
            velocity_step = (target_velocity - self.max_joint_velocity) / remaining_steps
            self.max_joint_velocity += velocity_step
            self.max_joint_delta_H1 = self.max_joint_velocity * self.control_dt
            self.max_joint_delta_wrists = self.max_joint_velocity * 5 * self.control_dt

        # Overflow protection
        if math.isclose(self.max_joint_velocity, target_velocity, rel_tol=1e-6):
            self.velocity_changed = True

    def return_control(self):
        """Gradually reduce control influence to return control to system"""
        steps = self.time_for_return_control / self.control_dt
        value_of_step = 1.0 / steps
        
        for _ in range(int(steps) + 1):
            self.impact -= value_of_step
            self.impact = np.clip(self.impact, 0.0, 1.0)
            self.cmd_msg_H1.motor_cmd[h1.FROM_NAMES_TO_INDEXES["IMPACT"]].q = self.impact
            self.get_logger().info(f"Impact = {round(self.impact, 3)}")
            self.publisher_arm_sdk.publish(self.cmd_msg_H1)
            time.sleep(self.control_dt)

    def alignment_wrists(self):
        """Move wrists to neutral position"""

        self.get_logger().warn('WRISTS WRIST WRIST')
        self.robot.update_target_pose(32, 1.74)  # Left wrist
        self.robot.update_target_pose(33, -3.16)    # Right wrist
        
        # Calculate movement deltas
        # delta_left = (self.robot.get_target_pose(32) - 
        #              self.robot.get_current_pose(32)) / 2
        # delta_right = (self.robot.get_target_pose(33) - 
        #               self.robot.get_current_pose(33)) / 2

        # for _ in range(2):
        #     # Update temporary poses
        #     self.robot.update_temporary_pose(
        #         32, self.robot.get_temporary_pose(32) + delta_left
        #     )
        #     self.robot.update_temporary_pose(
        #         33, self.robot.get_temporary_pose(33) + delta_right
        #     )

        #     # Prepare wrist commands
        #     for j in self.active_joints_wrists:
        #         mes_index = self.robot.get_joint_info_by_index(j).index_in_msg
        #         coeff_and_mode = h1.determine_coeff_and_mode(j)
                
        #         self.cmd_msg_wrists.cmds[mes_index].q = self.robot.get_temporary_pose(j)
        #         self.cmd_msg_wrists.cmds[mes_index].dq = 0.0
        #         self.cmd_msg_wrists.cmds[mes_index].tau = 0.0
        #         self.cmd_msg_wrists.cmds[mes_index].kp = coeff_and_mode[0]
        #         self.cmd_msg_wrists.cmds[mes_index].kd = coeff_and_mode[1]

        #     self.publisher_wrist_cmds.publish(self.cmd_msg_wrists)
        #     self.get_logger().warn(f'{self.cmd_msg_wrists}')

        #     time.sleep(self.control_dt * 100)

        
    #     for j in self.active_joints_wrists:
    #         mes_index = self.robot.get_joint_info_by_index(j).index_in_msg
    #         coeff_and_mode = h1.determine_coeff_and_mode(j)  # (Kp, Kd, mode)
            
    #         self.cmd_msg_wrists.cmds[mes_index].q = self.robot.get_target_pose(j)
    #         self.cmd_msg_wrists.cmds[mes_index].dq = 0.0
    #         self.cmd_msg_wrists.cmds[mes_index].tau = 0.0
    #         self.cmd_msg_wrists.cmds[mes_index].kp = coeff_and_mode[0]
    #         self.cmd_msg_wrists.cmds[mes_index].kd = coeff_and_mode[1]
                
    #     self.publisher_wrist_cmds.publish(self.cmd_msg_wrists)


    def straighten_fingers(self):
        """Straighten all fingers to default position"""
        for i in self.active_joints_hands:
            message_index = self.robot.get_joint_info_by_index(i).index_in_msg
            self.cmd_msg_hands.cmds[message_index].q = 1.0
        self.publisher_cmd.publish(self.cmd_msg_hands)


def main(args=None):
    rclpy.init(args=args)
    node = LowLevelControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping node...")
    except Exception as e:
        node.get_logger().error(f"Node error: {e}")
    finally:
        # Cleanup procedures
        node.straighten_fingers()
        
        if node.impact != 0.0:
            node.return_control()
            
        node.get_logger().info("Node stopped.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
