#!/usr/bin/env python3

"""
АННОТАЦИЯ
Скрипт создаёт ROS2-ноду "low_level_control_node", которая позволяет управлять
сочленениями робота "Unitree H1". Нода слушает топик "positions_to_unitree", в
который публикуется сообщение типа "String", который представляет собой
контескацию 2 сток: десериализованного json-пакета, ключами которого являются
индексы "Unitree H1", а значениями - целевые значения координат сочленения
данного индекса, и переменной типа float, которая означет степень вляния
управляющих координат. Значение переменной влияния должно находится в пределах
[0.0, 1.0], где 1.0 - полный контроль, а 0.0 - отсутсвие контроля.
Эти 2 логических блока в сообщении разделяются знаком "$"
Пример сообщения:
{"16": -0.1, "19": 1.65, "12": -0.1, "13": 0.0, "14": -0.1, "15": 1.65}$1.0
"""

"""
ANNOTATION
The script creates a ROS2 node “low_level_control_node” that allows to control the
the articulations of the “Unitree H1” robot. The node listens to the “positions_to_unitree” topic, to
which publishes a message of type “String”, which is a
contescation of 2 stokes: a deserialized json package whose keys are the
indices “Unitree H1”, and the values are the target values of the articulation coordinates of the
of the given index, and a variable of float type, which means the degree of influence of the
of the control coordinates. The value of the influence variable should be in the range
[0.0, 1.0], where 1.0 is full control and 0.0 is no control.
These 2 logical blocks are separated by a “$” sign in the message
Example message:
{“16”: -0.1, “19”: 1.65, “12”: -0.1, “13”: 0.0, “14”: -0.1, “15”: 1.65}$1.0
"""

import json
import time
import math

import numpy as np
import rclpy
import h1_info_library as h1
from rclpy.node import Node
from std_msgs.msg import String
from unitree_go.msg import LowCmd, LowState
from unitree_sdk2py.utils.crc import CRC

FREQUENCY = 333.33
START_JOINT_VELOCITY = 0.5
MAX_JOINT_VELOCITY = 7.0
TARGET_TOPIC = "arm_sdk"
TIME_TO_CHANGE_VELOCITY = 7.0
TARGET_ACTION = 'teleoperation'


class LowLevelControlNode(Node):
    def __init__(self):
        super().__init__("low_level_control_without_hands_node")
        
        self.declare_parameter("target_action", TARGET_ACTION)
        self.target_action_param = self.get_parameter("target_action").value

        self.robot = h1.RobotData(target_action = self.target_action_param, include_hands_with_fingers = True)

        self.active_joints_H1 = [
            h1.FROM_NAMES_TO_INDEXES["right_shoulder_roll_joint"],
            h1.FROM_NAMES_TO_INDEXES["right_shoulder_pitch_joint"],
            h1.FROM_NAMES_TO_INDEXES["right_shoulder_yaw_joint"],
            h1.FROM_NAMES_TO_INDEXES["right_elbow_joint"],
            h1.FROM_NAMES_TO_INDEXES["left_shoulder_pitch_joint"],
            h1.FROM_NAMES_TO_INDEXES["left_shoulder_roll_joint"],
            h1.FROM_NAMES_TO_INDEXES["left_shoulder_yaw_joint"],
            h1.FROM_NAMES_TO_INDEXES["left_elbow_joint"],
            h1.FROM_NAMES_TO_INDEXES["torso_joint"],
        ]

        self.impact = 0.0
        self.max_temperature = -1.0
        self.start_time = self.get_clock().now()

        # частота обновления
        self.control_dt = 1 / FREQUENCY
        self.time_for_return_control = 1.0

        # Объявление параметра максимальной скорости
        self.velocity_changed = False
        self.declare_parameter("max_joint_velocity_param", MAX_JOINT_VELOCITY)
        self.max_joint_velocity = START_JOINT_VELOCITY

        # Обявление параметра целевого топика
        self.declare_parameter("target_topic_param", TARGET_TOPIC)
        self.target_topic = self.get_parameter("target_topic_param").value

        # максимальный угол на который может измениться целевая поза за один оборот ноды
        self.max_joint_delta = self.max_joint_velocity * self.control_dt

        # считает количество итераций цикла timer_callback
        self.timer_call_count = 0

        self.subscription_LowCmd = self.create_subscription(
            LowState, "lowstate", self.listener_callback_LowCmd, 10
        )

        self.subscription_positions_to_unitree = self.create_subscription(
            String,
            "positions_to_unitree",
            self.listener_callback_positions_to_unitree,
            10,
        )

        self.create_cmd_msg = LowCmd
        # коррекция ошибки - отсутсвие запрашиваемого поля в полях сообщения LowCmd
        # нужно для нормального подсчёта crc
        setattr(
            self.create_cmd_msg,
            "__idl_typename__",
            "unitree_go.msg.dds_.LowCmd_",
        )
        self.cmd_msg = self.create_cmd_msg()
        # константы сообщений
        self.cmd_msg.head[0] = 254
        self.cmd_msg.head[1] = 239
        self.cmd_msg.level_flag = 255
        self.cmd_msg.gpio = 0

        self.publisher_arm_sdk = self.create_publisher(
            LowCmd, 
            self.target_topic, 
            10
        )
        self.timer_arm_sdk = self.create_timer(
            self.control_dt, 
            self.timer_callback_arm_sdk
        )

        self.publisher_temperature = self.create_publisher(
            String, 
            "max_temperature", 
            10
        )
        self.timer_temperature = self.create_timer(
            1.0, 
            self.timer_callback_temperature
        )

        self.subscription_LowCmd  # prevent unused variable warning
        self.subscription_positions_to_unitree

        self.get_logger().info("Node started")

    def timer_callback_temperature(self):
        msg = String()
        msg.data = str(self.max_temperature)
        self.publisher_temperature.publish(msg)

    def listener_callback_positions_to_unitree(self, msg):
        raw_data = msg.data

        # Разделяем данные по символу '$'
        parts = raw_data.split("$")
        if len(parts) != 2:
            self.get_logger().error(f"Invalid message format: {raw_data}")
            return

        data_part, impact_part = parts
        
        try:
            self.impact = np.clip(float(impact_part), 0.0, 1.0)
        except ValueError:
            self.get_logger().error(f"Invalid impact value: {impact_part}")
            return

        # Если часть данных пустая или содержит только '{}', пропускаем обработку позы
        if not data_part or data_part == "{}":
            self.get_logger().debug(
                "Empty pose data received, skipping processing"
            )
            return

        try:
            pose = json.loads(data_part)
            self.get_logger().debug(f"data = {pose}")
            self.get_logger().debug(f"impact = {self.impact}")

            for key, value in pose.items():
                if (
                    key.isdigit()
                ):  # Проверяем, является ли ключ числом в строке
                    index = int(key)
                    limits = self.robot.get_joint_info_by_index(index).limits
                    self.robot.update_target_pose(index, np.clip(
                        value, 
                        limits[0],
                        limits[1]
                    ))

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def listener_callback_LowCmd(self, msg):
        """Обновляем текущее положение"""
        temps = []
        for i in self.active_joints_H1:
            message_index = self.robot.get_joint_info_by_index(i).index_in_msg
            self.robot.update_current_pose(i, msg.motor_state[message_index].q)
            temps.append(msg.motor_state[i].temperature)
        self.max_temperature = max(temps)

    def timer_callback_arm_sdk(self):

        if self.timer_call_count <= 10 or self.impact == 0.0:

            for i in self.active_joints_H1:
                self.robot.update_temporary_pose(i, self.robot.joints_pose_status[i].current_pose)
                self.get_logger().debug(
                    f"Обновление temporary_pose = {self.robot.joints_pose_status}"
                )

        else:
            # Установка значений для H1
            self.cmd_msg.motor_cmd[
                h1.FROM_NAMES_TO_INDEXES["IMPACT"]
            ].q = self.impact
            for i in self.active_joints_H1:
                delta_H1 = self.robot.joints_pose_status[i].target_pose - self.robot.joints_pose_status[i].current_pose
                clamped_delta_H1 = np.clip(
                    delta_H1, 
                    -self.max_joint_delta, 
                    self.max_joint_delta
                )
                self.robot.update_temporary_pose(i,  self.robot.joints_pose_status[i].temporary_pose + clamped_delta_H1)

            for j in self.active_joints_H1:
                mes_index = self.robot.get_joint_info_by_index(j).index_in_msg
                coeff_and_mode = h1.determine_coeff_and_mode(j)  # (Kp, Kd, mode)
                self.cmd_msg.motor_cmd[mes_index].q = self.robot.joints_pose_status[j].temporary_pose
                self.cmd_msg.motor_cmd[mes_index].dq = 0.0
                self.cmd_msg.motor_cmd[mes_index].tau = 0.0
                self.cmd_msg.motor_cmd[mes_index].kp = coeff_and_mode[0]
                self.cmd_msg.motor_cmd[mes_index].kd = coeff_and_mode[1]
                self.cmd_msg.motor_cmd[mes_index].mode = coeff_and_mode[2]

        # Подсчет контрольной суммы, использует CRC для проверки целостности команд
        self.crc = CRC()
        self.cmd_msg.crc = self.crc.Crc(self.cmd_msg)
        self.publisher_arm_sdk.publish(self.cmd_msg)
        self.timer_call_count += 1

        end_time = self.get_clock().now()
        duration = end_time - self.start_time  # Duration объект
        
        if not self.velocity_changed and duration.nanoseconds/1e9 > TIME_TO_CHANGE_VELOCITY:
            target_velocity = self.get_parameter("max_joint_velocity_param").value
            steps_needed = TIME_TO_CHANGE_VELOCITY / self.control_dt  # Общее число шагов
            velocity_step = (target_velocity - START_JOINT_VELOCITY) / steps_needed
            
            self.max_joint_velocity = min(
                self.max_joint_velocity + velocity_step,
                target_velocity
            )
            self.max_joint_delta = self.max_joint_velocity * self.control_dt

            if math.isclose(self.max_joint_velocity, target_velocity, rel_tol=1e-6):
                self.velocity_changed = True


    def return_control(self):
        """уводит руки в положение для ходьбы"""
        steps = self.time_for_return_control / self.control_dt
        value_of_step = 1.0 / steps
        for _ in range(int(steps) + 1):
            self.impact -= value_of_step
            self.impact = np.clip(self.impact, 0.0, 1.0)
            self.cmd_msg.motor_cmd[
                h1.FROM_NAMES_TO_INDEXES["IMPACT"]
            ].q = self.impact
            self.get_logger().info(f"Impact = {round(self.impact, 3)}")

            self.publisher_arm_sdk.publish(self.cmd_msg)
            time.sleep(self.control_dt)


def main(args=None):
    rclpy.init(args=args)
    node = LowLevelControlNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Stop node.")

    except Exception as e:
        node.get_logger().error(e)

    finally:
        if node.impact != 0.0:
            node.return_control()
        node.get_logger().info("Node stoped.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
