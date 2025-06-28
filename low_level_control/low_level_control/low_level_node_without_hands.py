#!/usr/bin/env python3

'''
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
'''

'''
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
'''

import time
import json
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from unitree_go.msg import LowState
from unitree_go.msg import LowCmd
from unitree_sdk2py.utils.crc import CRC


FREQUENCY = 333.33
MAX_JOINT_VELOCITY = 3.0

JOINT_INDEX = {
    'right_hip_roll_joint': 0,
    'right_hip_pitch_joint': 1,
    'right_knee_joint': 2,
    'left_hip_roll_joint': 3,
    'left_hip_pitch_joint': 4,
    'left_knee_joint': 5,
    'torso_joint': 6,
    'left_hip_yaw_joint': 7,
    'right_hip_yaw_joint': 8,
    'NOT USED': 9,
    'left_ankle_joint': 10,
    'right_ankle_joint': 11,
    'right_shoulder_roll_joint': 12,
    'right_shoulder_pitch_joint': 13,
    'right_shoulder_yaw_joint': 14,
    'right_elbow_joint': 15,
    'left_shoulder_roll_joint': 16,
    'left_shoulder_pitch_joint': 17,
    'left_shoulder_yaw_joint': 18,
    'left_elbow_joint': 19
}

LIMITS_OF_JOINTS_UNITREE_H1 = {
    0: [-0.43, 0.43],  # right_hip_roll_joint M
    1: [-3.14, 2.53],  # right_hip_pitch_joint M
    2: [-0.26, 2.05],  # right_knee_joint L
    3: [-0.43, 0.43],  # left_hip_roll_joint M
    4: [-3.14, 2.53],  # left_hip_pitch_joint M
    5: [0.26, 2.05],  # left_knee_joint L
    6: [-2.35, 2.35],  # torso_joint M
    7: [-0.43, 0.43],  # left_hip_yaw_joint M
    8: [-0.43, 0.43],  # right_hip_yaw_joint M
    9: [None, None],  # NOT USED
    10: [-0.87, 0.52],  # left_ankle_joint S
    11: [-0.87, 0.52],  # right_ankle_joint S
    12: [-1.9, 0.5],  # right_shoulder_pitch_joint M
    13: [-2.2, 0.0],  # right_shoulder_roll_joint M
    14: [-1.5, 1.3],  # right_shoulder_yaw_joint M
    15: [-0.5, 1.65],  # right_elbow_joint M
    16: [-1.9, 0.5],  # left_shoulder_pitch_joint M
    17: [0.0, 2.2],  # left_shoulder_roll_joint M
    18: [-1.3, 1.5],  # left_shoulder_yaw_joint M
    19: [-0.5, 1.65]  # left_elbow_joint M
}


class LowLevelControlNode(Node):

    def __init__(self):
        super().__init__('low_level_control_node')

        self.active_joints = [
            JOINT_INDEX['right_shoulder_roll_joint'],
            JOINT_INDEX['right_shoulder_pitch_joint'],
            JOINT_INDEX['right_shoulder_yaw_joint'],
            JOINT_INDEX['right_elbow_joint'],
            JOINT_INDEX['left_shoulder_pitch_joint'],
            JOINT_INDEX['left_shoulder_roll_joint'],
            JOINT_INDEX['left_shoulder_yaw_joint'],
            JOINT_INDEX['left_elbow_joint']
        ]

        self.target_pos = {
            0: 0.0,  # right_hip_roll_joint M
            1: 0.0,  # right_hip_pitch_joint M
            2: 0.0,  # right_knee_joint L
            3: 0.0,  # left_hip_roll_joint M
            4: 0.0,  # left_hip_pitch_joint M
            5: 0.0,  # left_knee_joint L
            6: 0.0,  # torso_joint M
            7: 0.0,  # left_hip_yaw_joint M
            8: 0.0,  # right_hip_yaw_joint M
            9: 0.0,  # NOT USED
            10: 0.0,  # left_ankle_joint S
            11: 0.0,  # right_ankle_joint S
            12: 0.0,  # right_shoulder_pitch_joint M
            13: 0.0,  # right_shoulder_roll_joint M
            14: 0.0,  # right_shoulder_yaw_joint M
            15: 0.0,  # right_elbow_joint M
            16: 0.0,  # left_shoulder_pitch_joint M
            17: 0.0,  # left_shoulder_roll_joint M
            18: 0.0,  # left_shoulder_yaw_joint M
            19: 0.0  # left_elbow_joint M
        }

        # текущая позиция
        self.current_jpos = {
            0: 0.0,  # right_hip_roll_joint M
            1: 0.0,  # right_hip_pitch_joint M
            2: 0.0,  # right_knee_joint L
            3: 0.0,  # left_hip_roll_joint M
            4: 0.0,  # left_hip_pitch_joint M
            5: 0.0,  # left_knee_joint L
            6: 0.0,  # torso_joint M
            7: 0.0,  # left_hip_yaw_joint M
            8: 0.0,  # right_hip_yaw_joint M
            9: 0.0,  # NOT USED
            10: 0.0,  # left_ankle_joint S
            11: 0.0,  # right_ankle_joint S
            12: 0.0,  # right_shoulder_pitch_joint M
            13: 0.0,  # right_shoulder_roll_joint M
            14: 0.0,  # right_shoulder_yaw_joint M
            15: 0.0,  # right_elbow_joint M
            16: 0.0,  # left_shoulder_pitch_joint M
            17: 0.0,  # left_shoulder_roll_joint M
            18: 0.0,  # left_shoulder_yaw_joint M
            19: 0.0  # left_elbow_joint M
        }

        self.current_jpos_des = {
            0: 0.0,  # right_hip_roll_joint M
            1: 0.0,  # right_hip_pitch_joint M
            2: 0.0,  # right_knee_joint L
            3: 0.0,  # left_hip_roll_joint M
            4: 0.0,  # left_hip_pitch_joint M
            5: 0.0,  # left_knee_joint L
            6: 0.0,  # torso_joint M
            7: 0.0,  # left_hip_yaw_joint M
            8: 0.0,  # right_hip_yaw_joint M
            9: 0.0,  # NOT USED
            10: 0.0,  # left_ankle_joint S
            11: 0.0,  # right_ankle_joint S
            12: 0.0,  # right_shoulder_pitch_joint M
            13: 0.0,  # right_shoulder_roll_joint M
            14: 0.0,  # right_shoulder_yaw_joint M
            15: 0.0,  # right_elbow_joint M
            16: 0.0,  # left_shoulder_pitch_joint M
            17: 0.0,  # left_shoulder_roll_joint M
            18: 0.0,  # left_shoulder_yaw_joint M
            19: 0.0  # left_elbow_joint M
        }

        self.impact = 0.0
        self.max_temperature = -1.0

        # частота обновления
        self.control_dt = 1 / FREQUENCY
        self.time_for_return_control = 1.0

        self.max_joint_delta = MAX_JOINT_VELOCITY * self.control_dt

        # считает количество итераций цикла timer_callback
        self.timer_call_count = 0

        self.subscription_LowCmd = self.create_subscription(
            LowState,
            'lowstate',
            self.listener_callback_LowCmd,
            10)

        self.subscription_positions_to_unitree = self.create_subscription(
            String,
            'positions_to_unitree',
            self.listener_callback_positions_to_unitree,
            10)

        self.create_cmd_msg = LowCmd
        # коррекция ошибки - отсутсвие запрашиваемого поля в полях сообщения LowCmd
        # нужно для нормального подсчёта crc
        setattr(
            self.create_cmd_msg,
            '__idl_typename__',
            'unitree_go.msg.dds_.LowCmd_'
        )
        self.cmd_msg = self.create_cmd_msg()
        # константы сообщений
        self.cmd_msg.head[0] = 254
        self.cmd_msg.head[1] = 239
        self.cmd_msg.level_flag = 255
        self.cmd_msg.gpio = 0

        self.publisher_arm_sdk = self.create_publisher(LowCmd, 'arm_sdk', 10)
        self.timer_arm_sdk = self.create_timer(
            self.control_dt,
            self.timer_callback_arm_sdk
        )


        self.publisher_temperature = self.create_publisher(String, 'max_temperature', 10)
        self.timer_temperature = self.create_timer(
            1.0,
            self.timer_callback_temperature
        )

        self.subscription_LowCmd  # prevent unused variable warning
        self.subscription_positions_to_unitree

        self.get_logger().info('Node started')

    def timer_callback_temperature(self):
        msg = String()
        msg.data = str(self.max_temperature)
        self.publisher_temperature.publish(msg)
        

    def listener_callback_positions_to_unitree(self, msg):
        raw_data = msg.data
        data, impact = raw_data.split('$')

        self.impact = float(impact)

        try:
            pose = json.loads(data)
            self.get_logger().info(f'data = {pose}')
            self.get_logger().info(f'impact = {impact}')
            for i in self.active_joints:
                if i == JOINT_INDEX['torso_joint']:
                    self.target_pos[i] = 0.0
                else:
                    self.target_pos[i] = np.clip(
                        pose[str(i)],
                        LIMITS_OF_JOINTS_UNITREE_H1[i][0],
                        LIMITS_OF_JOINTS_UNITREE_H1[i][1]
                    )
        except Exception as e:
            self.get_logger().error(e)

    def listener_callback_LowCmd(self, msg):
        '''Обновляем текущее положение'''
        temps = []
        for i in self.active_joints:
            self.current_jpos[i] = msg.motor_state[i].q
            temps.append(msg.motor_state[i].temperature)
        self.max_temperature = max(temps)
        

    def timer_callback_arm_sdk(self):

        if self.timer_call_count <= 5 or self.impact == 0.0:
            self.current_jpos_des = self.current_jpos.copy()
            self.get_logger().info(
                f'Обновление current_jpos_des = {self.current_jpos_des}')

        else:
            self.cmd_msg.motor_cmd[JOINT_INDEX['NOT USED']].q = self.impact
            for i in self.active_joints:
                delta = self.target_pos[i] - self.current_jpos_des[i]
                clamped_delta = np.clip(
                    delta,
                    -self.max_joint_delta,
                    self.max_joint_delta
                )
                self.current_jpos_des[i] += clamped_delta

            for j in self.active_joints:
                coeff_and_mode = determine_coeff_and_mode(j)  # (Kp, Kd, mode)
                self.cmd_msg.motor_cmd[j].q = self.current_jpos_des[j]
                self.cmd_msg.motor_cmd[j].dq = 0.0
                self.cmd_msg.motor_cmd[j].tau = 0.5
                self.cmd_msg.motor_cmd[j].kp = coeff_and_mode[0]
                self.cmd_msg.motor_cmd[j].kd = coeff_and_mode[1]
                self.cmd_msg.motor_cmd[j].mode = coeff_and_mode[2]

        # Подсчет контрольной суммы, использует CRC для проверки целостности команд
        self.crc = CRC()
        self.cmd_msg.crc = self.crc.Crc(self.cmd_msg)
        self.publisher_arm_sdk.publish(self.cmd_msg)
        self.timer_call_count += 1

    def return_control(self):
        steps = self.time_for_return_control / self.control_dt
        value_of_step = 1.0 / steps
        for _ in range(int(steps) + 1):
            self.impact -= value_of_step
            self.impact = np.clip(self.impact, 0.0, 1.0)
            self.cmd_msg.motor_cmd[JOINT_INDEX['NOT USED']].q = self.impact
            self.get_logger().info(f'Impact = {round(self.impact, 3)}')
            self.publisher_arm_sdk.publish(self.cmd_msg)
            time.sleep(self.control_dt)


def determine_coeff_and_mode(index_of_joint_of_unitree_h1: int) -> tuple:
    '''Функция определения коэффиэнтов Kp, Kd и мода для моторов'''
    size_S = [10, 11]
    size_L = [2, 5]

    # determine Kp and Kd
    if index_of_joint_of_unitree_h1 in size_S:
        Kp = 80.0
        Kd = 2.0

    elif index_of_joint_of_unitree_h1 in size_L:
        Kp = 200.0
        Kd = 5.0

    else:
        Kp = 100.0
        Kd = 3.0

    # determine mode for enable
    if index_of_joint_of_unitree_h1 < 9:
        mode = 0x0A
    elif index_of_joint_of_unitree_h1 > 9:
        mode = 0x01
    else:
        mode = 0x00

    return (Kp, Kd, mode)


def main(args=None):
    rclpy.init(args=args)
    node = LowLevelControlNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Stop node.')

    except Exception as e:
        node.get_logger().error(e)

    finally:
        if node.impact != 0.0:
            node.return_control()
        node.get_logger().info('Node stoped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
