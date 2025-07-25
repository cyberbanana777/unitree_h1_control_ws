#!/usr/bin/env python3

'''
АННОТАЦИЯ
Скрипт создаёт ROS2-ноду "low_level_control_node", которая позволяет управлять
сочленениями робота "Unitree H1" и руками "inspire hands". Нода слушает топик "positions_to_unitree", в
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
the articulations of the “Unitree H1” robot and hands "inspire hands". The node listens to the “positions_to_unitree” topic, to
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

import json
import time

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32
from unitree_go.msg import MotorState
from unitree_go.msg import MotorStates
from unitree_go.msg import MotorCmd
from unitree_go.msg import MotorCmds
from unitree_go.msg import LowCmd
from unitree_go.msg import LowState
from unitree_sdk2py.utils.crc import CRC


# Частота в гц для ноды
FREQUENCY = 333.33
# коэффициэнт на который умножается control_dt (сек), который устанавливает максимальную дельту поворота мотора
START_JOINT_VELOCITY = 0.5
MAX_JOINT_VELOCITY = 7.0
MAX_FINGER_VELOCITY = 0.6
TARGET_TOPIC = 'arm_sdk'
TIME_TO_CHANGE_VELOCITY = 5.0 # в секундах

JOINT_INDEX_H1 = {
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

JOINT_INDEX_HANDS = {
    'right_pinky': 0,
    'right_ring': 1,
    'right_middle': 2,
    'right_index': 3,
    'right_thumb_bend': 4,
    'right_thumb_rotation': 5,
    'left_pinky': 6,
    'left_ring': 7,
    'left_middle': 8,
    'left_index': 9,
    'left_thumb_bend': 10,
    'left_thumb_rotation': 11,
}

JOINT_INDEX_WRISTS = {
    'left_wrist' : 0,
    'right_wrist' : 1
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
    15: [-1.1, 1.65],  # right_elbow_joint M
    16: [-1.9, 0.5],  # left_shoulder_pitch_joint M
    17: [0.0, 2.2],  # left_shoulder_roll_joint M
    18: [-1.3, 1.5],  # left_shoulder_yaw_joint M
    19: [-1.1, 1.65]  # left_elbow_joint M
}

LIMITS_OF_JOINTS_UNITREE_HANDS = {
    0: [0.0, 1.0],  # right_pinky
    1: [0.0, 1.0],  # right_ring
    2: [0.0, 1.0],  # right_middle
    3: [0.0, 1.0],  # right_index
    4: [0.0, 1.0],  # right_thumb-bend
    5: [0.0, 1.0],  # right_thumb-rotation
    6: [0.0, 1.0],  # left_pinky
    7: [0.0, 1.0],  # left_ring
    8: [0.0, 1.0],  # left_middle
    9: [0.0, 1.0],  # left_index
    10: [0.0, 1.0],  # left_thumb-bend
    11: [0.0, 1.0]  # left_thumb-rotation
}

LIMITS_OF_JOINTS_UNITREE_WRISTS = {
    0: [-1.1, 4.58],  # left_wrist
    1: [-6.0, -0.23]  # right_wrist
}


class LowLevelControlNode(Node):

    def __init__(self):
        super().__init__('low_level_control_with_hands_node')

        self.active_joints_H1 = [
            JOINT_INDEX_H1['right_shoulder_roll_joint'],
            JOINT_INDEX_H1['right_shoulder_pitch_joint'],
            JOINT_INDEX_H1['right_shoulder_yaw_joint'],
            JOINT_INDEX_H1['right_elbow_joint'],
            JOINT_INDEX_H1['left_shoulder_pitch_joint'],
            JOINT_INDEX_H1['left_shoulder_roll_joint'],
            JOINT_INDEX_H1['left_shoulder_yaw_joint'],
            JOINT_INDEX_H1['left_elbow_joint'],
            JOINT_INDEX_H1['torso_joint']
        ]

        self.active_joints_hands = [
            JOINT_INDEX_HANDS['right_pinky'],
            JOINT_INDEX_HANDS['right_ring'],
            JOINT_INDEX_HANDS['right_middle'],
            JOINT_INDEX_HANDS['right_index'],
            JOINT_INDEX_HANDS['right_thumb_bend'],
            JOINT_INDEX_HANDS['right_thumb_rotation'],
            JOINT_INDEX_HANDS['left_pinky'],
            JOINT_INDEX_HANDS['left_ring'],
            JOINT_INDEX_HANDS['left_middle'],
            JOINT_INDEX_HANDS['left_index'],
            JOINT_INDEX_HANDS['left_thumb_bend'],
            JOINT_INDEX_HANDS['left_thumb_rotation']
        ]

        self.active_joints_wrists = [
            JOINT_INDEX_WRISTS['left_wrist'],
            JOINT_INDEX_WRISTS['right_wrist']
        ]

        # целевая позиция
        self.target_pos_H1 = {
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

        self.target_pos_hands = {
            0: 1.0,  # pinky
            1: 1.0,  # ring
            2: 1.0,  # middle
            3: 1.0,  # index
            4: 1.0,  # thumb-bend
            5: 1.0,  # thumb-rotation
            6: 1.0,  # pinky
            7: 1.0,  # ring
            8: 1.0,  # middle
            9: 1.0,  # index
            10: 1.0,  # thumb-bend
            11: 1.0  # thumb-rotation
        }

        self.target_pos_wrists = {
            0: 0.0, # left_wrist
            1: -1.0  # right_wrist
        }

        # текущая позиция
        self.current_jpos_H1 = {
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

        self.current_jpos_hands = {
            0: 1.0,  # pinky
            1: 1.0,  # ring
            2: 1.0,  # middle
            3: 1.0,  # index
            4: 1.0,  # thumb-bend
            5: 1.0,  # thumb-rotation
            6: 1.0,  # pinky
            7: 1.0,  # ring
            8: 1.0,  # middle
            9: 1.0,  # index
            10: 1.0,  # thumb-bend
            11: 1.0  # thumb-rotation
        }

        self.current_jpos_wrists = {
            0: -1.0, # left_wrist
            1: -1.0  # right_wrist
        }

        # текущая позиция на максимальную дельту приближенная к целевой позиции
        self.current_jpos_des_H1 = {
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

        self.current_jpos_des_hands = {
            0: 1.0,  # pinky
            1: 1.0,  # ring
            2: 1.0,  # middle
            3: 1.0,  # index
            4: 1.0,  # thumb-bend
            5: 1.0,  # thumb-rotation
            6: 1.0,  # pinky
            7: 1.0,  # ring
            8: 1.0,  # middle
            9: 1.0,  # index
            10: 1.0,  # thumb-bend
            11: 1.0  # thumb-rotation
        }

        self.current_jpos_des_wrists = {
            0: 0.0, # left_wrist
            1: -1.0  # right_wrist
        }

        # create control_message for hands (fingers)
        self.cmd_msg_hands = MotorCmds()
        for i in self.active_joints_hands:
            cmd_sub_msg = MotorCmd()
            self.cmd_msg_hands.cmds.append(cmd_sub_msg)
        self.get_logger().debug(f'cmd_msg_hands = {str(self.cmd_msg_hands)}')

        # create control_message for wrists
        self.cmd_msg_wrists = MotorCmds()
        for i in self.active_joints_wrists:
            cmd_sub_msg = MotorCmd()
            self.cmd_msg_wrists.cmds.append(cmd_sub_msg)
        self.get_logger().debug(f'cmd_msg_wrists = {str(self.cmd_msg_wrists)}')

        # create feedback_message
        self.feedback_msg = MotorStates()
        for i in self.active_joints_hands:
            feedback_sub_msg = MotorState()
            self.feedback_msg.states.append(feedback_sub_msg)
        self.get_logger().debug(f'feedback_msg = {str(self.feedback_msg)}')

        self.max_joint_delta_hands = MAX_FINGER_VELOCITY

        # показывает насколько робот смещается в соторону координат с arm_sdk топика
        self.impact = 0.0
        self.max_temperature = -1.0

        # Объявление параметра максимальной скорости
        self.velocity_changed = False 
        self.declare_parameter('max_joint_velocity_param', MAX_JOINT_VELOCITY)
        self.max_joint_velocity = START_JOINT_VELOCITY

        # Обявление параметра целевого топика
        self.declare_parameter('target_topic_param', TARGET_TOPIC)
        self.target_topic = self.get_parameter('target_topic_param').value

        # частота обновления
        self.control_dt = 1 / FREQUENCY

        # время за которое мы возвращаем контроль системе при закрытии ноды
        self.time_for_return_control = 2.0

        # максимальный угол на который может измениться целевая поза за один оборот ноды
        self.max_joint_delta_H1 = self.max_joint_velocity * self.control_dt
        self.max_joint_wrists = self.max_joint_velocity * 5 * self.control_dt

        # считает количество итераций цикла timer_callback
        self.timer_call_count = 0

        self.publisher_cmd = self.create_publisher(
            MotorCmds,
            'inspire/cmd',
            10
        )

        self.subscription_fingers_states = self.create_subscription(
            MotorStates,
            'inspire/state',
            self.listener_callback_fingers_states,
            10
        )

        # for wrists
        self.subscription_wrist_states = self.create_subscription(
            MotorStates,
            'wrist/states',
            self.listener_callback_wrist_states,
            10
        )

        self.publisher_wrist_cmds = self.create_publisher(
            MotorCmds,
            'wrist/cmds',
            10
        )

        # подписка на топик, в который публикуется информация о углах поворота звеньев
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
            self.create_cmd_msg, '__idl_typename__', 'unitree_go.msg.dds_.LowCmd_')
        self.cmd_msg_H1 = self.create_cmd_msg()

        # константы сообщений
        self.cmd_msg_H1.head[0] = 254
        self.cmd_msg_H1.head[1] = 239
        self.cmd_msg_H1.level_flag = 255
        self.cmd_msg_H1.gpio = 0

        # publish terget pose for H1 
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
            Float32, 
            'max_temperature', 
            10
            )
        
        self.timer_temperature = self.create_timer(
            1.0,
            self.timer_callback_temperature
        )

        self.subscription_LowCmd  # prevent unused variable warning
        self.subscription_positions_to_unitree

        self.get_logger().info('Node started')

        self.start_time = self.get_clock().now()  # возвращает Time объект

    def listener_callback_positions_to_unitree(self, msg):
        raw_data = msg.data
        
        # Разделяем данные по символу '$'
        parts = raw_data.split('$')
        if len(parts) != 2:
            self.get_logger().error(f"Invalid message format: {raw_data}")
            return
        
        data_part, impact_part = parts
        self.impact = float(impact_part) if impact_part else 0.0  # Устанавливаем impact в 0.0, если нет значения
        
        # Если часть данных пустая или содержит только '{}', пропускаем обработку позы
        if not data_part or data_part == '{}':
            self.get_logger().debug('Empty pose data received, skipping processing')
            return

        try:
            pose = json.loads(data_part)
            self.get_logger().debug(f'data = {pose}')
            self.get_logger().debug(f'impact = {self.impact}')

            H1_pose = {}
            hands_pose = {}
            wrists_pose = {}

            for key, value in pose.items():
                if key.isdigit():  # Проверяем, является ли ключ числом в строке
                    num = int(key)
                    if num == 32 or num == 33:
                        wrists_pose[num - 32] = value
                    elif num <= 19:
                        H1_pose[num] = value
                    else:
                        hands_pose[num - 20] = value

            for i in H1_pose:
                if i in self.active_joints_H1:
                    if i == JOINT_INDEX_H1['torso_joint']:
                        self.target_pos_H1[i] = 0.0
                    else:
                        self.get_logger().info(f'h1_pose = {H1_pose[i]}')
                        self.target_pos_H1[i] = np.clip(
                            H1_pose.get(i),  # Используем get с значением по умолчанию
                            LIMITS_OF_JOINTS_UNITREE_H1[i][0],
                            LIMITS_OF_JOINTS_UNITREE_H1[i][1]
                        )
                        self.get_logger().info(f'h1_pose = {self.target_pos_H1}')

            for i in hands_pose:
                if i in self.active_joints_hands:
                    self.get_logger().debug(f'hands_pose = {hands_pose[i]}')
                    self.target_pos_hands[i] = np.clip(
                        hands_pose.get(i),  # Используем get с значением по умолчанию
                        LIMITS_OF_JOINTS_UNITREE_HANDS[i][0],
                        LIMITS_OF_JOINTS_UNITREE_HANDS[i][1]
                    )

            for i in wrists_pose:
                if i in self.active_joints_wrists:
                    self.get_logger().debug(f'wrists_pose = {wrists_pose[i]}')
                    self.target_pos_wrists[i] = np.clip(
                        wrists_pose.get(i),  # Используем get с значением по умолчанию
                        LIMITS_OF_JOINTS_UNITREE_WRISTS[i][0],
                        LIMITS_OF_JOINTS_UNITREE_WRISTS[i][1]
                    )

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")


    def listener_callback_LowCmd(self, msg):
        '''Обновляем текущее положение'''
        temps = []
        for i in self.active_joints_H1:
            self.current_jpos_H1[i] = msg.motor_state[i].q
            temps.append(msg.motor_state[i].temperature)
        self.max_temperature = max(temps)
     

    def listener_callback_fingers_states(self, msg):
        '''Обновляем текущее положение пальцев'''
        for i in self.active_joints_hands:
            self.current_jpos_hands[i] = msg.states[i].q

    def listener_callback_wrist_states(self, msg):
        '''Обновляем текущее положение кистей'''
        for i in self.active_joints_wrists:
            self.current_jpos_wrists[i] = msg.states[i].q


    def timer_callback_temperature(self):
        msg = Float32()
        msg.data = float(self.max_temperature)
        self.publisher_temperature.publish(msg)


    def timer_callback_arm_sdk(self):

        if self.timer_call_count <= 10 or self.impact == 0.0:
            self.current_jpos_des_H1 = self.current_jpos_H1.copy()
            self.get_logger().debug(
                f'Обновление current_jpos_des_H1 = {self.current_jpos_des_H1}')
            
            self.current_jpos_des_hands = self.current_jpos_hands.copy()
            self.get_logger().debug(
                f'Обновление current_jpos_des_hands = {self.current_jpos_des_hands}')
            
            self.current_jpos_des_wrists = self.current_jpos_wrists.copy()
            self.get_logger().debug(
                f'Обновление current_jpos_des_wrists = {self.current_jpos_des_wrists}')
            
            for i in self.active_joints_hands:
                self.cmd_msg_hands.cmds[i].q = 1.0
            self.publisher_cmd.publish(self.cmd_msg_hands)

        else:
            
            # Установка значений для H1
            self.cmd_msg_H1.motor_cmd[JOINT_INDEX_H1['NOT USED']
                                      ].q = self.impact
            for i in self.active_joints_H1:
                delta_H1 = self.target_pos_H1[i] - self.current_jpos_des_H1[i]
                clamped_delta_H1 = np.clip(
                    delta_H1,
                    -self.max_joint_delta_H1,
                    self.max_joint_delta_H1
                )
                self.current_jpos_des_H1[i] += clamped_delta_H1

            for j in self.active_joints_H1:
                coeff_and_mode = determine_coeff_and_mode(j)  # (Kp, Kd, mode)
                self.cmd_msg_H1.motor_cmd[j].q = self.current_jpos_des_H1[j]
                self.cmd_msg_H1.motor_cmd[j].dq = 0.0
                self.cmd_msg_H1.motor_cmd[j].tau = 0.0
                self.cmd_msg_H1.motor_cmd[j].kp = coeff_and_mode[0]
                self.cmd_msg_H1.motor_cmd[j].kd = coeff_and_mode[1]
                self.cmd_msg_H1.motor_cmd[j].mode = coeff_and_mode[2]

            # Установка занчений для hands
            for i in self.active_joints_hands:
                delta_hands = self.target_pos_hands[i] - \
                    self.current_jpos_des_hands[i]
                clamped_delta_hands = np.clip(
                    delta_hands,
                    -self.max_joint_delta_hands,
                    self.max_joint_delta_hands
                )
                clamped_delta_hands = round(clamped_delta_hands, 3)
                self.current_jpos_des_hands[i] += clamped_delta_hands
            self.get_logger().debug(f'{self.current_jpos_des_hands}')

            for i in self.active_joints_hands:
                self.cmd_msg_hands.cmds[i].q = self.current_jpos_des_hands[i]

            # Установка занчений для wrists
            for i in self.active_joints_wrists:
                delta_wrists = self.target_pos_wrists[i] - \
                    self.current_jpos_des_wrists[i]
                clamped_delta_wrists = np.clip(
                    delta_wrists,
                    -self.max_joint_wrists,
                    self.max_joint_wrists
                )
                clamped_delta_wrists = round(clamped_delta_wrists, 3)
                self.current_jpos_des_wrists[i] += clamped_delta_wrists
            self.get_logger().debug(f'{self.current_jpos_des_wrists}')

            for i in self.active_joints_wrists:
                self.cmd_msg_wrists.cmds[i].q = self.current_jpos_des_wrists[i]
                self.cmd_msg_wrists.cmds[i].dq = 0.0
                self.cmd_msg_wrists.cmds[i].tau = 0.0
                self.cmd_msg_wrists.cmds[i].kp = 10.0
                self.cmd_msg_wrists.cmds[i].kd = 2.0

        # Подсчет контрольной суммы, использует CRC для проверки целостности команд
        self.crc = CRC()
        self.cmd_msg_H1.crc = self.crc.Crc(self.cmd_msg_H1)
        self.publisher_arm_sdk.publish(self.cmd_msg_H1)
        self.publisher_cmd.publish(self.cmd_msg_hands)
        self.publisher_wrist_cmds.publish(self.cmd_msg_wrists)
        self.timer_call_count += 1


        end_time = self.get_clock().now()
        duration = end_time - self.start_time  # Duration объект
        if self.velocity_changed == False and duration.nanoseconds / 1e9 > TIME_TO_CHANGE_VELOCITY:
            self.max_joint_velocity = self.get_parameter('max_joint_velocity_param').value
            # максимальный угол на который может измениться целевая поза за один оборот ноды
            self.max_joint_delta_H1 = self.max_joint_velocity * self.control_dt
            self.max_joint_wrists = self.max_joint_velocity * 5 * self.control_dt
            self.velocity_changed = True


    def return_control(self):
        '''уводит руки в положение для ходьбы'''
        steps = self.time_for_return_control / self.control_dt
        value_of_step = 1.0 / steps
        for _ in range(int(steps) + 1):
            self.impact -= value_of_step
            self.impact = np.clip(self.impact, 0.0, 1.0)
            self.cmd_msg_H1.motor_cmd[JOINT_INDEX_H1['NOT USED']
                                      ].q = self.impact
            self.get_logger().info(f'Impact = {round(self.impact, 3)}')

            self.publisher_arm_sdk.publish(self.cmd_msg_H1)
            time.sleep(self.control_dt)

# Добавить, что бы ксти поворачивались в перпендикулярное полу положение !!!!!!!!!!!

    def straighten_fingers(self):
        '''распрямляет пальцы'''
        for i in self.active_joints_hands:
            delta_hands = 1.0 - self.current_jpos_des_hands[i]
            clamped_delta_hands = np.clip(
                delta_hands,
                -self.max_joint_delta_hands,
                self.max_joint_delta_hands
            )
            clamped_delta_hands = round(clamped_delta_hands, 3)
            self.current_jpos_des_hands[i] += clamped_delta_hands
            self.get_logger().debug(f'current_jpos_des_hands = {str(self.current_jpos_des_hands)}')

            for i in self.active_joints_hands:
                self.cmd_msg_hands.cmds[i].q = 1.0

                self.publisher_cmd.publish(self.cmd_msg_hands)
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
        node.straighten_fingers()
        if node.impact != 0.0:
            node.return_control()
        node.get_logger().info('Node stoped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
