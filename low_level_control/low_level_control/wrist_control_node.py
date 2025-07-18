#!/usr/bin/env python3

'''
АННОТАЦИЯ
ROS2-нода wrist_control_node управляет моторами DM4310 через CAN-интерфейс, 
получая целевые параметры (позицию, скорость, момент) через топик wrist/cmds
и публикуя их состояние (wrist/states) с частотой 1000 Гц. Нода автоматически
ограничивает позицию моторов соответствующими диапазонами и работает в 
MIT-режиме. Подключение инициализируется через UART-CAN адаптер (/dev/ttyACM0),
при ошибках выводятся логи. Для управления используются ROS2-сообщения 
MotorCmds (команды) и MotorStates (текущее состояние). Нода корректно завершает
работу, отключая моторы и соединение.
'''

'''
ANNOTATION
The wrist_control_node ROS2 node controls the DM4310 motors via the CAN
interface, receiving target parameters (position, speed, torque) via the
wrist/cmds topic and publishing their state (wrist/states) at a frequency of
1000 Hz. The node automatically limits the motor position to the appropriate
ranges and operates in MIT mode. The connection is initialized via the UART-CAN
adapter (/dev/ttyACM0), logs are output in case of errors. ROS2 messages
MotorCmds (commands) and MotorStates (current state) are used for control. 
The node gracefully terminates by disconnecting the motors and the connection.
'''

import serial
import numpy as np
import rclpy
from rclpy.node import Node
from unitree_go.msg import MotorCmds, MotorState, MotorStates
from .DM_CAN import Motor, DM_Motor_Type, MotorControl, Control_Type


TOPIC_CMD = 'wrist/cmds'
TOPIC_STATES = 'wrist/states'
FREQUENCY = 333

LEFT_ALIAS = 0
RIGHT_ALIAS = 1

MOTOR_LIMITS = {
    RIGHT_ALIAS: [-6.0, -0.23],
    LEFT_ALIAS: [-1.1, 4.58],
}
# Левый предел - предел вращения против часовой стрелке, считая от робота
# Правый предел - предел вращения по часовой стрелке, считая от робота

class PubSubNode(Node):
    def __init__(self):
        super().__init__('wrist_control_node')

        # 1. Инициализация мотора и подключения
        # Тип мотора, CAN ID,       Master ID
        self.motor_left = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
        self.motor_right = Motor(DM_Motor_Type.DM4310, 0x02, 0x12)

        try:
            self.serial_device = serial.Serial(
                '/dev/ttyACM0', 921600, timeout=0.5)
            self.motor_controler = MotorControl(self.serial_device)
            self.motor_controler.addMotor(self.motor_left)
            self.motor_controler.addMotor(self.motor_right)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection error: {e}")
            raise

        # 3. Включение мотора
        self.motor_controler.enable(self.motor_left)
        self.get_logger().info("Левый мотор включен")
        self.motor_controler.enable(self.motor_right)
        self.get_logger().info("Правый мотор включен")

        self.subscription = self.create_subscription(
            MotorCmds,
            TOPIC_CMD,
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(
            MotorStates,
            TOPIC_STATES,
            10)

        # Создаем таймер для периодической публикации (1 раз в секунду)
        self.timer = self.create_timer(
            1 / FREQUENCY,  # период в секундах
            self.timer_callback)

        self.get_logger().info(
            f'Node started: subscribes to "{TOPIC_CMD}", \
            publishes to "{TOPIC_STATES}"'
        )

        self.motor_controler.controlMIT(
            self.motor_left,
            10.0,
            2.0,
            0.0,   # Желаемая позиция
            0.0,  # Желаемая скорость
            0.0  # Момент 
        )

        self.motor_controler.controlMIT(
            self.motor_right,
            10.0,
            2.0,
            -2.0,   # Желаемая позиция
            0.0,  # Желаемая скорость
            0.0  # Момент 
        )


    def listener_callback(self, msg):
        """Обработка полученного сообщения."""
        self.get_logger().debug(f'Received: {msg}')
        left_motor_cmd = msg.cmds[LEFT_ALIAS]
        right_motor_cmd = msg.cmds[RIGHT_ALIAS]

        target_position_left = float(msg.cmds[LEFT_ALIAS].q)
        target_position_right = float(msg.cmds[RIGHT_ALIAS].q)

        target_position_left = np.clip(
            target_position_left, 
            MOTOR_LIMITS[LEFT_ALIAS][0], 
            MOTOR_LIMITS[LEFT_ALIAS][1]
        )
        target_position_right = np.clip(
            target_position_right, 
            MOTOR_LIMITS[RIGHT_ALIAS][0], 
            MOTOR_LIMITS[RIGHT_ALIAS][1]
        )

        self.motor_controler.controlMIT(
            self.motor_left,
            float(left_motor_cmd.kp),
            float(left_motor_cmd.kd),
            target_position_left,   # Желаемая позиция
            float(left_motor_cmd.dq),  # Желаемая скорость
            float(left_motor_cmd.tau)  # Момент 
        )

        self.motor_controler.controlMIT(
            self.motor_right,
            right_motor_cmd.kp,
            right_motor_cmd.kd,
            target_position_right,   # Желаемая позиция
            right_motor_cmd.dq,  # Желаемая скорость
            right_motor_cmd.tau  # Момент
        )


    def timer_callback(self):
        """Периодическая публикация состояния моторов."""
        motor_states_msg = MotorStates()
        motor_left_state_msg = MotorState()
        motor_right_state_msg = MotorState()

#        self.motor_controler.refresh_motor_status(self.motor_left)
#        self.motor_controler.refresh_motor_status(self.motor_right)

        motor_left_state_msg.mode = 0x01
        motor_left_state_msg.q = float(self.motor_left.getPosition())
        motor_left_state_msg.dq = float(self.motor_left.getVelocity())
        motor_left_state_msg.tau_est = float(self.motor_left.getTorque())

        motor_right_state_msg.mode = 0x01
        motor_right_state_msg.q = float(self.motor_right.getPosition())
        motor_right_state_msg.dq = float(self.motor_right.getVelocity())
        motor_right_state_msg.tau_est = float(self.motor_right.getTorque())

        motor_states_msg.states.append(motor_left_state_msg)
        motor_states_msg.states.append(motor_right_state_msg)

        self.publisher.publish(motor_states_msg)
        self.get_logger().debug(f'Message publish: {motor_states_msg}')


    def shutdown(self):
        """Корректное завершение работы."""
        self.motor_controler.disable(self.motor_left)
        self.motor_controler.disable(self.motor_right)
        self.serial_device.close()
        self.get_logger().info("Моторы отключены, соединение закрыто")


def main(args=None):
    rclpy.init(args=args)

    node = PubSubNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')

    except Exception as e:
        node.get_logger().error(f'Exception: {str(e)}')
    
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
