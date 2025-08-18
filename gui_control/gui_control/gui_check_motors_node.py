#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from threading import Thread
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.uix.image import Image
from kivy.uix.popup import Popup
from kivy.uix.gridlayout import GridLayout
from kivy.uix.scrollview import ScrollView
from kivy.uix.tabbedpanel import TabbedPanel, TabbedPanelItem
from kivy.graphics import Color, Rectangle
from kivy.core.window import Window
from kivy.metrics import dp
from kivy.lang import Builder
from kivy.clock import Clock
from unitree_go.msg import LowState, MotorStates
from kivy.uix.anchorlayout import AnchorLayout
import h1_info_library as h1

ERROR_THRESHOLD = 10
MAX_JOINT_NUMBER = 20

Builder.load_string("""
<TabbedPanel>:
    tab_width: dp(150)
    tab_height: dp(40)
<TabbedPanelItem>:
    font_size: '16sp'
""")
         

class ROS2MonitorNode(Node):
    def __init__(self, app):
        super().__init__('robot_monitor_node')
        self.app = app
        self.subscription = self.create_subscription(
            LowState,
            'lowstate',
            self.listener_callback,
            10
        )

        self.subscription_fingers_states = self.create_subscription(
            MotorStates, 
            "inspire/state", 
            self.listener_callback_fingers_states, 
            10
        )
        self.get_logger().info("Monitor node initialized")

    def listener_callback(self, msg):
        try:
            Clock.schedule_once(lambda dt: self.app.process_ros_data(msg))
        except Exception as e:
            self.get_logger().error(f"Error in callback: {str(e)}")

    def listener_callback_fingers_states(self, msg):
        try:
            Clock.schedule_once(lambda dt: self.app.process_ros_data(msg))
        except Exception as e:
            self.get_logger().error(f"Error in callback: {str(e)}")


class RobotMonitorApp(App):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.robot_state = h1.RobotState(include_hands_with_fingers=False)
        self.devices = [
            {"id": 1, "name": "Основная камера", "temp": 45.2, "error": 120},
            {"id": 2, "name": "Лидар", "temp": 50.1, "error": 380},
            {"id": 3, "name": "Главный компьютер", "temp": 65.0, "error": 250},
            {"id": 4, "name": "Резервный компьютер", "temp": 40.0, "error": 0},
            {"id": 5, "name": "Датчик окружения", "temp": 35.5, "error": 450}
        ]
        self.ros_node = None
        self.ros_thread = None
        self.motor_labels = {}
        self.device_labels = {}

    def build(self):
        root = BoxLayout(orientation='vertical')
        
        title = Label(text='Мониторинг состояния робота', 
                     size_hint=(1, 0.1),
                     font_size='24sp',
                     bold=True)
        root.add_widget(title)
        
        threshold_label = Label(text=f'Порог ошибки: {ERROR_THRESHOLD}',
                              size_hint=(1, 0.05),
                              color=(1, 0, 0, 1))
        root.add_widget(threshold_label)
        
        tabs = TabbedPanel(do_default_tab=False)
        
        tab_main = TabbedPanelItem(text='Основные суставы')
        main_scroll = ScrollView()
        main_grid = self.create_main_joints_grid()
        main_scroll.add_widget(main_grid)
        tab_main.add_widget(main_scroll)
        tabs.add_widget(tab_main)
        
        tab_devices = TabbedPanelItem(text='Устройства')
        devices_scroll = ScrollView()
        devices_grid = self.create_devices_grid()
        devices_scroll.add_widget(devices_grid)
        tab_devices.add_widget(devices_scroll)
        tabs.add_widget(tab_devices)
        
        root.add_widget(tabs)
        
        buttons_layout = BoxLayout(size_hint=(1, 0.1))
        btn_scheme = Button(text='Показать схему робота')
        btn_scheme.bind(on_press=self.show_scheme)
        buttons_layout.add_widget(btn_scheme)
        
        btn_exit = Button(text='Выход')
        btn_exit.bind(on_press=self.exit_app)
        buttons_layout.add_widget(btn_exit)
        
        root.add_widget(buttons_layout)
        
        self.main_grid = main_grid
        self.devices_grid = devices_grid
        
        self.start_ros_node()
        
        return root

    def start_ros_node(self):
        def run_ros_node():
            rclpy.init()
            self.ros_node = ROS2MonitorNode(self)
            rclpy.spin(self.ros_node)
            rclpy.shutdown()
        
        self.ros_thread = Thread(target=run_ros_node, daemon=True)
        self.ros_thread.start()

    def create_main_joints_grid(self):
        grid = GridLayout(cols=5, spacing=dp(10), padding=dp(10),
                        size_hint=(1, None), row_default_height=dp(40))
        grid.bind(minimum_height=grid.setter('height'))
        
        # Заголовки столбцов
        grid.add_widget(Label(text='ID', bold=True))
        grid.add_widget(Label(text='Название', bold=True))
        grid.add_widget(Label(text='Температура', bold=True))
        grid.add_widget(Label(text='Ошибка', bold=True))
        grid.add_widget(Label(text='Статус', bold=True))

        # Получаем список моторов в правильном порядке
        motors = [self.robot_state.motors_info[i] for i in self.robot_state.get_all_abs_indices()]
        
        for motor in motors:
            grid.add_widget(Label(text=str(motor.abs_index)))
            grid.add_widget(Label(text=motor.name_joint))
            
            temp_label = Label(text=f"{motor.temperature}°C")
            grid.add_widget(temp_label)
            
            error_label = Label(text=str(motor.error))
            grid.add_widget(error_label)
            
            # Контейнер для индикатора с центрированием
            indicator_box = AnchorLayout(anchor_x='center', anchor_y='center', 
                                    size_hint=(1, 1))
            indicator = Label(size_hint=(None, None), size=(dp(30), dp(30)))
            indicator_box.add_widget(indicator)
            
            # Устанавливаем цвет фона
            with indicator.canvas.before:
                Color(1, 0, 0, 1) if motor.error >= ERROR_THRESHOLD else Color(0, 1, 0, 1)
                Rectangle(pos=indicator.pos, size=indicator.size)
            
            # Функция для обновления позиции прямоугольника
            def update_rect(instance, value, motor=motor, indicator=indicator):
                instance.canvas.before.clear()
                with instance.canvas.before:
                    Color(1, 0, 0, 1) if motor.error >= ERROR_THRESHOLD else Color(0, 1, 0, 1)
                    Rectangle(pos=indicator.pos, size=indicator.size)
            
            indicator.bind(pos=update_rect, size=update_rect)
            grid.add_widget(indicator_box)
            
            self.motor_labels[motor.abs_index] = {
                'temp': temp_label,
                'error': error_label,
                'indicator': indicator
            }
        
        return grid

    def create_devices_grid(self):
        grid = GridLayout(cols=5, spacing=dp(10), padding=dp(10),
                        size_hint=(1, None), row_default_height=dp(40))
        grid.bind(minimum_height=grid.setter('height'))
        
        # Заголовки столбцов
        grid.add_widget(Label(text='ID', bold=True))
        grid.add_widget(Label(text='Название', bold=True))
        grid.add_widget(Label(text='Температура', bold=True))
        grid.add_widget(Label(text='Ошибка', bold=True))
        grid.add_widget(Label(text='Статус', bold=True))

        for device in self.devices:
            grid.add_widget(Label(text=str(device['id'])))
            grid.add_widget(Label(text=device['name']))
            
            temp_label = Label(text=f"{device['temp']}°C")
            grid.add_widget(temp_label)
            
            error_label = Label(text=str(device['error']))
            grid.add_widget(error_label)
            
            # Контейнер для индикатора с центрированием
            indicator_box = AnchorLayout(anchor_x='center', anchor_y='center', 
                                    size_hint=(1, 1), padding=(dp(10), 0))
            indicator = Label(size_hint=(None, None), size=(dp(30), dp(30)))
            indicator_box.add_widget(indicator)
            
            # Устанавливаем цвет фона
            with indicator.canvas.before:
                Color(1, 0, 0, 1) if device['error'] >= ERROR_THRESHOLD else Color(0, 1, 0, 1)
                self.rect = Rectangle(pos=indicator.pos, size=indicator.size)
            
            # Функция для обновления позиции прямоугольника
            def update_rect(instance, value, device=device, indicator=indicator):
                instance.canvas.before.clear()
                with instance.canvas.before:
                    Color(1, 0, 0, 1) if device['error'] >= ERROR_THRESHOLD else Color(0, 1, 0, 1)
                    Rectangle(pos=indicator.pos, size=indicator.size)
            
            indicator.bind(pos=update_rect, size=update_rect)
            grid.add_widget(indicator_box)
            
            self.device_labels[device['id']] = {
                'temp': temp_label,
                'error': error_label,
                'indicator': indicator
            }
        
        return grid

    def process_ros_data(self, msg):
        for i in self.robot_state.get_all_abs_indices():
            msg_indx = self.robot_state.get_msg_index_by_abs_index(i)
            motor_type = self.robot_state.get_type_by_abs_index(i)  # Исправленное имя метода
            
            if motor_type == 'default_body_joint':  # Используем 'main_joint' вместо 'default_body_joint'
                msg_joint = msg.motor_state[msg_indx]
            elif motor_type == 'finger_joint':
                msg_joint = msg.states[msg_indx]
            else:
                continue  # Пропускаем неизвестные типы

            # Приводим температуру к int
            temperature = int(msg_joint.temperature)
            error = msg_joint.lost
            
            self.robot_state.update_temperature_by_abs_index(i, temperature)
            self.robot_state.update_error_by_abs_index(i, error)

            if i in self.motor_labels:
                labels = self.motor_labels[i]
                labels['temp'].text = f"{temperature}°C"
                labels['error'].text = str(error)
                
                # Обновляем индикатор
                indicator = labels['indicator']
                indicator.canvas.before.clear()
                with indicator.canvas.before:
                    Color(1, 0, 0, 1) if error >= ERROR_THRESHOLD else Color(0, 1, 0, 1)
                    Rectangle(pos=indicator.pos, size=indicator.size)

    def show_scheme(self, instance):
        content = BoxLayout(orientation='vertical')
        img = Image(size_hint=(1, 0.8))
        btn_layout = BoxLayout(size_hint=(1, 0.2))
        btn_close = Button(text='Закрыть')
        btn_layout.add_widget(btn_close)
        content.add_widget(img)
        content.add_widget(btn_layout)
        
        popup = Popup(title='Схема робота',
                     content=content,
                     size_hint=(0.9, 0.9))
        btn_close.bind(on_press=popup.dismiss)
        popup.open()

    def exit_app(self, instance):
        if self.ros_node:
            self.ros_node.destroy_node()
        App.get_running_app().stop()

def main(args=None):
    RobotMonitorApp().run()

if __name__ == '__main__':
    main()