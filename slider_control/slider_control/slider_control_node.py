import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk
import json
import threading
from ament_index_python.packages import get_package_share_directory
import os
import queue

STEP_BUTTON = 0.05
ASK_CONFIGURATION = True
DEFOLT_CONFIGURATION = 1

JOINT_INDEX_H1_WITH_HANDS = {
    'right_hip_roll_joint': 0,
    'right_hip_pitch_joint': 1,
    'right_knee_joint': 2,
    'left_hip_roll_joint': 3,
    'left_hip_pitch_joint': 4,
    'left_knee_joint': 5,
    'torso_joint': 6,
    'left_hip_yaw_joint': 7,
    'right_hip_yaw_joint': 8,
    'IMPACT': 9,
    'left_ankle_joint': 10,
    'right_ankle_joint': 11,
    'right_shoulder_pitch_joint': 12, 
    'right_shoulder_roll_joint': 13,
    'right_shoulder_yaw_joint': 14,
    'right_elbow_joint': 15,
    'left_shoulder_pitch_joint': 16, 
    'left_shoulder_roll_joint': 17,
    'left_shoulder_yaw_joint': 18,
    'left_elbow_joint': 19,
    'right_pinky': 20,
    'right_ring': 21,
    'right_middle': 22,
    'right_index': 23,
    'right_thumb_bend': 24,
    'right_thumb_rotation': 25,
    'left_pinky': 26,
    'left_ring': 27,
    'left_middle': 28,
    'left_index': 29,
    'left_thumb_bend': 30,
    'left_thumb_rotation': 31,
    'left_wrist': 32,
    'right_wrist': 33
    }

LIMITS_OF_JOINTS_UNITREE_H1_WITH_HANDS = {
    0: [-0.43, 0.43],  # right_hip_roll_joint
    1: [-3.14, 2.53],  # right_hip_pitch_joint
    2: [-0.26, 2.05],  # right_knee_joint
    3: [-0.43, 0.43],  # left_hip_roll_joint
    4: [-3.14, 2.53],  # left_hip_pitch_joint
    5: [-0.26, 2.05],   # left_knee_joint
    6: [-2.35, 2.35],  # torso_joint
    7: [-0.43, 0.43],  # left_hip_yaw_joint
    8: [-0.43, 0.43],  # right_hip_yaw_joint
    9: [0.0, 1.0],   # IMPACT
    10: [-0.87, 0.52], # left_ankle_joint
    11: [-0.87, 0.52], # right_ankle_joint
    12: [-2.87, 2.87],   # right_shoulder_pitch_joint
    13: [-3.11, 0.34],   # right_shoulder_roll_joint
    14: [-4.45, 1.3 ],   # right_shoulder_yaw_joint
    15: [-1.25, 2.61],  # right_elbow_joint
    16: [-2.87, 2.87],   # left_shoulder_pitch_joint
    17: [-0.34, 3.11],    # left_shoulder_roll_joint
    18: [-1.3, 4.45],   # left_shoulder_yaw_joint
    19: [-1.25, 2.61],  # left_elbow_joint
    20: [0.0, 1.0],    # right_pinky
    21: [0.0, 1.0],    # right_ring
    22: [0.0, 1.0],    # right_middle
    23: [0.0, 1.0],    # right_index
    24: [0.0, 1.0],    # right_thumb_bend
    25: [0.0, 1.0],    # right_thumb_rotation
    26: [0.0, 1.0],    # left_pinky
    27: [0.0, 1.0],    # left_ring
    28: [0.0, 1.0],    # left_middle
    29: [0.0, 1.0],    # left_index
    30: [0.0, 1.0],    # left_thumb_bend
    31: [0.0, 1.0],    # left_thumb_rotation
    32: [-1.1, 4.58],  # left_wrist
    33: [-6.0, -0.23]  # right_wrist
}

class SliderControlNode(Node):
    def __init__(self):
        super().__init__('slider_control_node')

        self.declare_parameter('ask_configuration_param', ASK_CONFIGURATION)
        self.declare_parameter('configuration_param', DEFOLT_CONFIGURATION)

        self.configuration = self.get_parameter('configuration_param').value

        self.get_logger().info('Chose configuration:\n 1 - arms\n 2 - arms + hands\n 3 - legs + arms\n 4 - legs + arms + hands')

        if self.get_parameter('ask_configuration_param').value == True:
            self.configuration = int(input("Enter the configuration number: "))
        
        if self.configuration == 1:
            self.param_joints = [9, 12, 13, 14, 15, 16, 17, 18, 19]
        elif self.configuration == 2:
            self.param_joints = [9, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33]
        elif self.configuration == 3:
            self.param_joints = [9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
        elif self.configuration == 4:
            self.param_joints = [9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33]
        
        self.publisher_ = self.create_publisher(String, 'positions_to_unitree', 10)
        self.slider_values = {i: 0.0 for i in self.param_joints}
        self.value_labels = {}
        self.min_labels = {}
        self.max_labels = {}
        self.entry_widgets = {}
                
        # Create a queue for thread-safe communication
        self.gui_events = queue.Queue()
        
        # Start ROS spinning in a separate thread
        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()
        
        # Run Tkinter in the main thread
        self.setup_gui()
        self.get_logger().info('slider_control_node started.')


    def ros_spin(self):
        rclpy.spin(self)

    def setup_gui(self):
        try:
            self.root = tk.Tk()
            self.root.title("ROS2 Joint Controller (Humanoid Joints)")
            self.root.geometry("1400x1000")
            
            style = ttk.Style()
            style.theme_use('clam')
            
            # Главный контейнер с 3 колонками (используем только grid)
            main_frame = ttk.Frame(self.root, padding="10")
            main_frame.pack(fill=tk.BOTH, expand=True)
            
            # Левая колонка - правая рука
            right_arm_frame = ttk.Frame(main_frame)
            right_arm_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
            
            # Центральная колонка - картинка и IMPACT
            center_frame = ttk.Frame(main_frame)
            center_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
            
            # Правая колонка - левая рука
            left_arm_frame = ttk.Frame(main_frame)
            left_arm_frame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
            
            # Настройка весов колонок
            main_frame.columnconfigure(0, weight=1)
            main_frame.columnconfigure(1, weight=1)
            main_frame.columnconfigure(2, weight=1)
            main_frame.rowconfigure(0, weight=1)
            
            # Загрузка изображения
            try:
                package_share_dir = get_package_share_directory('slider_control')
                image_path = os.path.join(package_share_dir, 'resource', 'unitree_h1_joints.png')
                original_image = tk.PhotoImage(file=image_path)
                
                # Уменьшаем изображение в 2 раза
                self.image = original_image.subsample(2, 2)  # первый аргумент - по X, второй - по Y
                
                image_label = ttk.Label(center_frame, image=self.image)
                image_label.grid(row=0, column=0, pady=20)

            except Exception as e:
                self.get_logger().warn(f"Could not load image: {str(e)}")
                # Запасной вариант - пустая метка
                ttk.Label(center_frame, text="Robot Image").grid(row=0, column=0, pady=20)
            
            # Создаем элементы управления для суставов
            for joint_id in self.param_joints:
                joint_name = [k for k, v in JOINT_INDEX_H1_WITH_HANDS.items() if v == joint_id][0]
                
                # Выбираем фрейм в зависимости от типа сустава
                if joint_id == 9:  # IMPACT
                    parent = center_frame
                    row = 1  # Под изображением
                elif 'right' in joint_name.lower():
                    parent = right_arm_frame
                    row = self.param_joints.index(joint_id)
                elif 'left' in joint_name.lower():
                    parent = left_arm_frame
                    row = self.param_joints.index(joint_id)
                else:
                    continue
                
                # Создаем фрейм для сустава (используем только grid)
                slider_frame = ttk.Frame(parent, padding="5", relief="ridge")
                slider_frame.grid(row=row, column=0, sticky="ew", pady=2)
                
                # Добавляем элементы управления
                ttk.Label(slider_frame, 
                        text=f'{joint_name}\n(ID: {joint_id})', 
                        font=('Arial', 9, 'bold')).grid(row=0, column=0, columnspan=3)
                
                limits = LIMITS_OF_JOINTS_UNITREE_H1_WITH_HANDS[joint_id]
                ttk.Label(slider_frame, 
                        text=f"Min: {limits[0]:.2f}, Max: {limits[1]:.2f}",
                        font=('Arial', 8)).grid(row=1, column=0, columnspan=3)
                
                # Кнопки и поле ввода
                ttk.Button(slider_frame, text="-", width=3,
                        command=lambda idx=joint_id: self.adjust_value(idx, -STEP_BUTTON)
                        ).grid(row=2, column=0, sticky="w")
                
                self.entry_widgets[joint_id] = ttk.Entry(slider_frame, width=8, 
                                                    font=('Arial', 10, 'bold'),
                                                    justify='center')
                self.entry_widgets[joint_id].insert(0, "0.0")
                self.entry_widgets[joint_id].grid(row=2, column=1)
                self.entry_widgets[joint_id].bind('<Return>', 
                                                lambda e, idx=joint_id: self.entry_changed(idx))
                
                ttk.Button(slider_frame, text="+", width=3,
                        command=lambda idx=joint_id: self.adjust_value(idx, STEP_BUTTON)
                        ).grid(row=2, column=2, sticky="e")
                
                self.value_labels[joint_id] = ttk.Label(slider_frame, 
                                                    text="0.0", 
                                                    font=('Arial', 10, 'bold'))
                self.value_labels[joint_id].grid(row=3, column=0, columnspan=3)
            
            self.root.protocol("WM_DELETE_WINDOW", self.on_close)

            self.root.after(100, self.process_ros_events)
            self.root.mainloop()
            
        except Exception as e:
            self.get_logger().error(f"GUI error: {str(e)}")
            raise
    
    def process_ros_events(self):
        """Process any pending ROS events in the GUI thread"""
        try:
            while True:
                try:
                    # Get any pending GUI events from ROS thread
                    callback, args = self.gui_events.get_nowait()
                    callback(*args)
                except queue.Empty:
                    break
        except Exception as e:
            self.get_logger().error(f"Error processing ROS events: {str(e)}")
        
        # Schedule next check
        self.root.after(100, self.process_ros_events)

    def on_close(self):
        """Handle window close event"""
        self.get_logger().info("Closing GUI window")
        self.root.destroy()
        # Signal ROS to shutdown
        rclpy.shutdown()

    def check_gui(self):
        if not hasattr(self, 'root') or not self.root.winfo_exists():
            self.get_logger().warn("GUI window not available")

    def on_close(self):
        """Handle window close event"""
        self.get_logger().info("Closing GUI window")
        self.root.quit()  # Use quit() instead of destroy() for cleaner shutdown
    def adjust_value(self, index, delta):
        try:
            current_value = float(self.entry_widgets[index].get())
            limits = LIMITS_OF_JOINTS_UNITREE_H1_WITH_HANDS[index]
            new_value = current_value + delta
            
            # Проверка границ
            if new_value < limits[0]:
                new_value = limits[0]
            elif new_value > limits[1]:
                new_value = limits[1]
                
            self.entry_widgets[index].delete(0, tk.END)
            self.entry_widgets[index].insert(0, f"{new_value:.2f}")
            self.entry_changed(index)
        except ValueError:
            self.get_logger().warn(f"Invalid value for joint {index}")

    def entry_changed(self, index):
        try:
            value = float(self.entry_widgets[index].get())
            limits = LIMITS_OF_JOINTS_UNITREE_H1_WITH_HANDS[index]
            
            # Проверка границ
            if value < limits[0]:
                value = limits[0]
                self.entry_widgets[index].delete(0, tk.END)
                self.entry_widgets[index].insert(0, f"{limits[0]:.2f}")
            elif value > limits[1]:
                value = limits[1]
                self.entry_widgets[index].delete(0, tk.END)
                self.entry_widgets[index].insert(0, f"{limits[1]:.2f}")
            
            self.slider_values[index] = value
            self.value_labels[index].config(text=f"{value:.2f}")
            self.publish_values()
        except ValueError:
            self.get_logger().warn(f"Invalid value entered for joint {index}")

    def publish_values(self):
        data_dict = {}
        for k, v in self.slider_values.items():
            if k != 9:
                data_dict[str(k)] = v
        impact = self.slider_values[9]
        msg = String()
        msg.data = json.dumps(data_dict) + f'${impact}'
        self.publisher_.publish(msg)
        self.get_logger().debug(f"Published: {msg.data[:100]}...")


def main(args=None):
    rclpy.init(args=args)
    
    if 'DISPLAY' not in os.environ:
        os.environ['DISPLAY'] = ':0'
        os.environ['XAUTHORITY'] = '/home/user/.Xauthority'
    
    node = SliderControlNode()
    
    # Start ROS in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    try:
        # Run Tkinter mainloop in main thread
        node.root.mainloop()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.root.quit()
        rclpy.shutdown()
        ros_thread.join()
        node.destroy_node()

if __name__ == '__main__':
    main()
