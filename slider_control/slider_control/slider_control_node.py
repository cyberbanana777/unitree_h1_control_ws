import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk
import json
import threading

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
    'right_shoulder_roll_joint': 12,
    'right_shoulder_pitch_joint': 13,
    'right_shoulder_yaw_joint': 14,
    'right_elbow_joint': 15,
    'left_shoulder_roll_joint': 16,
    'left_shoulder_pitch_joint': 17,
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
}

LIMITS_OF_JOINTS_UNITREE_H1_WITH_HANDS = {
    0: [-0.43, 0.43],  # right_hip_roll_joint
    1: [-3.14, 2.53],  # right_hip_pitch_joint
    2: [-0.26, 2.05],  # right_knee_joint
    3: [-0.43, 0.43],  # left_hip_roll_joint
    4: [-3.14, 2.53],  # left_hip_pitch_joint
    5: [0.26, 2.05],   # left_knee_joint
    6: [-2.35, 2.35],  # torso_joint
    7: [-0.43, 0.43],  # left_hip_yaw_joint
    8: [-0.43, 0.43],  # right_hip_yaw_joint
    9: [0.0, 1.0],   # IMPACT
    10: [-0.87, 0.52], # left_ankle_joint
    11: [-0.87, 0.52], # right_ankle_joint
    12: [-1.9, 0.5],   # right_shoulder_pitch_joint
    13: [-2.2, 0.0],   # right_shoulder_roll_joint
    14: [-1.5, 1.3],   # right_shoulder_yaw_joint
    15: [-1.1, 1.65],  # right_elbow_joint
    16: [-1.9, 0.5],   # left_shoulder_pitch_joint
    17: [0.0, 2.2],    # left_shoulder_roll_joint
    18: [-1.3, 1.5],   # left_shoulder_yaw_joint
    19: [-1.1, 1.65],  # left_elbow_joint
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
    31: [0.0, 1.0]     # left_thumb_rotation
}

PARAM_JOINTS = [6, 9, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31]

class SliderControlNode(Node):
    def __init__(self):
        super().__init__('slider_control_node')
        
        self.publisher_ = self.create_publisher(String, 'positions_to_unitree', 10)
        self.slider_values = {i: 0.0 for i in PARAM_JOINTS}
        self.value_labels = {}
        self.min_labels = {}
        self.max_labels = {}
        self.entry_widgets = {}
        
        self.gui_thread = threading.Thread(target=self.setup_gui, daemon=True)
        self.gui_thread.start()
        
        self.timer = self.create_timer(0.1, self.check_gui)
        self.get_logger().info('slider_control_node started.')

    def setup_gui(self):
        try:
            self.root = tk.Tk()
            self.root.title("ROS2 Joint Controller (Humanoid Joints)")
            self.root.geometry("1200x1000")
            
            style = ttk.Style()
            style.theme_use('clam')
            
            main_frame = ttk.Frame(self.root, padding="10")
            main_frame.pack(fill=tk.BOTH, expand=True)
            
            self.counter = 0
            
            for joint_id in PARAM_JOINTS:
                if LIMITS_OF_JOINTS_UNITREE_H1_WITH_HANDS[joint_id][0] is None:
                    continue
                    
                slider_frame = ttk.Frame(main_frame, padding="5", relief="ridge")
                slider_frame.grid(row=self.counter//4, column=self.counter%4, sticky="nsew", padx=5, pady=5)
                self.counter += 1
                
                # Название параметра
                joint_name = [k for k, v in JOINT_INDEX_H1_WITH_HANDS.items() if v == joint_id][0]
                param_label = ttk.Label(slider_frame, 
                                     text=f'{joint_name}\n(ID: {joint_id})', 
                                     font=('Arial', 9, 'bold'))
                param_label.pack()
                
                limits = LIMITS_OF_JOINTS_UNITREE_H1_WITH_HANDS[joint_id]

                # Верхняя метка (максимальное значение)
                max_label = ttk.Label(slider_frame, 
                                   text=f"Min: {limits[0]:.2f}, Max: {limits[1]:.2f}",
                                   font=('Arial', 8))
                max_label.pack()
                self.max_labels[joint_id] = max_label
                
                # Фрейм для элементов управления
                control_frame = ttk.Frame(slider_frame)
                control_frame.pack(fill=tk.BOTH, expand=True, pady=5)
                
                # Кнопка "-"
                minus_btn = ttk.Button(control_frame, text="-", width=3,
                                     command=lambda idx=joint_id: self.adjust_value(idx, -0.1))
                minus_btn.pack(side=tk.LEFT, padx=2)
                
                # Поле ввода
                entry = ttk.Entry(control_frame, width=8, font=('Arial', 10, 'bold'), justify='center')
                entry.insert(0, "0.0")
                entry.pack(side=tk.LEFT, padx=2)
                entry.bind('<Return>', lambda event, idx=joint_id: self.entry_changed(idx))
                entry.bind('<FocusOut>', lambda event, idx=joint_id: self.entry_changed(idx))
                self.entry_widgets[joint_id] = entry
                
                # Кнопка "+"
                plus_btn = ttk.Button(control_frame, text="+", width=3,
                                    command=lambda idx=joint_id: self.adjust_value(idx, 0.1))
                plus_btn.pack(side=tk.LEFT, padx=2)
                
                # Метка текущего значения (для отображения)
                value_label = ttk.Label(slider_frame, 
                                      text="0.0", 
                                      font=('Arial', 10, 'bold'),
                                      anchor="center")
                value_label.pack()
                self.value_labels[joint_id] = value_label
            
            for c in range(4):
                main_frame.columnconfigure(c, weight=1)
            for r in range((len(PARAM_JOINTS) + 3) // 4):
                main_frame.rowconfigure(r, weight=1)
                
            self.root.protocol("WM_DELETE_WINDOW", self.on_close)   
            self.root.mainloop()
        except Exception as e:
            self.get_logger().error(f"GUI error: {str(e)}")

    def check_gui(self):
        if not hasattr(self, 'root') or not self.root.winfo_exists():
            self.get_logger().warn("GUI window not available")

    def on_close(self):
        self.get_logger().info("Closing GUI window")
        self.root.destroy()

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
    
    import os
    if 'DISPLAY' not in os.environ:
        os.environ['DISPLAY'] = ':0'
        os.environ['XAUTHORITY'] = '/home/user/.Xauthority'
    
    node = SliderControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        if hasattr(node, 'root') and node.root.winfo_exists():
            node.root.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
