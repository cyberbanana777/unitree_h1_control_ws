#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

"""
АННОТАЦИЯ
Графический интерфейс для управления суставами робота Unitree H1 через ROS2.
Предоставляет поля для ввода числовых значений и кнопки, позволяющие
менять значение для 34 суставов (ноги, руки, кисти, торс, влиение заданных
координат на поведение системы - impact) с учетом ограничений диапазона
движений. Включает визуализацию схемы нумерации суставов. Зависит от ROS2
(rclpy), tkinter и ресурсов пакета gui_control.

ANNOTATION
A graphical interface (GUI) for controlling the joints of the Unitree H1 robot
via ROS2. Provides fields for entering numeric values and buttons that allow
to change the value for 34 joints (legs, arms, hands, torso, impact) taking
into account range of motion limitations. Includes visualization of the joint
numbering scheme. Depends on ROS2 (rclpy), tkinter, and gui_control package
resources.
"""

import json
import os
import tkinter as tk
from tkinter import ttk

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

STEP_BUTTON = 0.05
REPEAT_DELAY = 300  # milliseconds before repeat starts
REPEAT_INTERVAL = 100  # milliseconds between repeats

JOINT_NAMES = {
    "right_hip_roll_joint": 0,
    "right_hip_pitch_joint": 1,
    "right_knee_joint": 2,
    "left_hip_roll_joint": 3,
    "left_hip_pitch_joint": 4,
    "left_knee_joint": 5,
    "torso_joint": 6,
    "left_hip_yaw_joint": 7,
    "right_hip_yaw_joint": 8,
    "IMPACT": 9,
    "left_ankle_joint": 10,
    "right_ankle_joint": 11,
    "right_shoulder_pitch_joint": 12,
    "right_shoulder_roll_joint": 13,
    "right_shoulder_yaw_joint": 14,
    "right_elbow_joint": 15,
    "left_shoulder_pitch_joint": 16,
    "left_shoulder_roll_joint": 17,
    "left_shoulder_yaw_joint": 18,
    "left_elbow_joint": 19,
    "right_pinky": 20,
    "right_ring": 21,
    "right_middle": 22,
    "right_index": 23,
    "right_thumb_bend": 24,
    "right_thumb_rotation": 25,
    "left_pinky": 26,
    "left_ring": 27,
    "left_middle": 28,
    "left_index": 29,
    "left_thumb_bend": 30,
    "left_thumb_rotation": 31,
    "left_wrist": 32,
    "right_wrist": 33,
}

LIMITS_OF_JOINTS_UNITREE_H1_WITH_HANDS = {
    0: [-0.43, 0.43],  # right_hip_roll_joint
    1: [-3.14, 2.53],  # right_hip_pitch_joint
    2: [-0.26, 2.05],  # right_knee_joint
    3: [-0.43, 0.43],  # left_hip_roll_joint
    4: [-3.14, 2.53],  # left_hip_pitch_joint
    5: [-0.26, 2.05],  # left_knee_joint
    6: [-2.35, 2.35],  # torso_joint
    7: [-0.43, 0.43],  # left_hip_yaw_joint
    8: [-0.43, 0.43],  # right_hip_yaw_joint
    9: [0.0, 1.0],  # IMPACT
    10: [-0.87, 0.52],  # left_ankle_joint
    11: [-0.87, 0.52],  # right_ankle_joint
    12: [-2.87, 2.87],  # right_shoulder_pitch_joint
    13: [-3.11, 0.34],  # right_shoulder_roll_joint
    14: [-4.45, 1.3],  # right_shoulder_yaw_joint
    15: [-1.25, 2.61],  # right_elbow_joint
    16: [-2.87, 2.87],  # left_shoulder_pitch_joint
    17: [-0.34, 3.11],  # left_shoulder_roll_joint
    18: [-1.3, 4.45],  # left_shoulder_yaw_joint
    19: [-1.25, 2.61],  # left_elbow_joint
    20: [0.0, 1.0],  # right_pinky
    21: [0.0, 1.0],  # right_ring
    22: [0.0, 1.0],  # right_middle
    23: [0.0, 1.0],  # right_index
    24: [0.0, 1.0],  # right_thumb_bend
    25: [0.0, 1.0],  # right_thumb_rotation
    26: [0.0, 1.0],  # left_pinky
    27: [0.0, 1.0],  # left_ring
    28: [0.0, 1.0],  # left_middle
    29: [0.0, 1.0],  # left_index
    30: [0.0, 1.0],  # left_thumb_bend
    31: [0.0, 1.0],  # left_thumb_rotation
    32: [-1.1, 4.58],  # left_wrist
    33: [-6.0, -0.23],  # right_wrist
}


class ImageViewer:
    def __init__(self, parent):
        self.top = tk.Toplevel(parent)
        self.top.title("Просмотр изображения")
        self.top.protocol("WM_DELETE_WINDOW", self.close_window)

        package_share_dir = get_package_share_directory("gui_control")
        image_path = os.path.join(
            package_share_dir, "resource", "unitree_h1_joints.png"
        )

        try:
            self.photo = tk.PhotoImage(file=image_path)

            # Reduce display via subsample
            self.photo = self.photo.subsample(2, 2)  # Reduce by 2 times

            self.label = tk.Label(self.top, image=self.photo)
            self.label.pack(padx=10, pady=10)

        except Exception as e:
            error_msg = f"Не удалось загрузить изображение\nПоддерживаются только GIF, PGM/PPM\n{str(e)}"
            tk.Label(self.top, text=error_msg, wraplength=300).pack(
                padx=10, pady=10
            )

        close_btn = ttk.Button(
            self.top, text="Закрыть", command=self.close_window
        )
        close_btn.pack(pady=5)

    def close_window(self):
        self.top.destroy()


class JointControl:
    def __init__(self, parent, joint_id, node):
        self.joint_id = joint_id
        self.node = node
        joint_name = [
            name for name, id in JOINT_NAMES.items() if id == joint_id
        ][0]
        self.frame = ttk.Frame(parent)
        self.repeat_id = None

        # First we create all widgets
        # Joint name label
        label_text = f"{joint_id}: {joint_name}"
        self.label = ttk.Label(
            self.frame, text=label_text, width=30, anchor="w"
        )

        # Buttons and input fields
        self.dec_btn = ttk.Button(self.frame, text="-", width=3)
        self.value_entry = ttk.Entry(self.frame, width=10)
        self.inc_btn = ttk.Button(self.frame, text="+", width=3)
        self.cur_label = ttk.Label(self.frame, text="0.00", width=10)
        self.limits_label = ttk.Label(self.frame, width=15)

        # Setting up grid - removing stretching
        self.frame.columnconfigure(0, weight=0)  # Joint Mark (fixed)
        self.frame.columnconfigure(1, weight=0)  # Button "-" (fixed)
        self.frame.columnconfigure(2, weight=0)  # Input field (fixed)
        self.frame.columnconfigure(3, weight=0)  # Button "+" (fixed)
        self.frame.columnconfigure(4, weight=0)  # Current value (fixed)

        # Create widgets with fixed sizes
        self.label = ttk.Label(
            self.frame, text=label_text, width=25, anchor="w"
        )  # Fixed width
        self.dec_btn = ttk.Button(self.frame, text="-", width=3)
        self.value_entry = ttk.Entry(
            self.frame, width=10
        )  # width in characters
        self.inc_btn = ttk.Button(self.frame, text="+", width=3)
        self.cur_label = ttk.Label(self.frame, text="0.00", width=10)

        # Place without stretching
        self.label.grid(row=0, column=0, padx=2, sticky="w")
        self.dec_btn.grid(row=0, column=1, padx=2)
        self.value_entry.grid(row=0, column=2, padx=2)
        self.inc_btn.grid(row=0, column=3, padx=2)
        self.cur_label.grid(row=0, column=4, padx=2)

        # Hint with limits
        self.limits_label = ttk.Label(self.frame, width=20)
        self.limits_label.grid(
            row=1, column=2, columnspan=3, pady=(0, 5), sticky="w"
        )
        self.limits_label.grid_remove()

        # Rest of the initialization
        self.value_entry.insert(0, "0.0")
        self.setup_bindings()

    def setup_bindings(self):
        self.dec_btn.bind("<ButtonPress-1>", self.start_decrement)
        self.dec_btn.bind("<ButtonRelease-1>", self.stop_repeat)
        self.inc_btn.bind("<ButtonPress-1>", self.start_increment)
        self.inc_btn.bind("<ButtonRelease-1>", self.stop_repeat)
        self.value_entry.bind("<FocusIn>", self.on_entry_focus)
        self.value_entry.bind("<FocusOut>", self.on_entry_focus_out)
        self.value_entry.bind("<Return>", self.update_value)

    def on_entry_focus(self, event=None):
        # Highlight active entry and show limits
        self.value_entry.configure(style="Active.TEntry")
        limits = LIMITS_OF_JOINTS_UNITREE_H1_WITH_HANDS.get(
            self.joint_id, [0, 0]
        )
        self.limits_label.config(
            text=f"[{limits[0]:.2f}, {limits[1]:.2f}]", foreground="blue"
        )

        # Move the tooltip to the right by 10 pixels
        self.limits_label.grid(
            row=1, column=2, pady=(0, 5), padx=(3, 0)
        )  # Added padx=(10, 0)

    def on_entry_focus_out(self, event=None):
        # Remove highlight and hide limits
        self.value_entry.configure(style="TEntry")
        self.limits_label.grid_remove()

    def clamp_value(self, value):
        limits = LIMITS_OF_JOINTS_UNITREE_H1_WITH_HANDS.get(
            self.joint_id, [-float("inf"), float("inf")]
        )
        return max(limits[0], min(value, limits[1]))

    def start_increment(self, event=None):
        self.increment()
        self.repeat_id = self.frame.after(REPEAT_DELAY, self.repeat_increment)

    def start_decrement(self, event=None):  # Добавлен event=None
        self.decrement()
        self.repeat_id = self.frame.after(REPEAT_DELAY, self.repeat_decrement)

    def repeat_increment(self, event=None):
        self.increment()
        self.repeat_id = self.frame.after(
            REPEAT_INTERVAL, self.repeat_increment
        )

    def repeat_decrement(self, event=None):
        self.decrement()
        self.repeat_id = self.frame.after(
            REPEAT_INTERVAL, self.repeat_decrement
        )

    def stop_repeat(self, event=None):  # Добавлен event=None
        if self.repeat_id:
            self.frame.after_cancel(self.repeat_id)
            self.repeat_id = None

    def increment(self):
        current = float(self.value_entry.get())
        new_value = self.clamp_value(current + STEP_BUTTON)
        self.value_entry.delete(0, tk.END)
        self.value_entry.insert(0, f"{new_value:.2f}")
        self.update_joint_value(new_value)

    def decrement(self):
        current = float(self.value_entry.get())
        new_value = self.clamp_value(current - STEP_BUTTON)
        self.value_entry.delete(0, tk.END)
        self.value_entry.insert(0, f"{new_value:.2f}")
        self.update_joint_value(new_value)

    def update_value(self, event=None):
        try:
            new_value = float(self.value_entry.get())
            clamped_value = self.clamp_value(new_value)
            if clamped_value != new_value:
                self.value_entry.delete(0, tk.END)
                self.value_entry.insert(0, f"{clamped_value:.2f}")
            self.update_joint_value(clamped_value)
        except ValueError:
            self.value_entry.delete(0, tk.END)
            self.value_entry.insert(0, "0.0")
            self.update_joint_value(0.0)

    def update_joint_value(self, value):
        self.cur_label.config(text=f"{value:.2f}")
        self.node.publish_all_joints()

    def get_current_value(self):
        try:
            return float(self.cur_label.cget("text"))
        except (ValueError, AttributeError):
            return 0.0


class ImpactControl:
    def __init__(self, parent, node):
        self.node = node
        # Wrap in a LabelFrame like other controls
        self.outer_frame = ttk.LabelFrame(parent, text="IMPACT Control")
        self.frame = ttk.Frame(self.outer_frame)
        self.frame.pack(fill=tk.X, padx=5, pady=5)
        self.repeat_id = None

        # Configure grid with less spacing
        self.frame.columnconfigure(0, weight=1)
        self.frame.columnconfigure(1, weight=1)
        self.frame.columnconfigure(2, weight=1)
        self.frame.columnconfigure(3, weight=1)
        self.frame.columnconfigure(4, weight=1)

        # Label moved closer to entry
        self.label = ttk.Label(
            self.frame, text="9: IMPACT", width=15, anchor="w"
        )
        self.label.grid(row=0, column=0, sticky="w", padx=(0, 5))

        # Decrease button with press-and-hold
        self.dec_btn = ttk.Button(self.frame, text="-", width=3)
        self.dec_btn.grid(row=0, column=1, padx=2)
        self.dec_btn.bind("<ButtonPress-1>", self.start_decrement)
        self.dec_btn.bind("<ButtonRelease-1>", self.stop_repeat)

        # Value entry
        self.value_entry = ttk.Entry(self.frame, width=8)
        self.value_entry.bind("<Return>", self.update_value)
        self.value_entry.grid(row=0, column=2, padx=2)

        # Increase button with press-and-hold
        self.inc_btn = ttk.Button(self.frame, text="+", width=3)
        self.inc_btn.grid(row=0, column=3, padx=2)
        self.inc_btn.bind("<ButtonPress-1>", self.start_increment)
        self.inc_btn.bind("<ButtonRelease-1>", self.stop_repeat)

        # Current value label
        self.cur_label = ttk.Label(self.frame, text="0.00", width=8)
        self.cur_label.grid(row=0, column=4, padx=2)

        self.value_entry.insert(0, "0.0")

    def start_increment(self, event=None):
        self.increment()
        self.repeat_id = self.frame.after(REPEAT_DELAY, self.repeat_increment)

    def start_decrement(self, event=None):  # Добавлен event=None
        self.decrement()
        self.repeat_id = self.frame.after(REPEAT_DELAY, self.repeat_decrement)

    def repeat_increment(self, event=None):
        self.increment()
        self.repeat_id = self.frame.after(
            REPEAT_INTERVAL, self.repeat_increment
        )

    def repeat_decrement(self, event=None):
        self.decrement()
        self.repeat_id = self.frame.after(
            REPEAT_INTERVAL, self.repeat_decrement
        )

    def stop_repeat(self, event=None):  # Добавлен event=None
        if self.repeat_id:
            self.frame.after_cancel(self.repeat_id)
            self.repeat_id = None

    def clamp_value(self, value):
        return max(0.0, min(value, 1.0))

    def increment(self, event=None):
        current = float(self.value_entry.get())
        new_value = self.clamp_value(current + STEP_BUTTON)
        self.value_entry.delete(0, tk.END)
        self.value_entry.insert(0, f"{new_value:.2f}")
        self.update_impact_value(new_value)

    def decrement(self, event=None):
        current = float(self.value_entry.get())
        new_value = self.clamp_value(current - STEP_BUTTON)
        self.value_entry.delete(0, tk.END)
        self.value_entry.insert(0, f"{new_value:.2f}")
        self.update_impact_value(new_value)

    def update_value(self, event=None):
        try:
            new_value = float(self.value_entry.get())
            clamped_value = self.clamp_value(new_value)
            if clamped_value != new_value:
                self.value_entry.delete(0, tk.END)
                self.value_entry.insert(0, f"{clamped_value:.2f}")
            self.update_impact_value(clamped_value)
        except ValueError:
            self.value_entry.delete(0, tk.END)
            self.value_entry.insert(0, "0.0")
            self.update_impact_value(0.0)

    def update_impact_value(self, value):
        self.cur_label.config(text=f"{value:.2f}")
        self.node.publish_all_joints()

    def get_current_value(self):
        try:
            return float(self.cur_label.cget("text"))
        except (ValueError, AttributeError):
            return 0.0


class GUIControlNode(Node):
    def __init__(self):
        super().__init__("gui_control_node")
        self.get_logger().info("Node started")
        self.root = tk.Tk()
        self.root.title("Unitree H1 Control")
        self.running = True
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.publisher = self.create_publisher(
            String, "positions_to_unitree", 10
        )

        # Create custom style for active entry
        style = ttk.Style()
        style.configure("Active.TEntry", fieldbackground="lightblue")

        # Store all joint controls
        self.joint_controls = {}
        self.setup_gui()

    def add_joint_control(self, joint_id, control):
        self.joint_controls[joint_id] = control

    def on_close(self):
        """Обработчик закрытия окна"""
        self.running = False
        self.root.destroy()

    def publish_all_joints(self):
        """Publish all joint values in the required format"""
        msg = String()
        
        # Collect all joint values
        joint_data = {}
        for joint_id, control in self.joint_controls.items():
            if joint_id != 9:  # Skip IMPACT (handled separately)
                joint_data[str(joint_id)] = control.get_current_value()
        
        # Get IMPACT value
        impact_value = 0.0
        if hasattr(self, "impact_control") and self.impact_control:
            impact_value = self.impact_control.get_current_value()
        
        # Format message as {"16": -0.1, ..., "19": 1.65}$1.0
        msg.data = f"{json.dumps(joint_data)}${impact_value:.2f}"
        self.publisher.publish(msg)

    def add_image_viewer_button(self):
        # Create a button at the bottom of the interface
        img_btn_frame = ttk.Frame(self.root)
        img_btn_frame.pack(fill=tk.X, padx=5, pady=5)

        img_btn = ttk.Button(
            img_btn_frame,
            text="Показать изображение",
            command=self.open_image_viewer,
        )
        img_btn.pack(pady=5)

    def open_image_viewer(self):
        # Create a window with an image
        self.image_viewer = ImageViewer(self.root)

    def setup_gui(self):
        # Joint controls notebook
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Arms tab (combined left and right)
        arms_frame = ttk.Frame(notebook)

        # Left Arm controls
        left_arm_frame = ttk.LabelFrame(arms_frame, text="Left Arm")
        for joint_id in [16, 17, 18, 19]:  # left arm joints
            control = JointControl(left_arm_frame, joint_id, self)
            control.frame.pack(fill=tk.X, pady=2)
            self.add_joint_control(joint_id, control)

        # Add separator and torso joint
        ttk.Separator(left_arm_frame, orient="horizontal").pack(
            fill=tk.X, pady=5
        )
        control = JointControl(left_arm_frame, 6, self)  # torso_joint
        control.frame.pack(fill=tk.X, pady=2)
        self.add_joint_control(6, control)

        left_arm_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        # Right Arm controls
        right_arm_frame = ttk.LabelFrame(arms_frame, text="Right Arm")
        for joint_id in [12, 13, 14, 15]:  # right arm joints
            control = JointControl(right_arm_frame, joint_id, self)
            control.frame.pack(fill=tk.X, pady=2)
            self.add_joint_control(joint_id, control)
        right_arm_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)

        notebook.add(arms_frame, text="Arms")

        # Hands tab
        hands_frame = ttk.Frame(notebook)

        # Left Hand section
        left_hand_frame = ttk.LabelFrame(hands_frame, text="Left Hand")
        left_wrist_frame = ttk.LabelFrame(left_hand_frame, text="Left Wrist")
        control = JointControl(left_wrist_frame, 32, self)
        control.frame.pack(fill=tk.X, pady=2)
        self.add_joint_control(32, control)
        left_wrist_frame.pack(fill=tk.X, padx=5, pady=5)

        left_fingers_frame = ttk.LabelFrame(
            left_hand_frame, text="Left Fingers"
        )
        for joint_id in range(26, 32):  # left hand finger joints
            control = JointControl(left_fingers_frame, joint_id, self)
            control.frame.pack(fill=tk.X, pady=1)
            self.add_joint_control(joint_id, control)
        left_fingers_frame.pack(fill=tk.X, padx=5, pady=5)

        left_hand_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        # Right Hand section
        right_hand_frame = ttk.LabelFrame(hands_frame, text="Right Hand")
        right_wrist_frame = ttk.LabelFrame(
            right_hand_frame, text="Right Wrist"
        )
        control = JointControl(right_wrist_frame, 33, self)
        control.frame.pack(fill=tk.X, pady=2)
        self.add_joint_control(33, control)
        right_wrist_frame.pack(fill=tk.X, padx=5, pady=5)

        right_fingers_frame = ttk.LabelFrame(
            right_hand_frame, text="Right Fingers"
        )
        for joint_id in range(20, 26):  # right hand finger joints
            control = JointControl(right_fingers_frame, joint_id, self)
            control.frame.pack(fill=tk.X, pady=1)
            self.add_joint_control(joint_id, control)
        right_fingers_frame.pack(fill=tk.X, padx=5, pady=5)

        right_hand_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)

        notebook.add(hands_frame, text="Hands")

        # Legs tab
        legs_frame = ttk.Frame(notebook)

        # Left Leg controls
        left_leg_frame = ttk.LabelFrame(legs_frame, text="Left Leg")
        for joint_id in [3, 4, 5, 7, 10]:  # left leg joints
            control = JointControl(left_leg_frame, joint_id, self)
            control.frame.pack(fill=tk.X, pady=2)
            self.add_joint_control(joint_id, control)
        left_leg_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        # Right Leg controls
        right_leg_frame = ttk.LabelFrame(legs_frame, text="Right Leg")
        for joint_id in [0, 1, 2, 8, 11]:  # right leg joints
            control = JointControl(right_leg_frame, joint_id, self)
            control.frame.pack(fill=tk.X, pady=2)
            self.add_joint_control(joint_id, control)
        right_leg_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)

        notebook.add(legs_frame, text="Legs")

        # Control frame at bottom
        control_frame = ttk.Frame(self.root)
        control_frame.pack(fill=tk.X, padx=5, pady=5)

        # Create a frame for IMPACT and buttons
        impact_row_frame = ttk.Frame(control_frame)
        impact_row_frame.pack(fill=tk.X, pady=5)

        # Impact control left
        impact_frame = ttk.LabelFrame(impact_row_frame, text="IMPACT")
        impact_frame.pack(side=tk.LEFT)

        # Compact impact control inside impact_frame
        self.setup_compact_impact(impact_frame)

        # Empty frame to fill the space between IMPACT and the button
        spacer = ttk.Frame(impact_row_frame)
        spacer.pack(side=tk.LEFT, expand=True, fill=tk.X)

        # Show image button on the right (pressed to the right edge)
        self.img_btn = ttk.Button(
            impact_row_frame,
            text="Показать схему",
            command=self.open_image_viewer,
            width=17,
        )
        self.img_btn.pack(side=tk.RIGHT, padx=(0, 5))

    def open_image_viewer(self):
        ImageViewer(self.root)

    def setup_compact_impact(self, parent):
        frame = ttk.Frame(parent)
        frame.pack()

        # Compact layout
        ttk.Label(frame, text="Value:").grid(row=0, column=0, padx=2)

        dec_btn = ttk.Button(frame, text="-", width=3)
        dec_btn.grid(row=0, column=1, padx=2)

        value_entry = ttk.Entry(frame, width=6)
        value_entry.grid(row=0, column=2, padx=2)
        value_entry.insert(0, "0.0")

        inc_btn = ttk.Button(frame, text="+", width=3)
        inc_btn.grid(row=0, column=3, padx=2)

        cur_label = ttk.Label(frame, text="0.00", width=6)
        cur_label.grid(row=0, column=4, padx=2)

        # Connect to existing ImpactControl logic
        self.impact_control = ImpactControl(parent, self)
        self.add_joint_control(9, self.impact_control)  # IMPACT has ID 9
        self.impact_control.label.destroy()  # Remove default label
        self.impact_control.dec_btn.destroy()
        self.impact_control.value_entry.destroy()
        self.impact_control.inc_btn.destroy()
        self.impact_control.cur_label.destroy()

        # Connect new compact UI
        dec_btn.bind("<ButtonPress-1>", self.impact_control.start_decrement)
        dec_btn.bind("<ButtonRelease-1>", self.impact_control.stop_repeat)
        inc_btn.bind("<ButtonPress-1>", self.impact_control.start_increment)
        inc_btn.bind("<ButtonRelease-1>", self.impact_control.stop_repeat)
        value_entry.bind("<Return>", self.impact_control.update_value)

        # Keep references
        self.impact_control.value_entry = value_entry
        self.impact_control.cur_label = cur_label
        self.impact_control.dec_btn = dec_btn
        self.impact_control.inc_btn = inc_btn

    def run(self):
        """Основной цикл обработки событий"""
        try:
            while rclpy.ok() and self.running:
                self.root.update()
                rclpy.spin_once(self, timeout_sec=0.01)
                
                # Добавляем небольшую задержку, чтобы не нагружать процессор
                self.root.after(10)
                
        except tk.TclError as e:
            if "application has been destroyed" not in str(e):
                self.get_logger().error(f"Tkinter error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
        finally:
            if rclpy.ok():
                self.destroy_node()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GUIControlNode()
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Программа завершена по запросу пользователя")
    except Exception as e:
        node.get_logger().error(f"Ошибка: {str(e)}")
    finally:
        node.get_logger().info("Node stopped")
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()