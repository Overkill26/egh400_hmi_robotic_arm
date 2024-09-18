#!/usr/bin/env python3

import tkinter as tk
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class RoboticArmControl(Node):
    def __init__(self):
        super().__init__("robotic_arm_control")
        self.publisher_ = self.create_publisher(JointState, "joint_command", 10)

        # Initialize JointState message
        self.joint_state = JointState()
        self.joint_state.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.joint_state.position = [0.0] * len(self.joint_state.name)

        # Track the last published positions
        self.last_published_positions = list(self.joint_state.position)

    def set_joint_positions(self, positions):
        # Ensure positions is a list of floats
        if not all(isinstance(pos, float) for pos in positions):
            raise TypeError("Positions must be a list of floats.")
        
        # Constrain positions to be between -π and π
        constrained_positions = [np.clip(pos, -np.pi, np.pi) for pos in positions]
        
        # Publish only if the positions have changed
        if constrained_positions != self.last_published_positions:
            self.joint_state.position = constrained_positions
            self.joint_state.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.joint_state)
            self.last_published_positions = constrained_positions

def update_joint_positions():
    # Convert slider values from degrees to radians
    joint_positions = [np.deg2rad(float(slider.get())) for slider in sliders]
    ros2_publisher.set_joint_positions(joint_positions)

def move_end_effector(direction):
    print(f"End effector moved {direction}")

def create_gui():
    root = tk.Tk()
    root.title("Robotic Arm Control GUI")

    # Frame for sliders
    sliders_frame = tk.Frame(root)
    sliders_frame.pack(side=tk.LEFT, padx=10, pady=10)

    global sliders
    sliders = []
    for i in range(6):  # Only 6 joints for now
        label = tk.Label(sliders_frame, text=f"Joint {i+1}")
        label.pack()

        slider = tk.Scale(sliders_frame, from_=-180, to=180, orient=tk.HORIZONTAL, command=lambda value, i=i: update_joint_positions())
        slider.pack()
        sliders.append(slider)

    # Frame for End Effector Control
    effector_frame = tk.Frame(root)
    effector_frame.pack(side=tk.LEFT, padx=20, pady=10)

    # Title label for End Effector Control
    effector_label = tk.Label(effector_frame, text="End Effector Control", font=("Arial", 14))
    effector_label.grid(row=0, column=0, columnspan=2, pady=10)

    # Frame for XY control (D-Pad style)
    xy_frame = tk.Frame(effector_frame)
    xy_frame.grid(row=1, column=0, padx=10)

    # Arrow buttons for x and y movement
    up_button = tk.Button(xy_frame, text="↑", command=lambda: move_end_effector("Up"))
    up_button.grid(row=0, column=1)

    left_button = tk.Button(xy_frame, text="←", command=lambda: move_end_effector("Left"))
    left_button.grid(row=1, column=0)

    right_button = tk.Button(xy_frame, text="→", command=lambda: move_end_effector("Right"))
    right_button.grid(row=1, column=2)

    down_button = tk.Button(xy_frame, text="↓", command=lambda: move_end_effector("Down"))
    down_button.grid(row=2, column=1)

    # Frame for Z control (up and down) to the right of the D-Pad
    z_frame = tk.Frame(effector_frame)
    z_frame.grid(row=1, column=1, padx=20)

    # Arrow buttons for z-axis movement
    z_up_button = tk.Button(z_frame, text="Z ↑", command=lambda: move_end_effector("Z Up"))
    z_up_button.pack(pady=5)

    z_down_button = tk.Button(z_frame, text="Z ↓", command=lambda: move_end_effector("Z Down"))
    z_down_button.pack(pady=5)

    # Run the Tkinter event loop
    root.mainloop()

def start_ros2_node():
    rclpy.init()
    global ros2_publisher
    ros2_publisher = RoboticArmControl()
    rclpy.spin(ros2_publisher)
    ros2_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    ros2_thread = threading.Thread(target=start_ros2_node, daemon=True)
    ros2_thread.start()

    create_gui()
