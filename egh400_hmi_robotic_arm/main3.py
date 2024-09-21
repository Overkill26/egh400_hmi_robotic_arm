import tkinter as tk
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import ikpy.chain
import time

# Initialize the chain for inverse kinematics
my_chain = ikpy.chain.Chain.from_urdf_file("IKTesting/roar_arm.urdf", active_links_mask=[False, False, True, True, True, True, True, False, False])

initial_joint_angles = [0] * my_chain.active_links_mask
initial_position = my_chain.forward_kinematics(initial_joint_angles)
initial_position_xyz = initial_position[:3, 3]

# Global variable for target orientation
target_orientation = [0, -1, 0]

class RoboticArmControl(Node):
    def __init__(self):
        super().__init__("robotic_arm_control")
        self.publisher_ = self.create_publisher(JointState, "joint_command", 10)

        self.joint_state = JointState()
        self.joint_state.name = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "leftgripper", "rightgripper"]
        self.joint_state.position = [0.0] * len(self.joint_state.name)

        self.last_published_positions = list(self.joint_state.position)
        self.max_velocity = np.deg2rad(120)  # Maximum velocity in radians/second
        self.time_step = 0.05  # Time step for updates (in seconds)

    def set_joint_positions(self, target_positions):
        if not all(isinstance(pos, float) for pos in target_positions):
            raise TypeError("Positions must be a list of floats.")
        
        constrained_positions = [np.clip(pos, -np.pi, np.pi) if i < 7 else np.clip(pos, 0, 0.5) for i, pos in enumerate(target_positions)]

        # Interpolate the positions with velocity constraints
        current_positions = np.array(self.last_published_positions)
        target_positions = np.array(constrained_positions)
        while not np.allclose(current_positions, target_positions, atol=1e-3):
            # Calculate the difference and the step based on max velocity
            position_diff = target_positions - current_positions
            step = np.clip(position_diff, -self.max_velocity * self.time_step, self.max_velocity * self.time_step)
            current_positions += step

            # Update and publish joint positions
            self.joint_state.position = current_positions.tolist()
            self.joint_state.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.joint_state)

            # Store the last published positions
            self.last_published_positions = current_positions.tolist()

            # Wait for the next update
            time.sleep(self.time_step)

# Function to calculate joint angles using inverse kinematics
def move(x, y, z):
    target_position = np.array([x, y, z]) + initial_position_xyz
    
    # Calculate the inverse kinematics
    ik = my_chain.inverse_kinematics(target_position.tolist(), target_orientation, orientation_mode="Z")
    
    normalised_angles = [((angle + np.pi) % (2 * np.pi)) - np.pi for angle in ik.tolist()]
    normalised_angles = normalised_angles[1:]  # Skip joint0
    normalised_angles.append(0.0)  # Gripper position

    # Calculate the transformation matrix from the previous joints to the end effector
    end_effector_transform = my_chain.forward_kinematics(normalised_angles).tolist()
    
    # Compute the roll angle based on the desired orientation
    direction_vector = np.array([end_effector_transform[0][2], end_effector_transform[1][2], end_effector_transform[2][2]])
    flat_roll_angle = np.arctan2(direction_vector[1], direction_vector[0])
    print(flat_roll_angle)

    # Set joint 6 to maintain flatness
    #normalised_angles[5] = flat_roll_angle

    print("Roll angle (joint 6) for flat orientation: ", np.rad2deg(normalised_angles[5]))
    print("The angles of each joint are: ", normalised_angles)
    return normalised_angles

def update_joint_positions():
    joint_positions = [np.deg2rad(float(slider.get())) for slider in sliders[:-1]]
    gripper_position = float(sliders[-1].get())
    joint_positions.append(gripper_position)
    joint_positions.append(gripper_position)
    ros2_publisher.set_joint_positions(joint_positions)

def reset_sliders():
    for slider in sliders:
        slider.set(0)
    update_joint_positions()

def move_end_effector_xyz():
    try:
        x = float(x_entry.get())
        y = float(y_entry.get())
        z = float(z_entry.get())
        joint_positions = move(x, y, z)

        # Update sliders with the new joint positions
        for i, slider in enumerate(sliders[:-1]):  # Exclude gripper from the sliders
            slider.set(np.rad2deg(joint_positions[i]))  # Convert radians to degrees for the slider
        # Set the gripper position
        gripper_position = float(sliders[-1].get())
        ros2_publisher.set_joint_positions(joint_positions + [gripper_position, gripper_position])  # Add gripper positions

    except ValueError:
        print("Invalid input for X, Y, Z.")

# Functions to set target orientation
def set_orientation_flat():
    global target_orientation
    target_orientation = [0, -1, 0]
    print("Orientation set to flat.")

def set_orientation_up():
    global target_orientation
    target_orientation = [0, 1, 0]
    print("Orientation set to up.")

def set_orientation_left():
    global target_orientation
    target_orientation = [-1, 0, 0]
    print("Orientation set to left.")

def set_orientation_right():
    global target_orientation
    target_orientation = [1, 0, 0]
    print("Orientation set to right.")

def create_gui():
    root = tk.Tk()
    root.title("Robotic Arm Control GUI")

    sliders_frame = tk.Frame(root)
    sliders_frame.pack(side=tk.LEFT, padx=10, pady=10)

    global sliders
    sliders = []
    joint_names = [
        "Base Joint", "Shoulder Pan", "Shoulder Lift", "Elbow Lift", 
        "Elbow Pan", "Wrist Lift", "Wrist Pan"
    ]
    
    # Add Base Joint slider
    label = tk.Label(sliders_frame, text=joint_names[0].upper())
    label.pack()
    base_joint_slider = tk.Scale(sliders_frame, from_=-180, to=180, resolution=0.5, orient=tk.HORIZONTAL, length=400, command=lambda value: update_joint_positions())
    base_joint_slider.pack()
    sliders.append(base_joint_slider)

    for i in range(1, 7):  # Add sliders for joint1 to joint6
        label = tk.Label(sliders_frame, text=joint_names[i].upper())
        label.pack()

        slider = tk.Scale(sliders_frame, from_=-180, to=180, resolution=0.5, orient=tk.HORIZONTAL, length=400, command=lambda value, i=i: update_joint_positions())
        slider.pack()
        sliders.append(slider)

    label = tk.Label(sliders_frame, text="Grippers (Left & Right)")
    label.pack()

    gripper_slider = tk.Scale(sliders_frame, from_=0, to=0.03, resolution=0.0005, orient=tk.HORIZONTAL, length=400, command=lambda value: update_joint_positions())
    gripper_slider.pack()
    sliders.append(gripper_slider)

    effector_frame = tk.Frame(root)
    effector_frame.pack(side=tk.LEFT, padx=20, pady=10)

    effector_label = tk.Label(effector_frame, text="End Effector Control (XYZ)", font=("Arial", 14))
    effector_label.grid(row=0, column=0, columnspan=2, pady=10)

    # XYZ input fields
    tk.Label(effector_frame, text="X:").grid(row=1, column=0)
    global x_entry
    x_entry = tk.Entry(effector_frame)
    x_entry.grid(row=1, column=1)

    tk.Label(effector_frame, text="Y:").grid(row=2, column=0)
    global y_entry
    y_entry = tk.Entry(effector_frame)
    y_entry.grid(row=2, column=1)

    tk.Label(effector_frame, text="Z:").grid(row=3, column=0)
    global z_entry
    z_entry = tk.Entry(effector_frame)
    z_entry.grid(row=3, column=1)

    move_button = tk.Button(effector_frame, text="Move End Effector", command=move_end_effector_xyz)
    move_button.grid(row=4, column=0, columnspan=2, pady=10)

    reset_button = tk.Button(effector_frame, text="Reset Sliders", command=reset_sliders)
    reset_button.grid(row=5, column=0, columnspan=2, pady=10)

    # Orientation buttons
    orientation_frame = tk.Frame(root)
    orientation_frame.pack(side=tk.LEFT, padx=20, pady=10)

    flat_button = tk.Button(orientation_frame, text="y-", command=set_orientation_flat)
    flat_button.pack(pady=5)

    up_button = tk.Button(orientation_frame, text="y+", command=set_orientation_up)
    up_button.pack(pady=5)

    left_button = tk.Button(orientation_frame, text="x-", command=set_orientation_left)
    left_button.pack(pady=5)

    right_button = tk.Button(orientation_frame, text="x+", command=set_orientation_right)
    right_button.pack(pady=5)

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
