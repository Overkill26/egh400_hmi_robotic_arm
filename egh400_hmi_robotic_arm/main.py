import tkinter as tk
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class ROS2Publisher(Node):
    def __init__(self):
        super().__init__('ros2_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_command', 10)
        self.joint_state = JointState()
        self.joint_state.name = [
            "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
            "panda_joint5", "panda_joint6", "panda_joint7", "panda_finger_joint1",
            "panda_finger_joint2"
        ]
        self.joint_state.position = [0.0] * len(self.joint_state.name)

    def update_joint(self, joint_indices, position):
        try:
            position = float(position)
            for idx in joint_indices:
                self.joint_state.position[idx] = position
            self.publisher_.publish(self.joint_state)
        except ValueError:
            self.get_logger().error("Invalid position value: {}".format(position))

class GUI(tk.Tk):
    def __init__(self, ros2_publisher):
        super().__init__()
        self.ros2_publisher = ros2_publisher

        self.title("Control Panel")
        self.geometry("800x600")

        self.joint_names = self.ros2_publisher.joint_state.name
        self.custom_labels = [
            "Joint 1", "Joint 2", "Joint 3", "Joint 4",
            "Joint 5", "Joint 6", "Joint 7", "Finger Joint 1",
            "Finger Joint 2"
        ]
        self.sliders = []

        self.create_joint_sliders()

        self.finger_state = tk.StringVar(value="Closed")
        self.finger_button = tk.Button(self, text="Open Finger Joints", command=self.toggle_fingers)
        self.finger_button.grid(row=len(self.joint_names), column=0, pady=20, sticky="ew")

        self.create_end_effector_controls()

        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=1)
        self.grid_rowconfigure(3, weight=1)
        self.grid_rowconfigure(4, weight=1)
        self.grid_rowconfigure(5, weight=1)

    def create_joint_sliders(self):
        for i, joint_name in enumerate(self.joint_names):
            if joint_name in ["panda_finger_joint1", "panda_finger_joint2"]:
                continue
            
            frame = tk.Frame(self)
            frame.grid(row=i, column=0, pady=5, sticky="ew")

            label = tk.Label(frame, text=self.custom_labels[i])
            label.pack(side=tk.LEFT, padx=5)

            slider = tk.Scale(frame, from_=-180, to=180, orient='horizontal', label=f"{self.custom_labels[i]} (degrees)")
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
            slider.bind("<Motion>", lambda e, idx=i: self.on_slider_change(e, idx))
            
            self.sliders.append(slider)

    def create_end_effector_controls(self):
        frame = tk.Frame(self)
        frame.grid(row=0, column=1, rowspan=6, padx=20, pady=10, sticky="nswe")

        title_label = tk.Label(frame, text="End Effector Controls", font=('Arial', 14, 'bold'))
        title_label.grid(row=0, column=0, columnspan=2, pady=10)

        cross_frame = tk.Frame(frame)
        cross_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        button_size = 5

        tk.Button(cross_frame, text="←", width=button_size, height=button_size, command=self.move_x_left).grid(row=1, column=0, padx=10, pady=10)
        tk.Button(cross_frame, text="→", width=button_size, height=button_size, command=self.move_x_right).grid(row=1, column=2, padx=10, pady=10)
        tk.Button(cross_frame, text="↑", width=button_size, height=button_size, command=self.move_y_up).grid(row=0, column=1, padx=10, pady=10)
        tk.Button(cross_frame, text="↓", width=button_size, height=button_size, command=self.move_y_down).grid(row=2, column=1, padx=10, pady=10)

        z_frame = tk.Frame(frame)
        z_frame.grid(row=1, column=1, padx=10, pady=10, sticky="nsew")

        tk.Button(z_frame, text="↑", width=button_size, height=button_size, command=self.move_z_up).pack(side=tk.TOP, pady=10)
        tk.Button(z_frame, text="↓", width=button_size, height=button_size, command=self.move_z_down).pack(side=tk.BOTTOM, pady=10)

        frame.grid_rowconfigure(1, weight=1)
        frame.grid_rowconfigure(2, weight=1)
        frame.grid_rowconfigure(3, weight=1)
        frame.grid_rowconfigure(4, weight=1)
        frame.grid_rowconfigure(5, weight=1)
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_columnconfigure(1, weight=1)

    def on_slider_change(self, event, joint_index):
        degrees = self.sliders[joint_index].get()
        radians = math.radians(degrees)
        self.ros2_publisher.update_joint([joint_index], radians)

    def toggle_fingers(self):
        if self.finger_state.get() == "Closed":
            self.finger_state.set("Open")
            finger_position = 0.5
            self.finger_button.config(text="Close Finger Joints")
        else:
            self.finger_state.set("Closed")
            finger_position = 0.0
            self.finger_button.config(text="Open Finger Joints")

        finger_joint_indices = [
            self.ros2_publisher.joint_state.name.index("panda_finger_joint1"),
            self.ros2_publisher.joint_state.name.index("panda_finger_joint2")
        ]
        self.ros2_publisher.update_joint(finger_joint_indices, finger_position)

    def move_x_left(self):
        print("Move X Left")

    def move_x_right(self):
        print("Move X Right")

    def move_y_up(self):
        print("Move Y Up")

    def move_y_down(self):
        print("Move Y Down")

    def move_z_up(self):
        print("Move Z Up")

    def move_z_down(self):
        print("Move Z Down")

def main(args=None):
    rclpy.init(args=args)
    ros2_publisher = ROS2Publisher()

    gui = GUI(ros2_publisher)
    gui.mainloop()

    ros2_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
