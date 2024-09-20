import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ikpy.chain import Chain
import numpy as np

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        self.publisher = self.create_publisher(Float64MultiArray, 'joint_angles', 10)
        self.chain = Chain.from_urdf_file('/home/jover26/Documents/egh400_hmi_robotic_arm/Assembly_description/urdf/roar_arm.urdf')  # Path to your URDF file

    def compute_ik(self, x, y, z):
        # Compute the inverse kinematics
        target_position = [x, y, z]
        joint_angles = self.chain.inverse_kinematics(target_position)
        self.publish_joint_angles(joint_angles)

    def publish_joint_angles(self, joint_angles):
        msg = Float64MultiArray()
        msg.data = joint_angles.tolist()  # Convert to list if necessary
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing joint angles: {joint_angles}')


class IKGUI:
    def __init__(self, ik_node):
        self.ik_node = ik_node
        self.root = tk.Tk()
        self.root.title("IK Control")

        tk.Label(self.root, text="X:").pack()
        self.x_entry = tk.Entry(self.root)
        self.x_entry.pack()

        tk.Label(self.root, text="Y:").pack()
        self.y_entry = tk.Entry(self.root)
        self.y_entry.pack()

        tk.Label(self.root, text="Z:").pack()
        self.z_entry = tk.Entry(self.root)
        self.z_entry.pack()

        self.control_button = tk.Button(self.root, text="Move to XYZ", command=self.move_to_xyz)
        self.control_button.pack()

    def move_to_xyz(self):
        x = float(self.x_entry.get())
        y = float(self.y_entry.get())
        z = float(self.z_entry.get())
        self.ik_node.compute_ik(x, y, z)

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    ik_node = IKNode()
    gui = IKGUI(ik_node)
    gui.run()

    ik_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
