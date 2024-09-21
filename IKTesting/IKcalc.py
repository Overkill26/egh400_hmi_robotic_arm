import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import ikpy.chain

# Initialize the chain for inverse kinematics
my_chain = ikpy.chain.Chain.from_urdf_file("IKTesting/roar_arm.urdf", active_links_mask=[False, False, True, True, True, True, True, False, False])

initial_joint_angles = [0] * my_chain.active_links_mask
initial_position = my_chain.forward_kinematics(initial_joint_angles)
initial_position_xyz = initial_position[:3, 3]

# Global variable for target orientation
target_orientation = [0, -1, 0]

# Function to calculate joint angles using inverse kinematics
def move(x, y, z):
    target_position = np.array([x, y, z]) + initial_position_xyz
    
    # Calculate the inverse kinematics
    ik = my_chain.inverse_kinematics(target_position.tolist(), target_orientation, orientation_mode="Z")
    
    normalised_angles = [((angle + np.pi) % (2 * np.pi)) - np.pi for angle in ik.tolist()]
    normalised_angles = normalised_angles[1:]  # Skip joint0
    normalised_angles.append(0.0)  # Gripper position

    print("Joint angles (in radians):", normalised_angles)
    print("Joint angles (in degrees):", [np.rad2deg(angle) for angle in normalised_angles])
    return normalised_angles

def main():
    rclpy.init()
    print("Robotic Arm Control - Enter positions in X, Y, Z.")

    while True:
        try:
            x = float(input("Enter X position (or type 'exit' to quit): "))
            y = float(input("Enter Y position: "))
            z = float(input("Enter Z position: "))
            move(x, y, z)
        except ValueError:
            print("Invalid input. Please enter numeric values for X, Y, Z.")
        except KeyboardInterrupt:
            print("\nExiting...")
            break

    rclpy.shutdown()

if __name__ == "__main__":
    main()
