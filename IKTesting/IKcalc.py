import ikpy.chain
import ikpy.utils.plot as plot_utils
import numpy as np
import matplotlib.pyplot as plt

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

    # Forward kinematics to get the end effector's orientation
    end_effector_transform = my_chain.forward_kinematics(normalised_angles)

    # Extract the rotation matrix
    rotation_matrix = end_effector_transform[:3, :3]

    # Calculate the roll angle (assuming roll is around the x-axis)
    roll_angle = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
    print("Roll angle (in radians):", roll_angle)
    print("Roll angle (in degrees):", np.rad2deg(roll_angle))

    fig, ax = plot_utils.init_3d_figure()
    fig.set_figheight(9)
    fig.set_figwidth(13)

    my_chain.plot(ik, ax, target=target_position)

    plt.xlim(-0.5, 0.5)
    plt.ylim(-0.5, 0.5)
    ax.set_zlim(0, 0.6)
    plt.show()

    return normalised_angles

move(0, -0.4, -0.1)
