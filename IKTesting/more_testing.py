import ikpy.chain
import ikpy.utils.plot as plot_utils
import numpy as np
import matplotlib.pyplot as plt

# Load the robotic arm chain from URDF
my_chain = ikpy.chain.Chain.from_urdf_file("IKTesting/roar_arm.urdf", active_links_mask=[False, True, True, True, True, True, True, False, False])
print("Chain links:", my_chain.links)

# Get the initial joint angles (all zeros) and the initial end-effector position
initial_joint_angles = [0] * my_chain.active_links_mask  # Set all active joint angles to 0
initial_position = my_chain.forward_kinematics(initial_joint_angles)
initial_position_xyz = initial_position[:3, 3]
print("Initial end-effector position at spawn: ", initial_position_xyz)

# Function to move the end effector to a target (x, y, z) position relative to the initial position
def move(x, y, z):
    # Adjust target position to be relative to the initial end-effector position
    target_position = np.array([x, y, z]) + initial_position_xyz
    target_orientation = [0, -1, 0]  # No specific orientation requirement
    
    # Perform inverse kinematics to get joint angles for the target position
    ik = my_chain.inverse_kinematics(target_position.tolist(), target_orientation, orientation_mode="Y")
    
    # Print the joint angles calculated by IK
    print("The angles of each joint are (in radians): ", ik.tolist())
    print("The angles of each joint are (in degrees): ", np.rad2deg(ik).tolist())

    # Compute the actual position of the end effector using forward kinematics
    computed_position = my_chain.forward_kinematics(ik)
    print("Computed end-effector position after move: ", computed_position[:3, 3])  # Print the x, y, z position

    # Plot the robot arm and target position
    fig, ax = plot_utils.init_3d_figure()
    fig.set_figheight(9)
    fig.set_figwidth(13)

    my_chain.plot(ik, ax, target=target_position)

    plt.xlim(-0.5, 0.5)
    plt.ylim(-0.5, 0.5)
    ax.set_zlim(0, 0.6)
    plt.show()

# Move the end effector to (0, 0, -0.1)
move(0,0, -0.1)