import ikpy.chain
import ikpy.utils.plot as plot_utils
import numpy as np
import math
import matplotlib.pyplot as plt

# Load the URDF file for the robotic arm
my_chain = ikpy.chain.Chain.from_urdf_file("IKTesting/base.urdf", active_links_mask=[True, True, True, True, True, True])

# Function to move the robotic arm to a new target position (x, y, z)
def move(x, y, z):
    target_position = [x, y, z]
    target_orientation = [0, 0, 0]  # Keep orientation constant for now

    # Calculate inverse kinematics for the new target position
    ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Y")
    print("The angles of each joint are:", list(map(lambda r: math.degrees(r), ik.tolist())))

    # Verify by calculating forward kinematics
    computed_position = my_chain.forward_kinematics(ik)
    print("Computed position: %s, original position: %s" % (computed_position[:3, 3], target_position))
    print("Computed position (readable): %s" % ['%.2f' % elem for elem in computed_position[:3, 3]])

    # Update the plot with the new arm position
    fig, ax = plot_utils.init_3d_figure()
    fig.set_figheight(9)
    fig.set_figwidth(13)
    my_chain.plot(ik, ax, target=target_position)

    # Set plot limits for better visualization
    plt.xlim(-0.5, 0.5)
    plt.ylim(-0.5, 0.5)
    ax.set_zlim(0, 0.6)

    # Display the plot
    plt.show()

# Example usage of the move function:
move(0, 0.2, 0.3)
