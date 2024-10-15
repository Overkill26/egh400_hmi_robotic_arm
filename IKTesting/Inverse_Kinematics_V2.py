import ikpy.chain
import ikpy.utils.plot as plot_utils
import numpy as np
import matplotlib.pyplot as plt

my_chain = ikpy.chain.Chain.from_urdf_file("IKTesting/roar_arm.urdf", active_links_mask=[False, True, True, True, True, True, True, False, False])
print("Chain links:", my_chain.links)

initial_joint_angles = [0] * my_chain.active_links_mask
initial_position = my_chain.forward_kinematics(initial_joint_angles)
initial_position_xyz = initial_position[:3, 3]
print("Initial end-effector position at spawn: ", initial_position_xyz)

def move(x, y, z):
    target_position = np.array([x, y, z]) + initial_position_xyz
    target_orientation = [0, -1, 0]
    
    ik = my_chain.inverse_kinematics(target_position.tolist(), target_orientation, orientation_mode="Y")
    
    print("The angles of each joint are (in radians): ", ik.tolist())
    print("The angles of each joint are (in degrees): ", np.rad2deg(ik).tolist())

    computed_position = my_chain.forward_kinematics(ik)
    print("Computed end-effector position after move: ", computed_position[:3, 3])

    fig, ax = plot_utils.init_3d_figure()
    fig.set_figheight(9)
    fig.set_figwidth(13)

    my_chain.plot(ik, ax, target=target_position)

    plt.xlim(-0.5, 0.5)
    plt.ylim(-0.5, 0.5)
    ax.set_zlim(0, 0.6)
    plt.show()

move(0,0, -0.1)