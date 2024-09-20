import ikpy.chain
import ikpy.utils.plot as plot_utils

import numpy as np
import time
import math

import matplotlib.pyplot as plt

import ipywidgets as widgets

my_chain = ikpy.chain.Chain.from_urdf_file("IKTesting/roar_arm.urdf", active_links_mask=[False, True, True, True, True, True, True, False, False])
print(my_chain.links)

def normalize_angle(angle):
    return ((angle + np.pi) % (2 * np.pi)) - np.pi

def move(x, y, z):
    target_position = [x, y, z]
    target_orientation = [0, 0, 0]
    ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Y")
    
    normalised_angles = [normalize_angle(angle) for angle in ik.tolist()]
    
    print("The angles of each joint are: ", normalised_angles)

    computed_position = my_chain.forward_kinematics(ik)

    fig, ax = plot_utils.init_3d_figure()
    fig.set_figheight(9)
    fig.set_figwidth(13)

    my_chain.plot(ik, ax, target=target_position)

    plt.xlim(-0.5, 0.5)
    plt.ylim(-0.5, 0.5)
    ax.set_zlim(0, 0.6)
    plt.show()

move(0, 0.4, 0.58)
