import ikpy.chain
import ikpy.utils.plot as plot_utils

import numpy as np
import time
import math

import matplotlib.pyplot as plt

import ipywidgets as widgets

my_chain = ikpy.chain.Chain.from_urdf_file("IKTesting/roar_arm.urdf", active_links_mask=[False, True, True, True, True, True, True, False, False])

def move(x, y, z):
    target_position = [x, y, z]
    target_orientation = [0, 0, 0]
    ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Y")
    
    normalised_angles = [((angle + np.pi) % (2 * np.pi)) - np.pi for angle in ik.tolist()]
    return normalised_angles

print("Inverse Kinematics Calcualtions Starting")
print(move(0, 0.4, 0.58))
print(move(0, 0.4, 0.57))
print(move(0, 0.4, 0.56))
print(move(0, -0.4, 0.56))
print(move(0, 0.4, 0.56))
print(move(0, -0.4, 0.56)) 
