import numpy as np

# Your rotation matrix
current_orientation = np.array([
    [0.33061076, 0.79133372, -0.51428346],
    [-0.78333627, -0.0738431, -0.61719647],
    [-0.52638466, 0.60690868, 0.59546708]
])

# Extract forward vector (first column of the matrix)
forward_vector = current_orientation[:, 0]

# Calculate the roll angle (in radians)
roll_angle = np.arctan2(forward_vector[1], forward_vector[0])

# Normalize to the range of -pi to pi
if roll_angle > np.pi:
    roll_angle -= 2 * np.pi
elif roll_angle < -np.pi:
    roll_angle += 2 * np.pi

# Convert to degrees for readability
roll_angle_degrees = np.rad2deg(roll_angle)
print("Roll angle in degrees:", roll_angle_degrees)
