import tkinter as tk
from tkinter import ttk

def update_slider_value(axis, value):
    print(f"Axis {axis}: {value} degrees")

def move_x(direction):
    print(f"Move X: {direction}")

def move_y(direction):
    print(f"Move Y: {direction}")

def move_z(direction):
    print(f"Move Z: {direction}")

root = tk.Tk()
root.title("6-Axis Robotic Arm Controller")

title_label = ttk.Label(root, text="6-Axis Robotic Arm Controller", font=("Helvetica", 16))
title_label.pack(pady=10)

slider_frame = ttk.Frame(root)
slider_frame.pack(side=tk.LEFT, padx=20, pady=20)

sliders = []
for i in range(1, 7):
    axis_label = ttk.Label(slider_frame, text=f"Axis {i}:")
    axis_label.grid(row=i-1, column=0, padx=10, pady=5)

    slider = ttk.Scale(slider_frame, from_=-180, to=180, orient="horizontal", length=400,
                       command=lambda value, axis=i: update_slider_value(axis, int(float(value))))
    slider.grid(row=i-1, column=1, padx=10, pady=5)
    sliders.append(slider)

control_frame = ttk.Frame(root)
control_frame.pack(side=tk.LEFT, padx=20, pady=20)

end_effector_label = ttk.Label(control_frame, text="End Effector Controller", font=("Helvetica", 14))
end_effector_label.grid(row=0, column=0, columnspan=2, pady=10)

dpad_frame = ttk.Frame(control_frame)
dpad_frame.grid(row=1, column=0, padx=10)

up_button = ttk.Button(dpad_frame, text="Y ↑", command=lambda: move_y("up"))
up_button.grid(row=0, column=1, padx=10, pady=10)

left_button = ttk.Button(dpad_frame, text="X ←", command=lambda: move_x("left"))
left_button.grid(row=1, column=0, padx=10, pady=10)

right_button = ttk.Button(dpad_frame, text="X →", command=lambda: move_x("right"))
right_button.grid(row=1, column=2, padx=10, pady=10)

down_button = ttk.Button(dpad_frame, text="Y ↓", command=lambda: move_y("down"))
down_button.grid(row=2, column=1, padx=10, pady=10)

z_frame = ttk.Frame(control_frame)
z_frame.grid(row=1, column=1, padx=20)

z_up_button = ttk.Button(z_frame, text="Z ↑", command=lambda: move_z("up"))
z_up_button.grid(row=0, column=0, padx=10, pady=10)

z_down_button = ttk.Button(z_frame, text="Z ↓", command=lambda: move_z("down"))
z_down_button.grid(row=1, column=0, padx=10, pady=10)

root.mainloop()
