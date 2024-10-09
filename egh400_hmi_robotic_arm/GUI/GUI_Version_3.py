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

def reset_positions():
    for slider in sliders:
        slider.set(0)
    print("Positions reset to 0 degrees.")

def on_selection_change():
    selected = radio_var.get()
    if selected == 1:
        print("Use axis sliders selected")
    elif selected == 3:
        print("Use end effector buttons selected")

root = tk.Tk()
root.title("6-Axis Robotic Arm Controller")

title_label = ttk.Label(root, text="6-Axis Robotic Arm Controller", font=("Helvetica", 16))
title_label.pack(pady=10)

radio_var = tk.IntVar()

button_frame = ttk.Frame(root)
button_frame.pack(pady=10)

radio_use_axis = ttk.Radiobutton(button_frame, text="Use Axis Sliders", variable=radio_var, value=1, command=on_selection_change)
radio_use_axis.grid(row=0, column=0, padx=10)

reset_button = ttk.Button(button_frame, text="Reset Positions", command=reset_positions)
reset_button.grid(row=0, column=1, padx=10)

radio_use_effector = ttk.Radiobutton(button_frame, text="Use End Effector Buttons", variable=radio_var, value=3, command=on_selection_change)
radio_use_effector.grid(row=0, column=2, padx=10)

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

separator = ttk.Separator(root, orient='vertical')
separator.pack(side=tk.LEFT, fill='y', padx=20)

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
