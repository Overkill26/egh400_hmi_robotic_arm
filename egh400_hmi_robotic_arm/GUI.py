import tkinter as tk

# Function to handle slider changes
def slider_changed(value, axis):
    print(f"Axis {axis}: {value}")

# Function to handle end effector movement
def move_end_effector(direction):
    print(f"End effector moved {direction}")

# Create main window
root = tk.Tk()
root.title("Robotic Arm Control GUI")

# Frame for sliders
sliders_frame = tk.Frame(root)
sliders_frame.pack(side=tk.LEFT, padx=10, pady=10)

# Create 7 sliders for controlling robotic arm axes
sliders = []
for i in range(7):
    label = tk.Label(sliders_frame, text=f"Axis {i+1}")
    label.pack()
    
    slider = tk.Scale(sliders_frame, from_=-180, to=180, orient=tk.HORIZONTAL, command=lambda value, i=i: slider_changed(value, i+1))
    slider.pack()
    sliders.append(slider)

# Frame for End Effector Control
effector_frame = tk.Frame(root)
effector_frame.pack(side=tk.LEFT, padx=20, pady=10)

# Title label for End Effector Control
effector_label = tk.Label(effector_frame, text="End Effector Control", font=("Arial", 14))
effector_label.grid(row=0, column=0, columnspan=2, pady=10)

# Frame for XY control (D-Pad style)
xy_frame = tk.Frame(effector_frame)
xy_frame.grid(row=1, column=0, padx=10)

# Arrow buttons for x and y movement
up_button = tk.Button(xy_frame, text="↑", command=lambda: move_end_effector("Up"))
up_button.grid(row=0, column=1)

left_button = tk.Button(xy_frame, text="←", command=lambda: move_end_effector("Left"))
left_button.grid(row=1, column=0)

right_button = tk.Button(xy_frame, text="→", command=lambda: move_end_effector("Right"))
right_button.grid(row=1, column=2)

down_button = tk.Button(xy_frame, text="↓", command=lambda: move_end_effector("Down"))
down_button.grid(row=2, column=1)

# Frame for Z control (up and down) to the right of the D-Pad
z_frame = tk.Frame(effector_frame)
z_frame.grid(row=1, column=1, padx=20)

# Arrow buttons for z-axis movement
z_up_button = tk.Button(z_frame, text="Z ↑", command=lambda: move_end_effector("Z Up"))
z_up_button.pack(pady=5)

z_down_button = tk.Button(z_frame, text="Z ↓", command=lambda: move_end_effector("Z Down"))
z_down_button.pack(pady=5)

# Run the Tkinter event loop
root.mainloop()
