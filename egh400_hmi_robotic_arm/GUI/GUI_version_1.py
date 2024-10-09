import tkinter as tk
from tkinter import ttk

def update_slider_value(axis, value):
    print(f"Axis {axis}: {value} degrees")

root = tk.Tk()
root.title("6-Axis Robotic Arm Controller")
root.geometry("600x400")

title_label = ttk.Label(root, text="6-Axis Robotic Arm Controller", font=("Helvetica", 16))
title_label.pack(pady=10)

slider_frame = ttk.Frame(root)
slider_frame.pack(pady=20)

sliders = []
for i in range(1, 7):
    axis_label = ttk.Label(slider_frame, text=f"Axis {i}:")
    axis_label.grid(row=i-1, column=0, padx=10, pady=5)

    slider = ttk.Scale(slider_frame, from_=-180, to=180, orient="horizontal", length=400,
                       command=lambda value, axis=i: update_slider_value(axis, int(float(value))))
    slider.grid(row=i-1, column=1, padx=10, pady=5)
    sliders.append(slider)

root.mainloop()
