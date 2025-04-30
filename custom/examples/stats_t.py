import sys
import json
import math
import tkinter as tk
from tkinter import messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

def load_log(filename):
    with open(filename, 'r') as f:
        return json.load(f)

def vector_magnitude(vec):
    return math.sqrt(vec['x']**2 + vec['y']**2 + vec['z']**2)

def plot_speed_and_cloudiness_window(log):
    times = []
    ego_speeds = []
    cloudiness_list = []

    for entry in log:
        times.append(entry['timestamp'])
        ego_speed = vector_magnitude(entry['ego']['velocity'])
        ego_speeds.append(ego_speed)
        cloudiness_list.append(entry['weather']['cloudiness'])

    root = tk.Tk()
    root.title("Ego Speed and Weather Cloudiness Over Time")

    fig, ax1 = plt.subplots(figsize=(8, 5), dpi=100)

    color_speed = 'tab:blue'
    ax1.set_xlabel('Time (seconds)')
    ax1.set_ylabel('Ego Speed (m/s)', color=color_speed)
    ax1.plot(times, ego_speeds, color=color_speed)
    ax1.tick_params(axis='y', labelcolor=color_speed)

    ax2 = ax1.twinx()
    color_cloud = 'tab:gray'
    ax2.set_ylabel('Cloudiness (%)', color=color_cloud)
    ax2.plot(times, cloudiness_list, color=color_cloud, linestyle='--')
    ax2.tick_params(axis='y', labelcolor=color_cloud)

    fig.suptitle('Ego Vehicle Speed and Weather Cloudiness Over Time')

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.draw()
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    btn_close = tk.Button(root, text="Close", command=root.destroy)
    btn_close.pack(pady=5)

    root.mainloop()

def main(log_file):
    try:
        log = load_log(log_file)
    except Exception as e:
        messagebox.showerror("Error", f"Failed to load log: {e}")
        return
    if not log:
        messagebox.showinfo("Info", "Log file is empty.")
        return
    plot_speed_and_cloudiness_window(log)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python ego_speed_weather_gui.py path_to_log.json")
        sys.exit(1)
    log_file = sys.argv[1]
    main(log_file)