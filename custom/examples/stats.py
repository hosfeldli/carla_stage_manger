import sys
import json
import math
import tkinter as tk
from tkinter import messagebox

def load_log(filename):
    with open(filename, 'r') as f:
        return json.load(f)

def vector_magnitude(vec):
    return math.sqrt(vec['x']**2 + vec['y']**2 + vec['z']**2)

def compute_overall_stats(log):
    total_frames = len(log)
    if total_frames == 0:
        messagebox.showinfo("Info", "Log file is empty.")
        return None

    total_ego_speed = 0.0
    max_other_speed = 0.0
    total_cloudiness = 0.0

    for entry in log:
        ego_velocity = entry['ego']['velocity']
        ego_speed = vector_magnitude(ego_velocity)
        total_ego_speed += ego_speed

        for vehicle in entry.get('other_vehicles', []):
            speed = vector_magnitude(vehicle['velocity'])
            if speed > max_other_speed:
                max_other_speed = speed

        total_cloudiness += entry['weather']['cloudiness']

    avg_ego_speed = total_ego_speed / total_frames
    avg_cloudiness = total_cloudiness / total_frames

    return {
        "Total frames recorded": total_frames,
        "Average ego vehicle speed (m/s)": avg_ego_speed,
        "Maximum other vehicle speed (m/s)": max_other_speed,
        "Average weather cloudiness (%)": avg_cloudiness
    }

def main(log_file):
    # Load log and compute
    try:
        log = load_log(log_file)
    except Exception as e:
        messagebox.showerror("Error", f"Failed to load log: {e}")
        return

    stats = compute_overall_stats(log)
    if stats is None:
        return

    # Build GUI window
    root = tk.Tk()
    root.title("CARLA Log General Statistics")

    for i, (key, value) in enumerate(stats.items()):
        tk.Label(root, text=f"{key}:").grid(row=i, column=0, sticky="w", padx=10, pady=5)
        if isinstance(value, float):
            val_str = f"{value:.2f}"
        else:
            val_str = str(value)
        tk.Label(root, text=val_str).grid(row=i, column=1, sticky="w", padx=10, pady=5)

    btn_close = tk.Button(root, text="Close", command=root.destroy)
    btn_close.grid(row=len(stats), column=0, columnspan=2, pady=10)

    root.mainloop()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python general_stats_gui.py path_to_log.json")
        sys.exit(1)
    log_file = sys.argv[1]
    main(log_file)