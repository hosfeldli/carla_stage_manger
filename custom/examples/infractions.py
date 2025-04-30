import sys
import json
import math
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np

def vector_magnitude(vec):
    return math.sqrt(vec['x']**2 + vec['y']**2 + vec['z']**2)

def detect_infractions(entry):
    infractions = []
    ego = entry['ego']
    vel = ego['velocity']
    ctrl = ego['control']
    weather = entry['weather']

    speed = vector_magnitude(vel)
    SPEED_LIMIT = 15.0  # m/s
    BRAKE_HARD = 0.8
    FOG_DENSITY_LIMIT = 30

    if speed > SPEED_LIMIT:
        infractions.append(f"Speed too high: {speed:.2f} m/s > {SPEED_LIMIT}")
    if ctrl['brake'] > BRAKE_HARD:
        infractions.append(f"Hard braking: brake={ctrl['brake']:.2f} > {BRAKE_HARD}")
    if ctrl['reverse'] is True:
        infractions.append("Reverse gear engaged (reverse=True)")
    if 'gear' in ctrl and ctrl['gear'] < 0:
        infractions.append(f"Gear in reverse: gear={ctrl['gear']}")
    if weather['fog_density'] > FOG_DENSITY_LIMIT:
        infractions.append(f"High fog density: {weather['fog_density']} > {FOG_DENSITY_LIMIT}")

    return infractions

def rotation_to_yaw(rot):
    """In CARLA, rotation.yaw is degrees"""
    return rot['yaw']

class InfractionViewer(tk.Tk):
    def __init__(self, infractions_frames):
        super().__init__()
        self.title("CARLA Log Infractions Viewer")
        self.geometry("1000x700")

        self.infractions_frames = infractions_frames
        self.current_index = 0

        # Navigation frame with buttons and label
        nav_frame = tk.Frame(self)
        nav_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        self.btn_prev = tk.Button(nav_frame, text="<< Previous", command=self.show_prev)
        self.btn_prev.pack(side=tk.LEFT)

        self.lbl_info = tk.Label(nav_frame, text="")
        self.lbl_info.pack(side=tk.LEFT, padx=10)

        self.btn_next = tk.Button(nav_frame, text="Next >>", command=self.show_next)
        self.btn_next.pack(side=tk.LEFT)

        # Split main area horizontally:
        # Left: Text info
        # Right: Matplotlib plot (scene)

        main_frame = tk.Frame(self)
        main_frame.pack(fill=tk.BOTH, expand=True)

        self.text_area = scrolledtext.ScrolledText(main_frame, width=55, font=("Consolas", 11))
        self.text_area.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=5, pady=5)

        self.fig, self.ax = plt.subplots(figsize=(6,6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=main_frame)
        self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.update_view()

    def draw_scene(self, entry):
        self.ax.clear()
        ax = self.ax

        # Draw ego and other vehicles on a top-down 2D plot, x vs y
        # CARLA coords: x = east-west, y = north-south, z = up-down

        # Ego vehicle
        ego_tf = entry['ego']['transform']
        ego_loc = ego_tf['location']
        ego_rot = ego_tf['rotation']
        ego_x, ego_y = ego_loc['x'], ego_loc['y']
        yaw_deg = ego_rot['yaw']

        # Draw ego vehicle as triangle (arrow)
        ego_arrow_length = 4  # meters
        ego_arrow_width = 2

        # Calculate triangle points for arrow (centered at ego_x, ego_y)
        yaw_rad = np.deg2rad(yaw_deg)
        # Tip of arrow
        tip = np.array([ego_x, ego_y]) + ego_arrow_length * np.array([np.cos(yaw_rad), np.sin(yaw_rad)])
        # Base corners
        left_base = np.array([ego_x, ego_y]) + ego_arrow_width * np.array([np.cos(yaw_rad + np.pi*0.75), np.sin(yaw_rad + np.pi*0.75)])
        right_base = np.array([ego_x, ego_y]) + ego_arrow_width * np.array([np.cos(yaw_rad - np.pi*0.75), np.sin(yaw_rad - np.pi*0.75)])

        ego_triangle = np.array([tip, left_base, right_base])

        ax.fill(ego_triangle[:,0], ego_triangle[:,1], color='blue', alpha=0.7, label='Ego Vehicle')
        ax.plot(ego_x, ego_y, 'bo')

        # Other vehicles as red rectangles centered at their location, oriented by yaw
        for v in entry.get('other_vehicles', []):
            loc = v['transform']['location']
            rot = v['transform']['rotation']
            vx, vy = loc['x'], loc['y']
            vyaw = rot['yaw']
            self.draw_vehicle_rect(ax, vx, vy, vyaw, color='red', label=None)

        # Legend fix for multiple red rectangles
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        ax.set_title(f"Top-down View - Frame {entry['frame']} @ {entry['timestamp']:.2f}s")
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.axis('equal')

        # Set view limits centered on ego vehicle, +/- ~30m
        margin = 30
        ax.set_xlim(ego_x - margin, ego_x + margin)
        ax.set_ylim(ego_y - margin, ego_y + margin)

        ax.grid(True)
        self.canvas.draw()

    def draw_vehicle_rect(self, ax, cx, cy, yaw_deg, color='red', label=None):
        """
        Draw a rectangle approx representing a vehicle at center (cx, cy)
        rotated by yaw_deg degrees (CARLA rotation.yaw)
        Vehicle approx size: 4.5m L, 2.0m W
        """
        length = 4.5
        width = 2.0
        yaw_rad = np.deg2rad(yaw_deg)

        # Rectangle corners relative to center before rotation
        corners = np.array([
            [ length/2,  width/2],
            [ length/2, -width/2],
            [-length/2, -width/2],
            [-length/2,  width/2]
        ])

        # Rotation matrix
        rot_mat = np.array([
            [np.cos(yaw_rad), -np.sin(yaw_rad)],
            [np.sin(yaw_rad),  np.cos(yaw_rad)]
        ])

        rotated_corners = (rot_mat @ corners.T).T
        translated_corners = rotated_corners + np.array([cx, cy])

        ax.fill(translated_corners[:,0], translated_corners[:,1], color=color, alpha=0.5, label=label)

        # Optional: plot center point
        ax.plot(cx, cy, marker='o', color=color)

    def update_view(self):
        if not self.infractions_frames:
            self.text_area.delete('1.0', tk.END)
            self.text_area.insert(tk.END, "No infractions found in this log.")
            self.lbl_info.config(text="No infractions")
            self.btn_prev.config(state=tk.DISABLED)
            self.btn_next.config(state=tk.DISABLED)
            return

        frame_data = self.infractions_frames[self.current_index]
        entry = frame_data['entry']
        infractions = frame_data['infractions']

        self.lbl_info.config(text=f"Infraction {self.current_index + 1} / {len(self.infractions_frames)} - Frame {entry['frame']}")

        # Text info
        lines = []
        lines.append(f"Frame: {entry['frame']}")
        lines.append(f"Timestamp: {entry['timestamp']:.3f} s")
        lines.append("")

        ego = entry['ego']
        ctrl = ego['control']
        spd = vector_magnitude(ego['velocity'])
        lines.append("=== Ego Vehicle ===")
        lines.append(f"Speed: {spd:.2f} m/s")
        lines.append(f"Throttle: {ctrl['throttle']:.2f}")
        lines.append(f"Steer: {ctrl['steer']:.2f}")
        lines.append(f"Brake: {ctrl['brake']:.2f}")
        lines.append(f"Hand Brake: {ctrl['hand_brake']}")
        lines.append(f"Reverse: {ctrl['reverse']}")
        lines.append(f"Gear: {ctrl['gear']}")
        lines.append("")

        weather = entry['weather']
        lines.append("=== Weather ===")
        for k, v in weather.items():
            lines.append(f"{k}: {v}")

        lines.append("")
        lines.append(f"Other vehicles nearby: {len(entry.get('other_vehicles', []))}")
        lines.append("")
        lines.append("=== Infractions ===")
        for inf in infractions:
            lines.append(f"* {inf}")

        self.text_area.delete('1.0', tk.END)
        self.text_area.insert(tk.END, '\n'.join(lines))

        self.draw_scene(entry)

        self.btn_prev.config(state=tk.NORMAL if self.current_index > 0 else tk.DISABLED)
        self.btn_next.config(state=tk.NORMAL if self.current_index < len(self.infractions_frames) - 1 else tk.DISABLED)

    def show_prev(self):
        if self.current_index > 0:
            self.current_index -= 1
            self.update_view()

    def show_next(self):
        if self.current_index < len(self.infractions_frames) - 1:
            self.current_index += 1
            self.update_view()

def load_log(filename):
    with open(filename, 'r') as f:
        return json.load(f)

def main(log_file):
    try:
        log = load_log(log_file)
    except Exception as e:
        messagebox.showerror("Error", f"Failed to load log file:\n{e}")
        return

    infractions_frames = []
    for entry in log:
        infs = detect_infractions(entry)
        if infs:
            infractions_frames.append({'entry': entry, 'infractions': infs})

    app = InfractionViewer(infractions_frames)
    app.mainloop()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python infraction_viewer_gui.py path_to_log.json")
        sys.exit(1)

    log_path = sys.argv[1]
    main(log_path)