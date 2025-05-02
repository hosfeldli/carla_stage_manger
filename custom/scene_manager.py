import sys
import os
import random
import logging
import math
import time
import subprocess
import json
from PyQt5 import QtWidgets, QtCore, QtGui
import carla

# Constants
CARLA_HOST = '127.0.0.1'
CARLA_PORT = 2000
WINDOW_WIDTH = 2500
WINDOW_HEIGHT = 1000
FPS = 120
DEFAULT_SCALE = 5.0  # pixels per meter for map visualization
SPAWN_POINT_SELECT_DIST = 15  # pixels
VEHICLE_LENGTH = 4.5  # meters approx
VEHICLE_WIDTH = 2.0  # meters approx


def draw_oriented_triangle(painter, center: QtCore.QPoint, yaw_deg: float, size: int, color):
    painter.setBrush(QtGui.QBrush(color))
    angle_rad = math.radians(-yaw_deg)
    p1 = QtCore.QPoint(
        int(center.x() + size * math.cos(angle_rad)),
        int(center.y() + size * math.sin(angle_rad))
    )
    p2 = QtCore.QPoint(
        int(center.x() + size * math.cos(angle_rad + 2.5)),
        int(center.y() + size * math.sin(angle_rad + 2.5))
    )
    p3 = QtCore.QPoint(
        int(center.x() + size * math.cos(angle_rad - 2.5)),
        int(center.y() + size * math.sin(angle_rad - 2.5))
    )
    triangle = QtGui.QPolygon([p1, p2, p3])
    painter.drawPolygon(triangle)


class CarlaSceneManager(QtWidgets.QMainWindow):
    MOVE_STEP = 1.0
    ROTATE_STEP = 5.0

    def __init__(self):
        super().__init__()
        self.setWindowTitle('CARLA Scene Manager')
        self.setGeometry(100, 100, WINDOW_WIDTH, WINDOW_HEIGHT)

        # Initialize CARLA client and world
        self.client = carla.Client(CARLA_HOST, CARLA_PORT)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.blueprint_library = self.world.get_blueprint_library()

        # Actor management
        self.spawned_actors = []
        self.ego_actor = None
        self.selected_vehicle = None

        # Clear the world of vehicles, pedestrians, statics
        self._clean_world()

        # Spawn points
        self.spawn_points = self.map.get_spawn_points()
        self.selected_spawn_point = self.spawn_points[0] if self.spawn_points else carla.Transform()
        self.placement_transform = self.selected_spawn_point
        self.view_center = self.placement_transform.location

        # Mouse dragging state
        self.dragging = False
        self.last_mouse_pos = None

        # Static environment data
        self.static_buildings = []
        self.static_props = []
        self.sidewalks = []
        self._get_static_buildings_and_props()
        self._get_sidewalks()

        # Map scale
        self.scale = DEFAULT_SCALE

        # Vehicle trails and events
        self.vehicle_trails = {}
        self.active_events = []

        # AI management
        self.ai_all_enabled = False
        self.ai_queue = []

        # Camera follow setup
        self.follow_camera = None
        self.following_actor = None
        self.show_camera_in_ui = True

        # Initialize UI
        self.init_ui()

        # Update timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000 / FPS))

    def _clean_world(self):
        all_actors = self.world.get_actors()
        to_destroy = list(all_actors.filter('vehicle.*')) + \
                     list(all_actors.filter('walker.pedestrian.*')) + \
                     list(all_actors.filter('static.*'))
        for actor in to_destroy:
            try:
                actor.destroy()
            except Exception as e:
                logging.warning(f"Failed to destroy actor {actor.id}: {e}")
        logging.info(f"Destroyed {len(to_destroy)} actors to clear the scene.")
        self.spawned_actors.clear()
        self.ego_actor = None
        self.selected_vehicle = None
        if getattr(self, 'follow_camera', None) and self.follow_camera and self.follow_camera.is_alive:
            try:
                self.follow_camera.destroy()
            except Exception:
                pass
        self.follow_camera = None
        self.following_actor = None

    def _get_static_buildings_and_props(self):
        self.static_buildings.clear()
        self.static_props.clear()
        all_statics = self.world.get_actors().filter('static.*')
        for actor in all_statics:
            bp_id = actor.type_id if hasattr(actor, 'type_id') else ''
            role_name = actor.attributes.get('role_name', '') if hasattr(actor, 'attributes') else ''
            if 'building' in bp_id or 'building' in role_name:
                self.static_buildings.append(actor)
            else:
                self.static_props.append(actor)

    def _get_sidewalks(self):
        waypoints = self.map.generate_waypoints(2.0)
        sidewalks = []
        for wp in waypoints:
            if wp.lane_type == carla.LaneType.Sidewalk:
                sidewalks.append(wp)
        self.sidewalks = sidewalks

    def init_ui(self):
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout(central_widget)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        # Left - Map view display
        self.map_view = QtWidgets.QLabel()
        self.map_view.setFixedSize(int(WINDOW_WIDTH * 0.75), WINDOW_HEIGHT)
        self.map_view.setStyleSheet("background-color: #212121; border-radius: 5px;")
        self.map_view.setMouseTracking(True)
        self.map_view.mousePressEvent = self.map_mouse_press
        self.map_view.wheelEvent = self.map_wheel_event
        main_layout.addWidget(self.map_view)

        # Right - Controls and lists panel
        control_panel = QtWidgets.QWidget()
        control_layout = QtWidgets.QVBoxLayout(control_panel)
        control_layout.setSpacing(12)
        control_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.addWidget(control_panel)

        # Map selection combo box
        self.map_combo = QtWidgets.QComboBox()
        self.map_names = []
        try:
            self.map_names = self.client.get_available_maps()
        except Exception:
            # Defaults or fallback list
            self.map_names = ["Town01", "Town02", "Town03", "Town04", "Town05", "Town06", "Town07", "Town08"]
        self.map_combo.addItems(self.map_names)
        self.map_combo.setCurrentText(self.world.get_map().name)
        self.map_combo.currentTextChanged.connect(self.change_map)
        control_layout.addWidget(self._create_label("Select Map:"))
        control_layout.addWidget(self.map_combo)

        # Blueprint combo box
        self.bp_combo = QtWidgets.QComboBox()
        self.load_blueprints()
        control_layout.addWidget(self._create_label("Select Blueprint:"))
        control_layout.addWidget(self.bp_combo)

        # Sliders for View Center and Rotation Yaw
        self.sliders = {}
        slider_names = ['View Center X', 'View Center Y', 'Rotation Yaw']
        slider_ranges = {
            'View Center X': (-500, 500),
            'View Center Y': (-500, 500),
            'Rotation Yaw': (0, 360),
        }
        for name in slider_names:
            control_layout.addWidget(self._create_label(name))
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setMinimum(slider_ranges[name][0] * 10)
            slider.setMaximum(slider_ranges[name][1] * 10)
            slider.setSingleStep(1)
            slider.valueChanged.connect(self.slider_changed)
            control_layout.addWidget(slider)
            self.sliders[name] = slider
        self.update_viewcenter_sliders()
        self.update_sliders_from_transform()

        # Buttons for spawning and ego vehicle
        self.btn_spawn = QtWidgets.QPushButton("Spawn Selected Actor")
        self.btn_spawn.clicked.connect(self.spawn_selected_actor)
        control_layout.addWidget(self.btn_spawn)

        self.btn_spawn_ego = QtWidgets.QPushButton("Spawn Ego Vehicle")
        self.btn_spawn_ego.clicked.connect(self.spawn_ego)
        control_layout.addWidget(self.btn_spawn_ego)

        #self.btn_start_control = QtWidgets.QPushButton("Start Manual Control")
        #self.btn_start_control.clicked.connect(self.start_manual_control)
        #control_layout.addWidget(self.btn_start_control)

        #
        # --- Script Runner UI ---
        #
        control_layout.addWidget(self._create_label("Select Driving Script:"))

        # ComboBox - dynamically filled from scripts/*.py excluding those with 'log'
        self.script_combo = QtWidgets.QComboBox()
        self.available_scripts = []  # List of tuples (display_name, script_full_path)
        self.load_scripts()
        control_layout.addWidget(self.script_combo)

        # Run script button
        self.btn_run_script = QtWidgets.QPushButton("Run Script")
        self.btn_run_script.clicked.connect(self.run_selected_script)
        control_layout.addWidget(self.btn_run_script)

        # Vehicle list
        control_layout.addWidget(self._create_label("Vehicles in World:"))
        self.vehicle_list_widget = QtWidgets.QListWidget()
        self.vehicle_list_widget.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.vehicle_list_widget.itemSelectionChanged.connect(self.vehicle_selection_changed)
        control_layout.addWidget(self.vehicle_list_widget, stretch=1)
        self.vehicle_list_widget.itemDoubleClicked.connect(self.follow_selected_vehicle)

        # AI buttons and checkboxes
        self.btn_enable_ai = QtWidgets.QPushButton("Enable AI on Selected Vehicle")
        self.btn_enable_ai.clicked.connect(self.enable_ai_on_selected)
        self.btn_enable_ai.setEnabled(False)
        control_layout.addWidget(self.btn_enable_ai)
        self.chk_enable_ai_all = QtWidgets.QCheckBox("Enable AI on All Vehicles (Except Ego)")
        self.chk_enable_ai_all.stateChanged.connect(self.ai_all_checkbox_changed)
        control_layout.addWidget(self.chk_enable_ai_all)

        # Spawn count input
        spawn_hbox = QtWidgets.QHBoxLayout()
        spawn_hbox.setSpacing(6)
        self.input_vehicle_count = QtWidgets.QSpinBox()
        self.input_vehicle_count.setMinimum(1)
        self.input_vehicle_count.setMaximum(200)
        self.input_vehicle_count.setValue(1)
        spawn_hbox.addWidget(QtWidgets.QLabel("Number of random vehicles:"))
        spawn_hbox.addWidget(self.input_vehicle_count)
        control_layout.addLayout(spawn_hbox)

        # Spawn random button
        self.btn_spawn_random = QtWidgets.QPushButton("Spawn Random Vehicles")
        self.btn_spawn_random.clicked.connect(self.spawn_random_vehicles)
        control_layout.addWidget(self.btn_spawn_random)
        control_layout.addStretch()

        # SUMO control button
        self.btn_run_sumo = QtWidgets.QPushButton("Start SUMO Traffic Control")
        self.btn_run_sumo.clicked.connect(self.run_sumo_synchronization)
        control_layout.addWidget(self.btn_run_sumo)

        # Apply dark theme stylesheet
        self.apply_dark_theme()

    def load_scripts(self):
        """
        Load all python scripts in the 'scripts' directory (relative to this script),
        excluding any that contain 'log' in their filename (case insensitive).
        Fill the self.available_scripts list and populate self.script_combo.
        """
        self.script_combo.clear()
        self.available_scripts.clear()

        # Determine the scripts directory relative to this file
        scripts_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scripts")
        if not os.path.exists(scripts_dir):
            logging.warning(f"Scripts directory does not exist: {scripts_dir}")
            return

        # List all .py files excluding those with 'log' in name
        files = []
        for f in os.listdir(scripts_dir):
            f_lower = f.lower()
            if f.endswith(".py") and "log" not in f_lower:
                files.append(f)

        files.sort()  # Sort alphabetically

        for script_file in files:
            full_path = os.path.join(scripts_dir, script_file)
            display_name = script_file  # Can strip .py if you want
            self.available_scripts.append((display_name, full_path))
            self.script_combo.addItem(display_name)

        if not self.available_scripts:
            self.script_combo.addItem("(No scripts found)")

    def run_selected_script(self):
        """
        Run the selected script from the console in a separate process.
        Requires ego vehicle to be spawned.
        """
        index = self.script_combo.currentIndex()
        if index < 0 or index >= len(self.available_scripts):
            QtWidgets.QMessageBox.warning(self, "No Script Selected", "Please select a valid script to run.")
            return

        display_name, script_path = self.available_scripts[index]
        if not os.path.isfile(script_path):
            QtWidgets.QMessageBox.warning(self, "Script Missing", f"Script file not found:\n{script_path}")
            return

        if not self.ego_actor or not self.ego_actor.is_alive:
            QtWidgets.QMessageBox.warning(self, "Ego Vehicle Missing",
                                          "You must spawn the ego vehicle before running a driving script.")
            return

        try:
            spawn_args = [sys.executable, script_path]
            subprocess.Popen(spawn_args)
            QtWidgets.QMessageBox.information(self, "Script Launched",
                                              f"Script '{display_name}' launched successfully in a separate process.")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Run Script Error",
                                           f"Failed to launch script '{display_name}':\n{str(e)}")

    def _create_label(self, text):
        label = QtWidgets.QLabel(text)
        label.setStyleSheet("color: #EEEEEE; font-weight: 600;")
        label.setMargin(3)
        return label

    def apply_dark_theme(self):
        # Dark theme stylesheet for UI controls
        dark_style = """
         /* Main Window */
         QMainWindow {
             background-color: #121212;
             color: #EEEEEE;
         }
         /* QWidget and containers */
         QWidget {
             background-color: #121212;
             color: #EEEEEE;
             font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
             font-size: 11pt;
         }
         /* Labels */
         QLabel {
             color: #EEEEEE;
             font-weight: 600;
         }
         /* QPushButton */
         QPushButton {
             background-color: #252525;
             border: 1px solid #444444;
             border-radius: 6px;
             padding: 6px 12px;
             color: #EEEEEE;
             font-weight: 600;
         }
         QPushButton:hover {
             background-color: #3A3A3A;
             border-color: #AAAAAA;
         }
         QPushButton:pressed {
             background-color: #555555;
         }
         QPushButton:disabled {
             background-color: #2A2A2A;
             color: #777777;
             border-color: #444444;
         }
         /* QComboBox */
         QComboBox {
             background-color: #1E1E1E;
             border: 1px solid #555555;
             border-radius: 6px;
             padding: 4px 8px;
             selection-background-color: #3A3A3A;
             selection-color: #FFFFFF;
             min-height: 24px;
         }
         QComboBox:hover {
             border-color: #AAAAAA;
         }
         QComboBox:focus {
             border-color: #FF9800;
         }
         /* QSlider */
         QSlider::groove:horizontal {
             border: 1px solid #444444;
             height: 8px;
             background: #2A2A2A;
             border-radius: 4px;
         }
         QSlider::handle:horizontal {
             background: #0098FF;
             border: 1px solid #0098FF;
             width: 18px;
             margin: -6px 0;
             border-radius: 9px;
         }
         QSlider::handle:horizontal:hover {
             background: #1198FF;
             border: 1px solid #1198FF;
         }
         /* QSpinBox */
         QSpinBox {
             background-color: #1E1E1E;
             border: 1px solid #555555;
             border-radius: 6px;
             padding: 4px 8px;
             min-height: 24px;
             color: #EEEEEE;
         }
         QSpinBox:hover {
             border-color: #AAAAAA;
         }
         QSpinBox:focus {
             border-color: #FF9800;
         }
         /* QListWidget */
         QListWidget {
             background-color: #1E1E1E;
             border: 1px solid #555555;
             border-radius: 6px;
             color: #EEEEEE;
         }
         QListWidget::item:selected {
             background-color: #FF9800;
             color: #121212;
         }
         QListWidget::item:hover {
             background-color: #704400;
         }
         /* QCheckBox */
         QCheckBox {
             spacing: 6px;
             color: #EEEEEE;
         }
         QCheckBox::indicator {
             width: 16px;
             height: 16px;
         }
         QCheckBox::indicator:unchecked {
             border: 1px solid #777777;
             background: #2A2A2A;
             border-radius: 3px;
         }
         QCheckBox::indicator:checked {
             border: 1px solid #FF9800;
             background: #FF9800;
             border-radius: 3px;
         }
         """
        self.setStyleSheet(dark_style)

    def load_blueprints(self):
        self.bp_combo.clear()
        bps = list(self.blueprint_library.filter('vehicle.*')) + list(self.blueprint_library.filter('static.*'))
        self.bp_map = {}
        for bp in bps:
            self.bp_map[bp.id] = bp
            self.bp_combo.addItem(bp.id)

    def change_map(self, map_name):
        try:
            logging.info(f"Loading map {map_name}")
            self._clean_world()
            self.client.load_world(map_name)
            self.world = self.client.get_world()
            self.map = self.world.get_map()
            self.spawn_points = self.map.get_spawn_points()
            self.selected_spawn_point = self.spawn_points[0] if self.spawn_points else carla.Transform()
            self.placement_transform = self.selected_spawn_point
            self.view_center = self.placement_transform.location
            self._get_static_buildings_and_props()
            self._get_sidewalks()
            self.blueprint_library = self.world.get_blueprint_library()
            self.load_blueprints()
            self.update_sliders_from_transform()
            self.update_viewcenter_sliders()
            self.update_scene()
            if self.follow_camera and self.follow_camera.is_alive:
                try:
                    self.follow_camera.destroy()
                except Exception:
                    pass
            self.follow_camera = None
            self.following_actor = None
            logging.info(f"Map {map_name} loaded successfully")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Map Load Error", f"Failed to load map {map_name}: {e}")

    def update_sliders_from_transform(self):
        self.sliders['Rotation Yaw'].blockSignals(True)
        self.sliders['Rotation Yaw'].setValue(int(self.placement_transform.rotation.yaw))
        self.sliders['Rotation Yaw'].blockSignals(False)

    def update_viewcenter_sliders(self):
        self.sliders['View Center X'].blockSignals(True)
        self.sliders['View Center Y'].blockSignals(True)
        self.sliders['View Center X'].setValue(int(self.view_center.x * 10))
        self.sliders['View Center Y'].setValue(int(self.view_center.y * 10))
        self.sliders['View Center X'].blockSignals(False)
        self.sliders['View Center Y'].blockSignals(False)

    def slider_changed(self):
        self.view_center.x = self.sliders['View Center X'].value() / 10.0
        self.view_center.y = self.sliders['View Center Y'].value() / 10.0
        self.placement_transform.rotation.yaw = self.sliders['Rotation Yaw'].value()
        self.update_scene()

    def filtered_spawn_points(self):
        """Return spawn points that are not occupied by existing vehicles."""
        occupied_locations = set()
        for v in self.get_all_vehicles():
            loc = v.get_location()
            occupied_locations.add((loc.x, loc.y, loc.z))
        filtered = []
        for sp in self.spawn_points:
            sp_loc = sp.location
            occupied = False
            for ox, oy, oz in occupied_locations:
                dx = ox - sp_loc.x
                dy = oy - sp_loc.y
                dz = oz - sp_loc.z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if dist < 2.0:
                    occupied = True
                    break
            if not occupied:
                filtered.append(sp)
        return filtered

    def carla_transform_to_qpoint(self, transform):
        dx = transform.location.x - self.view_center.x
        dy = transform.location.y - self.view_center.y
        x = self.map_view.width() // 2 + int(dx * self.scale)
        y = self.map_view.height() // 2 - int(dy * self.scale)
        return QtCore.QPoint(x, y)

    def map_mouse_press(self, event):
        click_pos = event.pos()
        for sp in self.filtered_spawn_points():
            p = self.carla_transform_to_qpoint(sp)
            dist = (p - click_pos).manhattanLength()
            if dist <= SPAWN_POINT_SELECT_DIST:
                self.selected_spawn_point = sp
                self.placement_transform = sp
                self.update_sliders_from_transform()
                logging.info(f"Selected spawn point at {sp.location}")
                break
        if event.button() == QtCore.Qt.LeftButton:
            self.dragging = True
            self.last_mouse_pos = event.pos()

    def map_wheel_event(self, event):
        delta = event.angleDelta().y()
        factor = 1.1 if delta > 0 else 0.9
        new_scale = self.scale * factor
        if 1.0 <= new_scale <= 30.0:
            self.scale = new_scale
            self.update_scene()

    def mouseMoveEvent(self, event):
        if getattr(self, 'dragging', False):
            delta = event.pos() - self.last_mouse_pos
            dx = -delta.x() / self.scale
            dy = delta.y() / self.scale
            self.view_center.x += dx
            self.view_center.y += dy
            self.last_mouse_pos = event.pos()
            self.update_scene()
            self.update_viewcenter_sliders()

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.dragging = False

    def get_all_vehicles(self):
        vehicles = []
        all_vehicles = self.world.get_actors().filter('vehicle.*')
        for v in all_vehicles:
            if v.is_alive:
                vehicles.append(v)
        # Optionally ensure ego vehicle is included
        if self.ego_actor and self.ego_actor.is_alive and self.ego_actor not in vehicles:
            vehicles.append(self.ego_actor)
        return vehicles

    def update_vehicle_list(self):
        self.vehicle_list_widget.blockSignals(True)
        self.vehicle_list_widget.clear()
        vehicles = self.get_all_vehicles()
        self.vehicle_actors_map = {}
        for v in vehicles:
            trans = v.get_transform()
            loc = trans.location
            velocity = v.get_velocity()
            speed_kph = 3.6 * math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) if velocity else 0.0
            label = f"ID:{v.id} {v.type_id} Speed:{speed_kph:.1f} kph Loc:({loc.x:.1f}, {loc.y:.1f})"
            self.vehicle_list_widget.addItem(label)
            self.vehicle_actors_map[label] = v
        self.vehicle_list_widget.blockSignals(False)
        if self.selected_vehicle and self.selected_vehicle.is_alive:
            items = self.vehicle_list_widget.findItems(f"ID:{self.selected_vehicle.id}", QtCore.Qt.MatchStartsWith)
            if items:
                self.vehicle_list_widget.setCurrentItem(items[0])
            else:
                self.selected_vehicle = None
                self.btn_enable_ai.setEnabled(False)
        else:
            self.selected_vehicle = None
            self.btn_enable_ai.setEnabled(False)

    def vehicle_selection_changed(self):
        selected_items = self.vehicle_list_widget.selectedItems()
        if selected_items:
            label = selected_items[0].text()
            actor = self.vehicle_actors_map.get(label, None)
            if actor and actor.is_alive:
                self.selected_vehicle = actor
                self.btn_enable_ai.setEnabled(True)
            else:
                self.selected_vehicle = None
                self.btn_enable_ai.setEnabled(False)
        else:
            self.selected_vehicle = None
            self.btn_enable_ai.setEnabled(False)

    def run_sumo_synchronization(self):
        import subprocess
        import os
        import sys

        working_dir = os.path.expanduser("/home/user/development/carla-package/Co-Simulation/Sumo")
        python_exe = sys.executable
        cmd = [
            python_exe,
            "spawn_npc_sumo.py",
            "-n", f"{self.input_vehicle_count.value()}",
            "--tls-manager", "carla"
        ]
        print(f"Running SUMO synchronization with command: {cmd}")
        try:
            subprocess.Popen(cmd, cwd=working_dir)
            QtWidgets.QMessageBox.information(self, "SUMO Synchronization", "SUMO synchronization launched successfully.")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to run SUMO synchronization:\n{e}")

    def follow_selected_vehicle(self, item):
        """
         Double-click on vehicle - follow with camera sensor, center view, and show in UI.
         """
        actor = self.vehicle_actors_map.get(item.text(), None)
        if not actor or not actor.is_alive:
            QtWidgets.QMessageBox.warning(self, "Error", "Vehicle is no longer alive!")
            return
        # Clean up old camera
        if self.follow_camera and self.follow_camera.is_alive:
            try:
                self.follow_camera.destroy()
            except Exception:
                pass
        self.follow_camera = None
        self.following_actor = actor
        # Spawn a camera sensor and attach to actor
        bp_cam = self.blueprint_library.find('sensor.camera.rgb')
        bp_cam.set_attribute('image_size_x', '800')
        bp_cam.set_attribute('image_size_y', '600')
        bp_cam.set_attribute('fov', '90')
        cam_trans = carla.Transform(
            carla.Location(x=-6, z=2.5),  # 6m behind, 2.5m above
            carla.Rotation(pitch=-10)  # Slight downward pitch
        )
        self.follow_camera = self.world.spawn_actor(bp_cam, cam_trans, attach_to=actor)
        logging.info(f"Spawned camera sensor to follow {actor.id}")
        # Center view on actor
        self.view_center = actor.get_transform().location
        self.update_viewcenter_sliders()

    def enable_ai_on_selected(self):
        if not self.selected_vehicle or not self.selected_vehicle.is_alive:
            QtWidgets.QMessageBox.warning(self, "Error", "Selected vehicle is not alive or valid!")
            return
        self._enable_vehicle_ai(self.selected_vehicle)

    def ai_all_checkbox_changed(self, state):
        self.ai_all_enabled = state == QtCore.Qt.Checked
        vehicles = self.get_all_vehicles()
        for v in vehicles:
            if self.ego_actor and v.id == self.ego_actor.id:
                continue
            if not v.is_alive:
                continue
            if self.ai_all_enabled:
                self._enable_vehicle_ai(v)
            else:
                try:
                    v.set_autopilot(False)
                except Exception:
                    pass

    def _enable_vehicle_ai(self, vehicle):
        try:
            vehicle.set_autopilot(True)
            traffic_manager = self.client.get_trafficmanager()
            traffic_manager.ignore_lights_percentage(vehicle, 0)
            traffic_manager.auto_lane_change(vehicle, True)
            traffic_manager.vehicle_percentage_speed_difference(vehicle, -30)
            traffic_manager.distance_to_leading_vehicle(vehicle, 2.0)
            traffic_manager.force_lane_change(vehicle, True)
            traffic_manager.global_percentage_speed_difference(0)
            traffic_manager.distance_to_leading_vehicle(vehicle, 0.5)
            logging.info(f"Enabled AI on vehicle ID {vehicle.id}")
        except Exception as e:
            logging.error(f"Failed to enable AI on vehicle ID {vehicle.id}: {e}")

    def spawn_random_vehicles(self):
        count = self.input_vehicle_count.value()
        vehicle_bps = list(self.blueprint_library.filter('vehicle.*'))
        free_spawn_points = self.filtered_spawn_points()
        if len(free_spawn_points) < count:
            QtWidgets.QMessageBox.warning(
                self, "Error",
                f"Not enough free spawn points! Requested: {count}, available: {len(free_spawn_points)}"
            )
            return
        chosen_points = random.sample(free_spawn_points, count)
        spawned = 0
        for i in range(count):
            bp = random.choice(vehicle_bps)
            transform = chosen_points[i]
            actor = self.world.try_spawn_actor(bp, transform)
            if actor:
                self.spawned_actors.append(actor)
                spawned += 1
                logging.info(f"Spawned random vehicle {bp.id} at {transform.location}")
        if spawned == 0:
            QtWidgets.QMessageBox.warning(self, "Error", "Failed to spawn any random vehicles!")

    def spawn_selected_actor(self):
        bp_id = self.bp_combo.currentText()
        bp = self.bp_map[bp_id]
        if bp.id.startswith('static.'):
            location = self.placement_transform.location
            waypoint = self.map.get_waypoint(location, project_to_road=True)
            if waypoint:
                location.z = waypoint.transform.location.z
                self.placement_transform.location = location
            transform = self.placement_transform
        elif bp.id.startswith('vehicle.'):
            transform = self.placement_transform
        else:
            transform = self.placement_transform
        actor = self.world.try_spawn_actor(bp, transform)
        if actor:
            self.spawned_actors.append(actor)
            logging.info(f"Spawned actor {bp.id} at {transform.location}")
            # If it's a static prop, add to static_props for rendering
            if bp.id.startswith('static.'):
                self.static_props.append(actor)
            # Optionally update scene to refresh the map view
            self.update_scene()
        else:
            logging.warning(f"Failed to spawn actor {bp.id}")

    def spawn_ego(self):
        if self.ego_actor and self.ego_actor.is_alive:
            logging.info("Destroying existing ego actor")
            try:
                self.ego_actor.destroy()
            except Exception as e:
                logging.warning(f"Failed destroying previous ego vehicle: {e}")
            self.ego_actor = None

        bp_id = self.bp_combo.currentText()
        bp = self.bp_map[bp_id]
        if bp.has_attribute('role_name'):
            bp.set_attribute('role_name', 'hero')
        else:
            logging.warning(f"Blueprint {bp.id} does not have 'role_name' attribute.")
        transform = self.placement_transform
        print(f"Spawning ego vehicle {bp.id} at {transform.location}")
        ego = self.world.try_spawn_actor(bp, transform)
        if ego:
            self.ego_actor = ego
            logging.info(f"Ego vehicle spawned: {ego.type_id} at {ego.get_transform().location}")
            self.placement_transform = ego.get_transform()
            self.update_sliders_from_transform()
            self.view_center = self.placement_transform.location
            self.update_viewcenter_sliders()
            # Save ego position to file
            ego_position = {
                "x": transform.location.x,
                "y": transform.location.y,
                "z": transform.location.z,
                "yaw": transform.rotation.yaw,
                "pitch": transform.rotation.pitch,
                "roll": transform.rotation.roll
            }
            with open("ego_position.json", "w") as file:
                json.dump(ego_position, file)
        else:
            logging.warning("Failed to spawn ego vehicle")

    def start_manual_control(self):
        if not self.ego_actor or not self.ego_actor.is_alive:
            QtWidgets.QMessageBox.warning(self, "Error", "You must spawn the ego vehicle first!")
            return
        if self.ai_all_enabled:
            logging.info("AI already enabled on all vehicles via checkbox.")
        elif self.ai_queue:
            for vehicle in self.ai_queue:
                if vehicle.is_alive:
                    self._enable_vehicle_ai(vehicle)
            logging.info(f"Enabled AI on {len(self.ai_queue)} vehicles before manual control.")
            self.ai_queue.clear()
        spawn_args = [sys.executable, 'controller.py']  # Modify cwd if needed
        try:
            QtWidgets.QMessageBox.information(
                self,
                "Manual Control Started",
                "Manual control will start in a separate process.\nYou may continue editing the scene."
            )
        except Exception as e:
            logging.error(f"Error showing information box: {e}")
        QtCore.QTimer.singleShot(100, lambda: subprocess.Popen(spawn_args))

    def closeEvent(self, event):
        if self.follow_camera and self.follow_camera.is_alive:
            try:
                self.follow_camera.destroy()
            except Exception:
                pass
        self.follow_camera = None
        self.following_actor = None
        for actor in self.spawned_actors:
            if actor.is_alive:
                try:
                    actor.destroy()
                except Exception:
                    pass
        if self.ego_actor and self.ego_actor.is_alive:
            try:
                self.ego_actor.destroy()
            except Exception:
                pass
        event.accept()

    def update_scene(self):
        pixmap = QtGui.QPixmap(self.map_view.width(), self.map_view.height())
        pixmap.fill(QtCore.Qt.white)
        painter = QtGui.QPainter(pixmap)
        # Draw sidewalks
        pen_sidewalk = QtGui.QPen(QtGui.QColor(170, 170, 170))
        brush_sidewalk = QtGui.QBrush(QtGui.QColor(200, 200, 200))
        painter.setPen(pen_sidewalk)
        painter.setBrush(brush_sidewalk)
        for wp in self.sidewalks:
            p = self.carla_transform_to_qpoint(wp.transform)
            painter.drawRect(p.x() - int(self.scale * 0.5), p.y() - int(self.scale * 0.5),
                             int(self.scale), int(self.scale))

        # Draw buildings
        pen_building = QtGui.QPen(QtGui.QColor(100, 100, 100))
        brush_building = QtGui.QBrush(QtGui.QColor(120, 120, 120))
        painter.setPen(pen_building)
        painter.setBrush(brush_building)
        for building in self.static_buildings:
            p = self.carla_transform_to_qpoint(building.get_transform())
            painter.drawRect(p.x() - max(3, int(self.scale / 2)), p.y() - max(3, int(self.scale / 2)),
                             max(6, int(self.scale)), max(6, int(self.scale)))

        # Draw static props
        pen_prop = QtGui.QPen(QtGui.QColor(180, 180, 180))
        brush_prop = QtGui.QBrush(QtGui.QColor(180, 180, 180))
        painter.setPen(pen_prop)
        painter.setBrush(brush_prop)
        for prop in self.static_props:
            p = self.carla_transform_to_qpoint(prop.get_transform())
            painter.drawEllipse(p, max(2, int(self.scale / 3)), max(2, int(self.scale / 3)))

        # Draw spawn points
        pen_spawn = QtGui.QPen(QtGui.QColor(0, 150, 0), 2)
        painter.setPen(pen_spawn)
        painter.setBrush(QtGui.QBrush(QtGui.QColor(0, 150, 0, 150)))
        for sp in self.filtered_spawn_points():
            p = self.carla_transform_to_qpoint(sp)
            painter.drawEllipse(p, max(4, int(self.scale)), max(4, int(self.scale)))

        # Draw selected spawn point highlight
        if self.selected_spawn_point and self.selected_spawn_point in self.filtered_spawn_points():
            pen_cyan = QtGui.QPen(QtGui.QColor(0, 170, 170), 2)
            brush_cyan = QtGui.QBrush(QtGui.QColor(0, 170, 170, 150))
            painter.setPen(pen_cyan)
            painter.setBrush(brush_cyan)
            sp_p = self.carla_transform_to_qpoint(self.selected_spawn_point)
            painter.drawEllipse(sp_p, max(6, int(self.scale * 1.5)), max(6, int(self.scale * 1.5)))

        # Draw placement transform marker
        if self.placement_transform:
            p_selected = self.carla_transform_to_qpoint(self.placement_transform)
            painter.setPen(QtGui.QPen(QtCore.Qt.red, 2))
            painter.setBrush(QtGui.QBrush(QtCore.Qt.red))
            radius = max(3, int(self.scale / 2))
            painter.drawEllipse(p_selected, radius, radius)
            draw_oriented_triangle(painter, p_selected, self.placement_transform.rotation.yaw, radius * 3,
                                   QtGui.QColor(255, 0, 0))

        # Draw waypoints and paths
        waypoints = self.map.generate_waypoints(5.0)
        pen_wp = QtGui.QPen(QtGui.QColor(0, 0, 0), 2)
        painter.setPen(pen_wp)
        painter.setBrush(QtCore.Qt.NoBrush)
        waypoint_groups = {}
        for wp in waypoints:
            key = (wp.road_id, wp.lane_id)
            waypoint_groups.setdefault(key, []).append(wp)
        pen_route = QtGui.QPen(QtGui.QColor(0, 0, 200), 1)
        painter.setPen(pen_route)
        for key, wps in waypoint_groups.items():
            wps_sorted = sorted(wps, key=lambda w: w.s)
            for i in range(len(wps_sorted) - 1):
                p1 = self.carla_transform_to_qpoint(wps_sorted[i].transform)
                p2 = self.carla_transform_to_qpoint(wps_sorted[i + 1].transform)
                painter.drawLine(p1, p2)

        # Draw vehicles
        vehicles = self.get_all_vehicles()
        for actor in vehicles:
            if not actor.is_alive:
                continue
            tr = actor.get_transform()
            p = self.carla_transform_to_qpoint(tr)
            velocity = actor.get_velocity()
            speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
            speed_kph = 3.6 * speed
            if self.ego_actor and actor.id == self.ego_actor.id:
                color = QtGui.QColor(0, 128, 255)
                shadow_color = QtGui.QColor(0, 70, 150, 100)
            else:
                color = QtGui.QColor(255, 64, 64)
                shadow_color = QtGui.QColor(150, 20, 20, 100)
            self.draw_vehicle_rect(painter, p, tr.rotation.yaw, self.scale, color, shadow_color)
            self.draw_velocity_arrow(painter, p, tr.rotation.yaw, speed, self.scale)

        # Draw traffic lights and stop lines/indicators
        traffic_lights = self.world.get_actors().filter('traffic.traffic_light')
        for tl in traffic_lights:
            if not tl.is_alive:
                continue
            state = tl.get_state()
            if state == carla.TrafficLightState.Red:
                color = QtGui.QColor(255, 0, 0, 200)
            elif state == carla.TrafficLightState.Yellow:
                color = QtGui.QColor(255, 255, 0, 200)
            elif state == carla.TrafficLightState.Green:
                color = QtGui.QColor(0, 255, 0, 200)
            else:
                color = QtGui.QColor(150, 150, 150, 100)

            try:
                stop_location = tl.get_stop_location()
            except Exception:
                stop_location = None

            if stop_location is not None:
                p_stop = self.carla_transform_to_qpoint(carla.Transform(stop_location))
                tl_pos = tl.get_transform().location
                p_tl = self.carla_transform_to_qpoint(carla.Transform(tl_pos))
                pen = QtGui.QPen(color, max(2, int(self.scale / 4)))
                painter.setPen(pen)
                painter.drawLine(p_tl, p_stop)
                line_len = max(6, int(self.scale))
                painter.drawLine(p_stop.x() - line_len // 2, p_stop.y(), p_stop.x() + line_len // 2, p_stop.y())
            else:
                tl_transform = tl.get_transform()
                p = self.carla_transform_to_qpoint(tl_transform)
                yaw_rad = math.radians(tl_transform.rotation.yaw)
                offset_m = 3.0
                offset_x = offset_m * math.cos(yaw_rad)
                offset_y = offset_m * math.sin(yaw_rad)
                front_loc = tl_transform.location + carla.Location(x=offset_x, y=offset_y)
                p_front = self.carla_transform_to_qpoint(carla.Transform(front_loc))
                pen = QtGui.QPen(color, max(2, int(self.scale / 4)))
                painter.setPen(pen)
                painter.drawLine(p, p_front)

        # Draw following camera on map (if enabled and exists)
        if self.show_camera_in_ui and self.follow_camera and self.following_actor:
            cam_transform = self.follow_camera.get_transform()
            p_cam = self.carla_transform_to_qpoint(cam_transform)
            size = int(self.scale * 0.8)
            painter.setBrush(QtGui.QColor(130, 0, 200))
            painter.setPen(QtGui.QPen(QtCore.Qt.black, 2))
            draw_oriented_triangle(painter, p_cam, cam_transform.rotation.yaw, size, QtGui.QColor(130, 0, 200))
            painter.setPen(QtCore.Qt.black)
            painter.setFont(QtGui.QFont("Arial", 8))
            painter.drawText(p_cam.x() + size, p_cam.y(), "CAM")

        painter.end()
        self.map_view.setPixmap(pixmap)
        self.update_vehicle_list()

    def draw_vehicle_rect(self, painter, center: QtCore.QPoint, yaw_deg: float, scale, color, shadow_color):
        rect_length = int(VEHICLE_LENGTH * scale)
        rect_width = int(VEHICLE_WIDTH * scale)
        painter.save()
        painter.translate(center)
        painter.rotate(-yaw_deg)
        vehicle_rect = QtCore.QRect(-rect_length // 2, -rect_width // 2, rect_length, rect_width)
        painter.setBrush(color)
        painter.setPen(QtGui.QPen(QtCore.Qt.white, 1))
        painter.drawRoundedRect(vehicle_rect, 3, 3)
        painter.restore()

    def draw_velocity_arrow(self, painter, center: QtCore.QPoint, yaw_deg: float, speed_mps: float, scale):
        if speed_mps <= 0.01:
            return
        arrow_len = min(30, speed_mps * 3 * scale / 5)
        angle_rad = math.radians(-yaw_deg)
        end_x = center.x() + arrow_len * math.cos(angle_rad)
        end_y = center.y() + arrow_len * math.sin(angle_rad)
        painter.setPen(QtGui.QPen(QtGui.QColor(0, 255, 0), 2))
        painter.drawLine(center, QtCore.QPointF(end_x, end_y))
        left_angle = angle_rad + math.radians(150)
        right_angle = angle_rad - math.radians(150)
        head_len = arrow_len * 0.25
        left_x = end_x + head_len * math.cos(left_angle)
        left_y = end_y + head_len * math.sin(left_angle)
        right_x = end_x + head_len * math.cos(right_angle)
        right_y = end_y + head_len * math.sin(right_angle)
        points = [
            QtCore.QPointF(end_x, end_y),
            QtCore.QPointF(left_x, left_y),
            QtCore.QPointF(right_x, right_y)
        ]
        painter.setBrush(QtGui.QColor(0, 255, 0))
        painter.drawPolygon(QtGui.QPolygonF(points))

    def update(self):
        self.update_scene()
        # UI map auto-center on followed actor if following
        if self.following_actor and self.following_actor.is_alive:
            self.view_center = self.following_actor.get_transform().location
            self.update_viewcenter_sliders()
        # If following actor dies, also remove camera
        if self.following_actor and not self.following_actor.is_alive:
            if self.follow_camera and self.follow_camera.is_alive:
                try:
                    self.follow_camera.destroy()
                except Exception:
                    pass
            self.follow_camera = None
            self.following_actor = None


def main():
    logging.basicConfig(level=logging.INFO)
    app = QtWidgets.QApplication(sys.argv)
    window = CarlaSceneManager()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()