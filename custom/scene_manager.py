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
from traffic_handler import AggressiveTrafficManager
import sumolib
import traci

# Constants
CARLA_HOST = '127.0.0.1'
CARLA_PORT = 2000
WINDOW_WIDTH = 1400
WINDOW_HEIGHT = 900
FPS = 20
DEFAULT_SCALE = 5.0  # pixels per meter for map visualization
SPAWN_POINT_SELECT_DIST = 15  # pixels
VEHICLE_LENGTH = 4.5  # meters approx
VEHICLE_WIDTH = 2.0  # meters approx


def draw_oriented_triangle(painter, center: QtCore.QPoint, yaw_deg: float, size: int, color):
    painter.setBrush(QtGui.QBrush(color))
    angle_rad = math.radians(-yaw_deg)
    p1 = QtCore.QPoint(int(center.x() + size * math.cos(angle_rad)),
                       int(center.y() + size * math.sin(angle_rad)))
    p2 = QtCore.QPoint(int(center.x() + size * math.cos(angle_rad + 2.5)),
                       int(center.y() + size * math.sin(angle_rad + 2.5)))
    p3 = QtCore.QPoint(int(center.x() + size * math.cos(angle_rad - 2.5)),
                       int(center.y() + size * math.sin(angle_rad - 2.5)))
    triangle = QtGui.QPolygon([p1, p2, p3])
    painter.drawPolygon(triangle)


class CarlaSceneManager(QtWidgets.QMainWindow):
    MOVE_STEP = 1.0
    ROTATE_STEP = 5.0

    def __init__(self):
        super().__init__()
        self.setWindowTitle('CARLA Scene Manager - Detailed View')
        self.setGeometry(100, 100, WINDOW_WIDTH, WINDOW_HEIGHT)
        self.client = carla.Client(CARLA_HOST, CARLA_PORT)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()

        # Only list SUMO-safe maps here:
        self.sumo_cfg_dir = "./sumo_configs"
        self.safe_maps = self._list_sumo_maps()
        self.blueprint_library = self.world.get_blueprint_library()
        self.spawned_actors = []
        self.ego_actor = None
        self.selected_vehicle = None
        self._clean_world()
        self.spawn_points = self.map.get_spawn_points()
        self.selected_spawn_point = self.spawn_points[0] if self.spawn_points else carla.Transform()
        self.placement_transform = self.selected_spawn_point
        self.view_center = self.placement_transform.location
        self.dragging = False
        self.last_mouse_pos = None
        self.static_buildings = []
        self.static_props = []
        self.sidewalks = []
        self._get_static_buildings_and_props()
        self._get_sidewalks()
        self.scale = DEFAULT_SCALE
        self.vehicle_trails = {}
        self.active_events = []
        self.ai_all_enabled = False
        self.init_ui()
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000 / FPS))
        self.hybrid_traffic_manager = None
        # Use only SUMO-safe maps on init:
        self.update_hybrid_control_availability(self.map.name)

    def _list_sumo_maps(self):
        """List maps which have a sumo config available."""
        maps = []
        if os.path.isdir(self.sumo_cfg_dir):
            for file in os.listdir(self.sumo_cfg_dir):
                if file.endswith(".sumocfg"):
                    maps.append(file.replace(".sumocfg", ""))
        return maps

    def init_ui(self):
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout(central_widget)

        # Left - Map view
        self.map_view = QtWidgets.QLabel()
        self.map_view.setFixedSize(WINDOW_WIDTH // 2, WINDOW_HEIGHT)
        self.map_view.setStyleSheet("background-color:white;")
        self.map_view.setMouseTracking(True)
        self.map_view.mousePressEvent = self.map_mouse_press
        self.map_view.wheelEvent = self.map_wheel_event
        main_layout.addWidget(self.map_view)

        # Left - Map selection combo box - only SUMO-safe maps
        self.map_combo = QtWidgets.QComboBox()
        self.map_combo.addItems(sorted(self.safe_maps))
        current_map_name = self.world.get_map().name.split('/')[-1]
        if current_map_name in self.safe_maps:
            self.map_combo.setCurrentText(current_map_name)
        else:
            if self.safe_maps:
                self.map_combo.setCurrentIndex(0)
                self.change_map(self.safe_maps[0])
        self.map_combo.currentTextChanged.connect(self.change_map)
        main_layout.addWidget(self.map_combo)

        # Right - Controls and blueprints, etc.
        control_panel = QtWidgets.QWidget()
        control_layout = QtWidgets.QVBoxLayout(control_panel)
        main_layout.addWidget(control_panel)

        control_layout.addWidget(QtWidgets.QLabel("Select Blueprint:"))
        self.bp_combo = QtWidgets.QComboBox()
        control_layout.addWidget(self.bp_combo)

        # Load safe blueprints by default
        self.load_blueprints(safe=True)

        # Simplified Sliders for transform inputs
        self.sliders = {}
        slider_names = ['Location X', 'Location Y', 'Rotation Yaw']
        slider_ranges = {
            'Location X': (-500, 500),
            'Location Y': (-500, 500),
            'Rotation Yaw': (0, 360),
        }
        for name in slider_names:
            control_layout.addWidget(QtWidgets.QLabel(name))
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setMinimum(slider_ranges[name][0] * 10)
            slider.setMaximum(slider_ranges[name][1] * 10)
            slider.setSingleStep(1)
            slider.valueChanged.connect(self.slider_changed)
            control_layout.addWidget(slider)
            self.sliders[name] = slider
        self.update_sliders_from_transform()

        # Buttons for spawning and ego
        self.btn_spawn = QtWidgets.QPushButton("Spawn Selected Actor")
        self.btn_spawn.clicked.connect(self.spawn_selected_actor)
        control_layout.addWidget(self.btn_spawn)
        self.btn_spawn_ego = QtWidgets.QPushButton("Spawn Ego Vehicle")
        self.btn_spawn_ego.clicked.connect(self.spawn_ego)
        control_layout.addWidget(self.btn_spawn_ego)
        self.btn_start_control = QtWidgets.QPushButton("Start Manual Control")
        self.btn_start_control.clicked.connect(self.start_manual_control)
        control_layout.addWidget(self.btn_start_control)

        # List of vehicles in the world
        control_layout.addWidget(QtWidgets.QLabel("Vehicles in World:"))
        self.vehicle_list_widget = QtWidgets.QListWidget()
        self.vehicle_list_widget.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.vehicle_list_widget.itemSelectionChanged.connect(self.vehicle_selection_changed)
        control_layout.addWidget(self.vehicle_list_widget, stretch=1)

        # Button to enable AI on selected vehicle
        self.btn_enable_ai = QtWidgets.QPushButton("Enable AI on Selected Vehicle")
        self.btn_enable_ai.clicked.connect(self.enable_ai_on_selected)
        self.btn_enable_ai.setEnabled(False)
        control_layout.addWidget(self.btn_enable_ai)

        # Checkbox to mass enable AI (exclude ego)
        self.chk_enable_ai_all = QtWidgets.QCheckBox("Enable AI on All Vehicles (Except Ego)")
        self.chk_enable_ai_all.stateChanged.connect(self.ai_all_checkbox_changed)
        control_layout.addWidget(self.chk_enable_ai_all)

        # Input for number of random vehicles to spawn
        spawn_hbox = QtWidgets.QHBoxLayout()
        self.input_vehicle_count = QtWidgets.QSpinBox()
        self.input_vehicle_count.setMinimum(1)
        self.input_vehicle_count.setMaximum(50)
        self.input_vehicle_count.setValue(1)
        spawn_hbox.addWidget(QtWidgets.QLabel("Number of random vehicles:"))
        spawn_hbox.addWidget(self.input_vehicle_count)
        control_layout.addLayout(spawn_hbox)

        # Button to spawn random vehicles (multiple)
        self.btn_spawn_random = QtWidgets.QPushButton("Spawn Random Vehicles")
        self.btn_spawn_random.clicked.connect(self.spawn_random_vehicles)
        control_layout.addWidget(self.btn_spawn_random)

        # Hybrid Control checkbox
        self.chk_hybrid_control = QtWidgets.QCheckBox("Hybrid Control (SUMO-based)")
        self.chk_hybrid_control.setEnabled(False)
        self.chk_hybrid_control.stateChanged.connect(self.hybrid_control_changed)
        control_layout.addWidget(self.chk_hybrid_control)

        control_layout.addStretch()

    def load_blueprints(self, safe=False):
        self.bp_combo.clear()
        bps = list(self.blueprint_library.filter('vehicle.*'))
        vtypes = self._load_sumo_vtypes()

        if safe:
            filtered_bps = []
            for bp in bps:
                vtype = vtypes.get(bp.id, None)
                if vtype is None:
                    continue
                if vtype['vClass'] in ('motorcycle', 'bicycle'):
                    continue
                if bp.id.endswith(('microlino', 'carlacola', 'cybertruck', 't2', 'sprinter', 'firetruck', 'ambulance')):
                    continue
                filtered_bps.append(bp)
            bps = filtered_bps

        self.bp_map = {}
        for bp in bps:
            self.bp_map[bp.id] = bp
            self.bp_combo.addItem(bp.id)

        static_bps = list(self.blueprint_library.filter('static.*'))
        for bp in static_bps:
            self.bp_map[bp.id] = bp
            self.bp_combo.addItem(bp.id)

    def _load_sumo_vtypes(self):
        # Replace with actual SUMO vehicle types loading
        vtypes = {
            'vehicle.audi.a2': {'vClass': 'passenger'},
            'vehicle.tesla.model3': {'vClass': 'passenger'},
            'vehicle.harley-davidson.low_rider': {'vClass': 'motorcycle'},
            'vehicle.carlacola': {'vClass': 'special'},
            'vehicle.cybertruck': {'vClass': 'special'},
        }
        return vtypes

    def change_map(self, map_name):
        try:
            if map_name not in self.safe_maps:
                QtWidgets.QMessageBox.warning(self, "Map Error", f"Map '{map_name}' is not safe or SUMO supported!")
                return
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
            self.load_blueprints(safe=True)
            self.update_sliders_from_transform()
            self.update_scene()
            logging.info(f"Map {map_name} loaded successfully")
            self.update_hybrid_control_availability(map_name)
            if self.hybrid_traffic_manager:
                self.hybrid_traffic_manager.stop()
                self.hybrid_traffic_manager = None
                self.chk_hybrid_control.blockSignals(True)
                self.chk_hybrid_control.setChecked(False)
                self.chk_hybrid_control.blockSignals(False)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Map Load Error", f"Failed to load map {map_name}: {e}")

    def update_hybrid_control_availability(self, map_name):
        sumo_cfg_path = os.path.join(self.sumo_cfg_dir, f"{map_name}.sumocfg")
        if os.path.isfile(sumo_cfg_path):
            self.chk_hybrid_control.setEnabled(True)
            self.chk_hybrid_control.setToolTip(f"SUMO config found: {sumo_cfg_path}")
        else:
            self.chk_hybrid_control.setEnabled(False)
            self.chk_hybrid_control.setChecked(False)
            self.chk_hybrid_control.setToolTip("No SUMO config found for this map, Hybrid Control disabled.")

    def _clean_world(self):
        all_actors = self.world.get_actors()
        to_destroy = list(all_actors.filter('vehicle.*')) + list(all_actors.filter('walker.pedestrian.*')) + list(all_actors.filter('static.*'))
        for actor in to_destroy:
            try:
                actor.destroy()
            except Exception as e:
                logging.warning(f"Failed to destroy actor {actor.id}: {e}")
        logging.info(f"Destroyed {len(to_destroy)} actors to clear the scene.")
        self.spawned_actors.clear()
        self.ego_actor = None
        self.selected_vehicle = None

    def _get_static_buildings_and_props(self):
        self.static_buildings.clear()
        self.static_props.clear()
        all_statics = self.world.get_actors().filter('static.*')
        for actor in all_statics:
            bp_id = actor.type_id if hasattr(actor, 'type_id') else ''
            if 'building' in bp_id or 'building' in actor.attributes.get('role_name', ''):
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

    def update_sliders_from_transform(self):
        self.sliders['Location X'].blockSignals(True)
        self.sliders['Location Y'].blockSignals(True)
        self.sliders['Rotation Yaw'].blockSignals(True)
        self.sliders['Location X'].setValue(int(self.placement_transform.location.x * 10))
        self.sliders['Location Y'].setValue(int(self.placement_transform.location.y * 10))
        self.sliders['Rotation Yaw'].setValue(int(self.placement_transform.rotation.yaw))
        self.sliders['Location X'].blockSignals(False)
        self.sliders['Location Y'].blockSignals(False)
        self.sliders['Rotation Yaw'].blockSignals(False)

    def update_transform_from_sliders(self):
        self.placement_transform.location.x = self.sliders['Location X'].value() / 10.0
        self.placement_transform.location.y = self.sliders['Location Y'].value() / 10.0
        self.placement_transform.rotation.yaw = self.sliders['Rotation Yaw'].value()

    def slider_changed(self):
        self.update_transform_from_sliders()

    def filtered_spawn_points(self):
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
                self.view_center = sp.location
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

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.dragging = False

    def get_all_vehicles(self):
        vehicles = []
        if self.ego_actor and self.ego_actor.is_alive:
            vehicles.append(self.ego_actor)
        for actor in self.spawned_actors:
            if actor.is_alive and 'vehicle.' in actor.type_id:
                vehicles.append(actor)
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
            QtWidgets.QMessageBox.warning(self, "Error",
                                          f"Not enough free spawn points! Requested: {count}, available: {len(free_spawn_points)}")
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
            logging.info(f"AI already enabled on all vehicles via checkbox.")
        spawn_args = [sys.executable, 'controller.py']
        try:
            QtWidgets.QMessageBox.information(self, "Manual Control Started",
                                              "Manual control will start in a separate process.\nYou may continue editing the scene.")
        except Exception as e:
            logging.error(f"Error showing information box: {e}")
        QtCore.QTimer.singleShot(100, lambda: subprocess.Popen(spawn_args))

    def closeEvent(self, event):
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
        pen_sidewalk = QtGui.QPen(QtGui.QColor(170, 170, 170))
        brush_sidewalk = QtGui.QBrush(QtGui.QColor(200, 200, 200))
        painter.setPen(pen_sidewalk)
        painter.setBrush(brush_sidewalk)
        for wp in self.sidewalks:
            p = self.carla_transform_to_qpoint(wp.transform)
            painter.drawRect(p.x() - int(self.scale * 0.5), p.y() - int(self.scale * 0.5), int(self.scale), int(self.scale))
        pen_building = QtGui.QPen(QtGui.QColor(100, 100, 100))
        brush_building = QtGui.QBrush(QtGui.QColor(120, 120, 120))
        painter.setPen(pen_building)
        painter.setBrush(brush_building)
        for building in self.static_buildings:
            p = self.carla_transform_to_qpoint(building.get_transform())
            painter.drawRect(p.x() - max(3, int(self.scale / 2)), p.y() - max(3, int(self.scale / 2)), max(6, int(self.scale)), max(6, int(self.scale)))
        pen_prop = QtGui.QPen(QtGui.QColor(180, 180, 180))
        brush_prop = QtGui.QBrush(QtGui.QColor(180, 180, 180))
        painter.setPen(pen_prop)
        painter.setBrush(brush_prop)
        for prop in self.static_props:
            p = self.carla_transform_to_qpoint(prop.get_transform())
            painter.drawEllipse(p, max(2, int(self.scale / 3)), max(2, int(self.scale / 3)))
        pen_spawn = QtGui.QPen(QtGui.QColor(0, 150, 0), 2)
        painter.setPen(pen_spawn)
        painter.setBrush(QtGui.QBrush(QtGui.QColor(0, 150, 0, 150)))
        for sp in self.filtered_spawn_points():
            p = self.carla_transform_to_qpoint(sp)
            painter.drawEllipse(p, max(4, int(self.scale)), max(4, int(self.scale)))
        if self.selected_spawn_point and self.selected_spawn_point in self.filtered_spawn_points():
            pen_cyan = QtGui.QPen(QtGui.QColor(0, 170, 170), 2)
            brush_cyan = QtGui.QBrush(QtGui.QColor(0, 170, 170, 150))
            painter.setPen(pen_cyan)
            painter.setBrush(brush_cyan)
            sp_p = self.carla_transform_to_qpoint(self.selected_spawn_point)
            painter.drawEllipse(sp_p, max(6, int(self.scale * 1.5)), max(6, int(self.scale * 1.5)))
        if self.placement_transform:
            p_selected = self.carla_transform_to_qpoint(self.placement_transform)
            painter.setPen(QtGui.QPen(QtCore.Qt.red, 2))
            painter.setBrush(QtGui.QBrush(QtCore.Qt.red))
            radius = max(3, int(self.scale / 2))
            painter.drawEllipse(p_selected, radius, radius)
            draw_oriented_triangle(painter, p_selected, self.placement_transform.rotation.yaw, radius * 3, QtGui.QColor(255, 0, 0))
        waypoints = self.map.generate_waypoints(5.0)
        pen_wp = QtGui.QPen(QtGui.QColor(0, 0, 0), 2)
        painter.setPen(pen_wp)
        painter.setBrush(QtCore.Qt.NoBrush)
        waypoint_groups = {}
        for wp in waypoints:
            key = (wp.road_id, wp.lane_id)
            if key not in waypoint_groups:
                waypoint_groups[key] = []
            waypoint_groups[key].append(wp)
        pen_route = QtGui.QPen(QtGui.QColor(0, 0, 200), 1)
        painter.setPen(pen_route)
        for key, wps in waypoint_groups.items():
            wps_sorted = sorted(wps, key=lambda w: w.s)
            for i in range(len(wps_sorted) - 1):
                p1 = self.carla_transform_to_qpoint(wps_sorted[i].transform)
                p2 = self.carla_transform_to_qpoint(wps_sorted[i + 1].transform)
                painter.drawLine(p1, p2)
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
                color = QtGui.QColor(0, 128, 255)  # Ego blue
                shadow_color = QtGui.QColor(0, 70, 150, 100)
            else:
                color = QtGui.QColor(255, 64, 64)  # Other vehicles red
                shadow_color = QtGui.QColor(150, 20, 20, 100)
            self.draw_vehicle_rect(painter, p, tr.rotation.yaw, self.scale, color, shadow_color)
            self.draw_velocity_arrow(painter, p, tr.rotation.yaw, speed, self.scale)
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
        points = [QtCore.QPointF(end_x, end_y), QtCore.QPointF(left_x, left_y), QtCore.QPointF(right_x, right_y)]
        painter.setBrush(QtGui.QColor(0, 255, 0))
        painter.drawPolygon(QtGui.QPolygonF(points))

    def update(self):
        self.update_scene()
        if self.hybrid_traffic_manager:
            try:
                self.hybrid_traffic_manager.update()
            except Exception as e:
                logging.error(f"Error in HybridTrafficManager update: {e}")

    # Hybrid Control integration
    def hybrid_control_changed(self, state):
        if state == QtCore.Qt.Checked:
            self.start_hybrid_control()
        else:
            self.stop_hybrid_control()

    def start_hybrid_control(self):
        map_name = self.map_combo.currentText()
        sumo_cfg_path = os.path.join("./sumo_configs", f"{map_name}.sumocfg")
        if not os.path.isfile(sumo_cfg_path):
            QtWidgets.QMessageBox.warning(self, "Hybrid Control",
                                          f"No SUMO config found for map {map_name}. Cannot enable Hybrid Control.")
            self.chk_hybrid_control.blockSignals(True)
            self.chk_hybrid_control.setChecked(False)
            self.chk_hybrid_control.blockSignals(False)
            return
        try:
            if self.hybrid_traffic_manager:
                self.hybrid_traffic_manager.stop()
                self.hybrid_traffic_manager = None
            self.hybrid_traffic_manager = AggressiveTrafficManager(self.client, self.world, sumo_cfg_path)
            for v in self.get_all_vehicles():
                if self.ego_actor and v.id == self.ego_actor.id:
                    continue
                self.hybrid_traffic_manager.add_vehicle(v)
            for v in self.hybrid_traffic_manager.vehicles.values():
                try:
                    v["actor"].set_autopilot(False)
                except Exception:
                    pass
            logging.info("Hybrid control ENABLED on all vehicles except ego.")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Hybrid Control", f"Failed to start Hybrid Control: {e}")
            self.chk_hybrid_control.blockSignals(True)
            self.chk_hybrid_control.setChecked(False)
            self.chk_hybrid_control.blockSignals(False)

    def stop_hybrid_control(self):
        if self.hybrid_traffic_manager:
            self.hybrid_traffic_manager.stop()
            self.hybrid_traffic_manager = None
            logging.info("Hybrid control DISABLED.")
        if self.ai_all_enabled:
            for v in self.get_all_vehicles():
                if self.ego_actor and v.id == self.ego_actor.id:
                    continue
                try:
                    v.set_autopilot(True)
                except Exception:
                    pass


def main():
    logging.basicConfig(level=logging.INFO)
    app = QtWidgets.QApplication(sys.argv)
    window = CarlaSceneManager()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()