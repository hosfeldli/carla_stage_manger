import carla
import random
import time
import sumolib
import traci
import subprocess
import math
import logging

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

class AggressiveTrafficManager:
    def __init__(self, client, world, sumo_cfg_file, carla_tm_port=8000, sumo_port=8813):
        self.client = client
        self.world = world
        self.map = world.get_map()
        self.traffic_manager = client.get_trafficmanager(port=carla_tm_port)
        self.vehicles = {}
        self.sumo_cfg_file = sumo_cfg_file
        self.sumo_port = sumo_port
        self.sumo_process = None
        self.sumo_running = False
        self.obstacle_check_distance = 15  # Meters to check ahead for obstacles
        self.path_planning_lookahead = 30
        self.lane_change_interval = 5  # seconds minimum between lane changes
        self.speed_limit_excess = 10  # max % speed difference allowed
        self._start_sumo()

    def _start_sumo(self):
        try:
            sumo_binary = 'sumo'
            cmd = [
                sumo_binary,
                "-c", self.sumo_cfg_file,
                "--remote-port", str(self.sumo_port),
                "--start",
                "--step-length", "0.1"
            ]
            self.sumo_process = subprocess.Popen(cmd)
            time.sleep(3)
            traci.init(self.sumo_port)
            self.sumo_running = True
            logging.info(f"[SUMO] Started SUMO with TraCI on port {self.sumo_port}")
        except Exception as e:
            logging.error(f"[SUMO] Failed to start SUMO: {e}")
            self.sumo_running = False

    def stop(self):
        if self.sumo_running:
            try:
                traci.close()
                self.sumo_process.terminate()
                self.sumo_process.wait()
                logging.info("[SUMO] SUMO stopped.")
            except Exception as e:
                logging.error(f"[SUMO] Error stopping SUMO: {e}")
            finally:
                self.sumo_running = False

    def add_vehicle(self, vehicle, sumo_id=None):
        self.vehicles[vehicle.id] = {
            "actor": vehicle,
            "sumo_id": sumo_id if sumo_id else str(vehicle.id),
            "last_lane_change": 0
        }
        try:
            # Disable CARLA autopilot so we control speeds
            vehicle.set_autopilot(False)
            # Enable lane changes by traffic manager, we will force lane changes locally on obstacle
            self.traffic_manager.auto_lane_change(vehicle, True)
            self.traffic_manager.force_lane_change(vehicle, False)
            # Reset speed difference
            self.traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
        except Exception as e:
            logging.warning(f"[TM] Failed to configure TM on vehicle {vehicle.id}: {e}")

        if self.sumo_running:
            try:
                if self.vehicles[vehicle.id]["sumo_id"] not in traci.vehicle.getIDList():
                    loc = vehicle.get_location()
                    closest_edge = self._get_sumo_edge(loc)
                    if closest_edge:
                        traci.vehicle.add(
                            self.vehicles[vehicle.id]["sumo_id"],
                            routeID=closest_edge,
                            depart="now",
                            departPos="random"
                        )
                        logging.info(f"[SUMO] Vehicle {self.vehicles[vehicle.id]['sumo_id']} added on edge {closest_edge}")
                    else:
                        logging.warning(f"[SUMO] Could not find SUMO edge for vehicle {self.vehicles[vehicle.id]['sumo_id']}")
            except Exception as e:
                logging.error(f"[SUMO] Could not add vehicle {self.vehicles[vehicle.id]['sumo_id']}: {e}")

    def _get_sumo_edge(self, location):
        try:
            net = sumolib.net.readNet(self.sumo_cfg_file.replace(".sumocfg", ".net.xml"))
            x, y = location.x, location.y
            nearest_edge = net.getNearestEdge((x, y))
            if nearest_edge:
                edge = nearest_edge[0]
                edge_id = edge.getID()
                if edge_id:
                    return edge_id
            logging.warning(f"[SUMOLib] No nearest edge found at location ({x}, {y})")
            return None
        except Exception as e:
            logging.error(f"[SUMOLib] Error getting SUMO edge: {e}")
            return None

    def _detect_obstacle(self, vehicle):
        # Cast several rays forward to detect vehicles or static props blocking path
        transform = vehicle.get_transform()
        loc = transform.location
        forward = transform.get_forward_vector()
        max_distance = self.obstacle_check_distance
        ray_angles = [-10, -5, 0, 5, 10]  # degrees to left/right
        
        for angle_deg in ray_angles:
            angle_rad = math.radians(angle_deg)
            rotated_forward = carla.Vector3D(
                forward.x * math.cos(angle_rad) - forward.y * math.sin(angle_rad),
                forward.x * math.sin(angle_rad) + forward.y * math.cos(angle_rad),
                forward.z
            )
            start = loc + carla.Location(z=1.5)
            end = start + rotated_forward * max_distance
            raycast_results = self.world.cast_ray(start, end)
            for hit in raycast_results:
                if hasattr(hit, 'type_id') and hasattr(hit, 'distance'):
                    # Detect vehicles and static props/buildings as obstacles
                    if ('vehicle' in hit.type_id.lower() or 'static' in hit.type_id.lower()) and hit.distance < max_distance:
                        logging.debug(f"[Obstacle Detection] Vehicle {vehicle.id} obstacle detected at distance {hit.distance:.2f}m, angle {angle_deg} degrees")
                        return True
        return False

    def _try_safe_lane_change(self, vehicle_info):
        vehicle = vehicle_info["actor"]
        now = time.time()
        # Prevent too frequent lane changes
        if now - vehicle_info["last_lane_change"] < self.lane_change_interval:
            return False
        transform = vehicle.get_transform()
        waypoint = self.map.get_waypoint(transform.location, project_to_road=True)
        candidate_wps = []
        try:
            left_lane = waypoint.get_left_lane()
            right_lane = waypoint.get_right_lane()
            # Only consider driving lanes
            if left_lane and left_lane.lane_type == carla.LaneType.Driving:
                candidate_wps.append(left_lane)
            if right_lane and right_lane.lane_type == carla.LaneType.Driving:
                candidate_wps.append(right_lane)
        except Exception as e:
            logging.error(f"[Lane Change] Error getting adjacent lanes for vehicle {vehicle.id}: {e}")
        if not candidate_wps:
            logging.debug(f"[Lane Change] Vehicle {vehicle.id} no lane change options")
            return False
        safe_candidates = []
        for candidate in candidate_wps:
            loc = candidate.transform.location
            forward_vector = candidate.transform.get_forward_vector()
            check_start = loc + carla.Location(z=1.5)
            check_end = check_start + forward_vector * self.obstacle_check_distance
            ray_results = self.world.cast_ray(check_start, check_end)
            if not any(('vehicle' in r.type_id.lower() and r.distance < self.obstacle_check_distance) for r in ray_results):
                safe_candidates.append(candidate)
        if not safe_candidates:
            logging.debug(f"[Lane Change] Vehicle {vehicle.id} no safe lane change candidates")
            return False
        target_wp = random.choice(safe_candidates)
        # Plan waypoints ahead for lane change path
        path_wp = [target_wp]
        length = 3
        current_wp = target_wp
        while length < self.path_planning_lookahead:
            next_wps = current_wp.next(3.0)
            if next_wps:
                current_wp = next_wps[0]
                path_wp.append(current_wp)
                length += 3
            else:
                break
        # Set path for lane change with Traffic Manager
        locations = [wp.transform.location for wp in path_wp]
        self.traffic_manager.set_path(vehicle, locations)
        # Force lane change
        self.traffic_manager.force_lane_change(vehicle, True)
        vehicle_info["last_lane_change"] = now
        logging.info(f"[Lane Change] Vehicle {vehicle.id} performed safe lane change to lane {target_wp.lane_id}")
        return True

    def _slow_down(self, vehicle_info, amount=0.7):
        vehicle = vehicle_info["actor"]
        current_speed = math.sqrt(vehicle.get_velocity().x**2 + vehicle.get_velocity().y**2 + vehicle.get_velocity().z**2)
        if current_speed <= 0.1:
            return  # already stopped or nearly stopped
        target_speed = current_speed * amount
        speed_diff_perc = int((target_speed - current_speed) * 100 / current_speed)
        self.traffic_manager.vehicle_percentage_speed_difference(vehicle, speed_diff_perc)
        self.traffic_manager.force_lane_change(vehicle, False)
        logging.info(f"[Slow Down] Vehicle {vehicle.id} slowing down from {current_speed:.2f} to {target_speed:.2f} m/s")

    def update(self):
        if not self.sumo_running:
            return
        for vid, vehicle_info in list(self.vehicles.items()):
            vehicle = vehicle_info["actor"]
            sumo_id = vehicle_info["sumo_id"]
            if not vehicle.is_alive:
                logging.debug(f"[Update] Vehicle {vid} no longer alive, removing")
                del self.vehicles[vid]
                continue
            if sumo_id not in traci.vehicle.getIDList():
                logging.debug(f"[SUMO] Vehicle {sumo_id} not found in SUMO vehicle list")
                continue
            try:
                obstacle_ahead = self._detect_obstacle(vehicle)
                if obstacle_ahead:
                    # Try lane change first
                    changed = self._try_safe_lane_change(vehicle_info)
                    if not changed:
                        # If no lane change possible, slow down to avoid collision
                        self._slow_down(vehicle_info)
                else:
                    # No obstacle: follow SUMO speed normally
                    speed = traci.vehicle.getSpeed(sumo_id)
                    vel = vehicle.get_velocity()
                    current_speed = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
                    if current_speed > 0:
                        speed_diff_perc = int((speed - current_speed) * 100 / current_speed)
                    else:
                        speed_diff_perc = 0
                    # Clip for smoothness
                    speed_diff_perc = max(min(speed_diff_perc, self.speed_limit_excess), -50)
                    self.traffic_manager.vehicle_percentage_speed_difference(vehicle, speed_diff_perc)
                    # Clear forced lane change so SUMO lane decisions can apply
                    self.traffic_manager.force_lane_change(vehicle, False)
            except Exception as e:
                logging.error(f"[SUMO] Error updating vehicle {vehicle.id}: {e}")

    def __del__(self):
        self.stop()