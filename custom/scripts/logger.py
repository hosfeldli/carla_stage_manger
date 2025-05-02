import json
import threading
import time
from datetime import datetime

def transform_to_dict(transform):
    loc = transform.location
    rot = transform.rotation
    return {
        'location': {'x': loc.x, 'y': loc.y, 'z': loc.z},
        'rotation': {'pitch': rot.pitch, 'yaw': rot.yaw, 'roll': rot.roll}
    }

def velocity_to_dict(velocity):
    return {'x': velocity.x, 'y': velocity.y, 'z': velocity.z}

def control_to_dict(control):
    return {
        'throttle': control.throttle,
        'steer': control.steer,
        'brake': control.brake,
        'hand_brake': control.hand_brake,
        'reverse': control.reverse,
        'manual_gear_shift': control.manual_gear_shift,
        'gear': control.gear
    }

class WorldLogger:
    """
    Logs CARLA simulation world data incrementally writing a valid JSON array
    to a file with timestamps in filename.
    """

    def __init__(self, world_wrapper, filename=None, flush_interval=5.0):
        """
        :param world_wrapper: object with .world and .player (ego vehicle)
        :param filename: output JSON file (default carla_log_YYYYmmdd_HHMMSS.json)
        :param flush_interval: seconds between async writes to disk (default 5.0)
        """
        self.world_wrapper = world_wrapper
        self.world = world_wrapper.world
        if filename is None:
            timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.filename = f'logs/carla_log_{timestamp_str}.json'
        else:
            self.filename = filename
        self.flush_interval = flush_interval
        self._log = []
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._first_entry = True  # Tracks if we've written first array item for comma placement
        self._file_initialized = False
        self._file = None
        
        # Open file in write mode and write opening bracket of JSON array
        try:
            self._file = open(self.filename, 'w')
            self._file.write('[\n')  # Start JSON array
            self._file.flush()
            self._file_initialized = True
        except Exception as e:
            print(f"[WorldLogger] Failed to initialize log file '{self.filename}': {e}")
            self._file_initialized = False
            self._file = None

        # Start the background flush thread
        self._flush_thread = threading.Thread(target=self._flush_loop, daemon=True)
        self._flush_thread.start()

    def on_world_tick(self, snapshot):
        ego = self.world_wrapper.player
        if ego is None:
            return

        try:
            ego_transform = ego.get_transform()
            ego_velocity = ego.get_velocity()
            ego_control = ego.get_control()
            other_vehicles = [v for v in self.world.get_actors().filter('vehicle.*') if v.id != ego.id]
            others_data = []
            for vehicle in other_vehicles:
                try:
                    others_data.append({
                        'id': vehicle.id,
                        'type': vehicle.type_id,
                        'transform': transform_to_dict(vehicle.get_transform()),
                        'velocity': velocity_to_dict(vehicle.get_velocity())
                    })
                except Exception as e:
                    print(f"[WorldLogger] Error processing vehicle id {vehicle.id}: {e}")

            weather = self.world.get_weather()
            weather_dict = {
                'cloudiness': weather.cloudiness,
                'precipitation': weather.precipitation,
                'precipitation_deposits': weather.precipitation_deposits,
                'wind_intensity': weather.wind_intensity,
                'sun_azimuth_angle': weather.sun_azimuth_angle,
                'sun_altitude_angle': weather.sun_altitude_angle,
                'fog_density': weather.fog_density,
                'fog_distance': weather.fog_distance,
                'fog_falloff': weather.fog_falloff,
                'wetness': weather.wetness,
            }
            tick_data = {
                'frame': snapshot.frame,
                'timestamp': snapshot.timestamp.elapsed_seconds,
                'ego': {
                    'id': ego.id,
                    'type': ego.type_id,
                    'transform': transform_to_dict(ego_transform),
                    'velocity': velocity_to_dict(ego_velocity),
                    'control': control_to_dict(ego_control)
                },
                'other_vehicles': others_data,
                'weather': weather_dict
            }
            with self._lock:
                self._log.append(tick_data)
        except Exception as e:
            print(f"[WorldLogger] Exception during on_world_tick: {e}")

    def _flush_loop(self):
        while not self._stop_event.wait(self.flush_interval):
            self.flush()

    def flush(self):
        with self._lock:
            if not self._file_initialized or not self._log or self._file is None:
                return
            try:
                for item in self._log:
                    if not self._first_entry:
                        self._file.write(',\n')
                    else:
                        self._first_entry = False
                    json.dump(item, self._file, indent=2)
                self._file.flush()
                self._log.clear()
            except Exception as e:
                print(f"[WorldLogger] Error writing log to file: {e}")

    def close(self):
        self._stop_event.set()
        self._flush_thread.join()
        self.flush()
        if self._file_initialized and self._file is not None:
            try:
                # Guarantee we close the JSON array properly
                self._file.write('\n]\n')
                self._file.flush()
                self._file.close()
                print(f"[WorldLogger] Closed log file {self.filename}")
            except Exception as e:
                print(f"[WorldLogger] Failed to close log file properly: {e}")
            finally:
                self._file_initialized = False

    @staticmethod
    def read_log(filename):
        combined_log = []
        try:
            with open(filename, 'r') as f:
                combined_log = json.load(f)
        except Exception as e:
            print(f"[WorldLogger] Failed to read log: {e}")
        return combined_log