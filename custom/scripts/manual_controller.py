#!/usr/bin/env python
# Simplified CARLA manual control with steering wheel G29.
# Removed HUD sidebar/info, keeps camera rendering only.
# Added JSON world state logging every tick (ego + other vehicles + weather)
from __future__ import print_function
import glob
import os
import sys
import random
import collections
import math
import logging
import weakref
import json
import atexit
from configparser import ConfigParser
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
try:
    import pygame
    from pygame.locals import (KMOD_CTRL, KMOD_SHIFT, K_0, K_9, K_BACKQUOTE,
                               K_BACKSPACE, K_COMMA, K_DOWN, K_ESCAPE,
                               K_F1, K_LEFT, K_PERIOD, K_RIGHT, K_SLASH,
                               K_SPACE, K_TAB, K_UP, K_a, K_c, K_d, K_h,
                               K_m, K_p, K_q, K_r, K_s, K_w)
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')
try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')
from logger import WorldLogger  # Make sure logger.py exists and is in path with WorldLogger class

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def transform_to_dict(transform: carla.Transform):
    loc = transform.location
    rot = transform.rotation
    return {
        'location': {'x': loc.x, 'y': loc.y, 'z': loc.z},
        'rotation': {'pitch': rot.pitch, 'yaw': rot.yaw, 'roll': rot.roll}
    }

def velocity_to_dict(velocity: carla.Vector3D):
    return {'x': velocity.x, 'y': velocity.y, 'z': velocity.z}

def control_to_dict(control: carla.VehicleControl):
    return {
        'throttle': control.throttle,
        'steer': control.steer,
        'brake': control.brake,
        'hand_brake': control.hand_brake,
        'reverse': control.reverse,
        'manual_gear_shift': control.manual_gear_shift,
        'gear': control.gear
    }

class CollisionSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        print(f'Collision with {actor_type}')
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)

class LaneInvasionSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        print(f'Lane Invasion: Crossed line {" and ".join(text)}')

class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude

class CameraManager(object):
    def __init__(self, parent_actor, width, height):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.recording = False
        self.width = width
        self.height = height
        self._camera_transforms = [
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            carla.Transform(carla.Location(x=1.6, z=1.7))
        ]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', carla.ColorConverter.Raw, 'Camera RGB'],
            ['sensor.camera.depth', carla.ColorConverter.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', carla.ColorConverter.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', carla.ColorConverter.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', carla.ColorConverter.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', carla.ColorConverter.CityScapesPalette,
             'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(self.width))
                bp.set_attribute('image_size_y', str(self.height))
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '50')
            item.append(bp)
        self.index = None
        # Spawn first sensor by default
        self.set_sensor(0, notify=False)

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        if self.sensor is not None:
            self.sensor.set_transform(self._camera_transforms[self.transform_index])

    def set_sensor(self, index, notify=True):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else self.sensors[index][0] != self.sensors[self.index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.stop()
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index],
                attach_to=self._parent)
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            print(f"Camera sensor set to: {self.sensors[index][2]}")
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        print(f"Recording {'On' if self.recording else 'Off'}")

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.width, self.height) / 100.0
            lidar_data += (0.5 * self.width, 0.5 * self.height)
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.width, self.height, 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)

class DualControl(object):
    def __init__(self, world, start_in_autopilot=True):
        self._autopilot_enabled = start_in_autopilot
        self.world = world
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0

        # Initialize joystick wheel
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Joystick")
        elif joystick_count == 0:
            print("Warning: No joystick detected, keyboard control fallback will not be supported.")
            self._joystick = None
        else:
            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()

        # Read wheel config
        self._parser = ConfigParser()
        if not self._parser.read('wheel_config.ini'):
            raise FileNotFoundError("wheel_config.ini not found or unreadable")

        # Validate indices safely
        def parse_index(section, option):
            try:
                return int(self._parser.get(section, option))
            except Exception as e:
                print(f"Warning: Could not read '{option}' from config: {e}")
                return None

        self._steer_idx = parse_index('G29 Racing Wheel', 'steering_wheel')
        self._throttle_idx = parse_index('G29 Racing Wheel', 'throttle')
        self._brake_idx = parse_index('G29 Racing Wheel', 'brake')
        self._handbrake_idx = parse_index('G29 Racing Wheel', 'handbrake')

        # Hardcoded reverse button index - check if valid later
        self._reverse_idx = 7

    def parse_events(self, clock, world, camera_manager):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True  # signal to quit
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:  # Restart vehicle at current transform
                    world.restart(world.player.get_transform())
                elif event.button == 1:  # Toggle camera sensor
                    camera_manager.next_sensor()
                elif event.button == 3:  # Toggle weather preset
                    world.next_weather()
                elif event.button == self._reverse_idx:
                    self._control.gear = 1 if self._control.reverse else -1
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    world.restart(world.player.get_transform())
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather()
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    camera_manager.next_sensor()
                elif K_0 < event.key <= K_9:
                    camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r:
                    camera_manager.toggle_recording()

        if not self._autopilot_enabled:
            if self._joystick:
                self._parse_vehicle_wheel()
            self._control.reverse = self._control.gear < 0
            world.player.apply_control(self._control)
        return False

    def _parse_vehicle_wheel(self):
        if not self._joystick:
            return

        numAxes = self._joystick.get_numaxes()
        numButtons = self._joystick.get_numbuttons()

        # Get joystick axes safely
        def get_axis_safe(idx):
            if idx is None or idx < 0 or idx >= numAxes:
                return 0.0
            try:
                return float(self._joystick.get_axis(idx))
            except Exception:
                return 0.0

        # Get joystick button safely
        def get_button_safe(idx):
            if idx is None or idx < 0 or idx >= numButtons:
                return 0
            try:
                return self._joystick.get_button(idx)
            except Exception:
                return 0

        jsInputs = [get_axis_safe(i) for i in range(numAxes)]
        jsButtons = [get_button_safe(i) for i in range(numButtons)]

        # Safely calculate steer command
        steer_input = get_axis_safe(self._steer_idx)
        K1 = 1.0
        try:
            steerCmd = K1 * math.tan(1.1 * steer_input)
        except Exception:
            steerCmd = 0.0

        # Safe throttle calculation
        throttle_input = get_axis_safe(self._throttle_idx)
        try:
            val = -0.7 * throttle_input + 1.4
            val = max(val, 1e-5)  # Avoid log domain error
            throttleCmd = 1.6 + (2.05 * math.log10(val) - 1.2) / 0.92
        except Exception:
            throttleCmd = 0.0
        throttleCmd = max(0, min(throttleCmd, 1))

        # Safe brake calculation
        brake_input = get_axis_safe(self._brake_idx)
        try:
            val = -0.7 * brake_input + 1.4
            val = max(val, 1e-5)  # Avoid log domain error
            brakeCmd = 1.6 + (2.05 * math.log10(val) - 1.2) / 0.92
        except Exception:
            brakeCmd = 0.0
        brakeCmd = max(0, min(brakeCmd, 1))

        self._control.steer = steerCmd
        self._control.throttle = throttleCmd
        self._control.brake = brakeCmd
        self._control.hand_brake = bool(get_button_safe(self._handbrake_idx))

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

class World(object):
    def __init__(self, carla_world, actor_filter, ego_position):
        self.world = carla_world
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._actor_filter = actor_filter
        self._weather_presets = [getattr(carla.WeatherParameters, wp) for wp in [
            'ClearNoon', 'CloudyNoon', 'WetNoon', 'MidRainyNoon', 'HardRainNoon', 'ClearSunset']]
        self._weather_index = 0
        self._ego_position = ego_position

    def _get_spawn_transform(self):
        if not self._ego_position:
            return None
        return carla.Transform(
            carla.Location(
                x=self._ego_position['x'],
                y=self._ego_position['y'],
                z=self._ego_position['z']
            ),
            carla.Rotation(
                pitch=self._ego_position.get('pitch', 0.0),
                yaw=self._ego_position.get('yaw', 0.0),
                roll=self._ego_position.get('roll', 0.0)
            )
        )

    def restart(self, spawn_transform=None):
        if self.player is not None:
            self.destroy()
        if spawn_transform is None:
            spawn_transform = self._get_spawn_transform()
        if spawn_transform is None:
            print("No ego spawn position loaded or provided. Exiting program.")
            import sys
            sys.exit(1)
        ego_vehicles = [actor for actor in self.world.get_actors().filter(self._actor_filter)
                        if actor.attributes.get('role_name') == 'hero']
        if ego_vehicles:
            self.player = ego_vehicles[0]
            print(f"Found existing ego vehicle: {get_actor_display_name(self.player)} at {self.player.get_transform().location}")
            self.player.set_simulate_physics(False)
            self.player.set_transform(spawn_transform)
            self.player.set_target_velocity(carla.Vector3D(0, 0, 0))
            self.player.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
            self.world.tick()
            self.player.set_simulate_physics(True)
            print(f"Vehicle teleported to {self.player.get_transform()}")
        else:
            blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
            blueprint.set_attribute('role_name', 'hero')
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            self.player = self.world.try_spawn_actor(blueprint, spawn_transform)
            if self.player is None:
                print("Failed to spawn ego vehicle at the loaded position. Exiting program.")
                import sys
                sys.exit(1)
            self.collision_sensor = CollisionSensor(self.player)
            self.lane_invasion_sensor = LaneInvasionSensor(self.player)
            self.gnss_sensor = GnssSensor(self.player)
            self.world.set_weather(self._weather_presets[self._weather_index])
            self.player.set_transform(spawn_transform)
            print(f"Using {get_actor_display_name(self.player)} at {self.player.get_transform().location}, {self.player.get_transform().rotation}")

    def next_weather(self):
        self._weather_index = (self._weather_index + 1) % len(self._weather_presets)
        self.world.set_weather(self._weather_presets[self._weather_index])
        print(f"Weather changed to preset index {self._weather_index}")

    def destroy(self):
        sensors = []
        for sensor in [self.collision_sensor, self.lane_invasion_sensor, self.gnss_sensor]:
            if sensor is not None and sensor.sensor is not None:
                sensors.append(sensor.sensor)
        if self.camera_manager and self.camera_manager.sensor is not None:
            sensors.append(self.camera_manager.sensor)
        for sensor in sensors:
            sensor.stop()
            sensor.destroy()
        if self.player is not None:
            self.player.destroy()

def load_ego_position(filename='ego_position.json'):
    if not os.path.isfile(filename):
        print(f"Warning: {filename} not found. No ego spawn position loaded.")
        return None
    with open(filename, 'r') as f:
        pos = json.load(f)
    # Validate presence of required keys
    required = ['x', 'y', 'z', 'yaw', 'pitch', 'roll']
    if not all(k in pos for k in required):
        print(f"Warning: Invalid format for ego position in {filename}. Exiting.")
        return None
    print(f"Loaded ego spawn position from {filename}: {pos}")
    return pos

def run_manual_control(world, spawn_transform, width, height, autopilot=True):
    import pygame
    pygame.init()
    pygame.joystick.init()
    display = pygame.display.set_mode((width, height), pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption("CARLA Manual Control (No HUD Sidebar)")
    clock = pygame.time.Clock()

    # For this script, ignore spawn_transform argument and let World.restart() handle spawning
    world.restart()
    world.camera_manager = CameraManager(world.player, width, height)
    try:
        world_logger = WorldLogger(world, filename=None)
    except Exception:
        world_logger = None  # logging won't work if WorldLogger not available
    if world_logger:
        world.world.on_tick(world_logger.on_world_tick)
    controller = DualControl(world, autopilot)
    running = True
    last_flush_time = 0
    flush_interval = 5.0  # seconds
    while running:
        clock.tick_busy_loop(60)
        running = not controller.parse_events(clock, world, world.camera_manager)
        world.camera_manager.render(display)
        pygame.display.flip()
        # Periodically flush (every flush_interval seconds)
        now = pygame.time.get_ticks() / 1000.0  # seconds
        if world_logger and now - last_flush_time > flush_interval:
            world_logger.flush()
            last_flush_time = now
    # Flush and close only once after exiting loop
    if world_logger:
        world_logger.flush()
        world_logger.close()
    pygame.quit()
    world.destroy()

def start_manual_control_at(spawn_transform, host='127.0.0.1', port=2000,
                            actor_filter='vehicle.*', autopilot=False,
                            width=1280, height=720):
    """
    Initializes CARLA client, world, and runs manual control:
    Tries to take over an existing ego vehicle if found.
    Rendering camera feed only without HUD sidebar.
    """
    client = None
    world = None
    try:
        client = carla.Client(host, port)
        client.set_timeout(10.0)
        carla_world = client.get_world()
        # Load ego position from JSON file
        ego_position = load_ego_position('ego_position.json')
        world = World(carla_world, actor_filter, ego_position)
        run_manual_control(world, spawn_transform, width, height, autopilot)
    finally:
        if world is not None:
            world.destroy()
        import pygame
        pygame.quit()

if __name__ == '__main__':
    import argparse
    argparser = argparse.ArgumentParser(description='CARLA manual control - camera only (no HUD sidebar) with JSON logging')
    argparser.add_argument('--host', default='127.0.0.1', help='CARLA host IP')
    argparser.add_argument('-p', '--port', default=2000, type=int, help='CARLA port')
    argparser.add_argument('--spawn-x', default=0.0, type=float, help='Spawn X location')
    argparser.add_argument('--spawn-y', default=0.0, type=float, help='Spawn Y location')
    argparser.add_argument('--spawn-z', default=0.0, type=float, help='Spawn Z location')
    argparser.add_argument('--yaw', default=0.0, type=float, help='Spawn yaw rotation')
    argparser.add_argument('--autopilot', action='store_true', help='Enable autopilot on start')
    argparser.add_argument('--width', default=1280, type=int, help='Display width')
    argparser.add_argument('--height', default=720, type=int, help='Display height')
    argparser.add_argument('--log', default=None, help='File to log world data as JSON')
    args = argparser.parse_args()

    spawn_transform = carla.Transform(
        carla.Location(x=args.spawn_x, y=args.spawn_y, z=args.spawn_z),
        carla.Rotation(yaw=args.yaw)
    )

    logging.basicConfig(level=logging.INFO)
    print("Starting manual control with camera rendering only (no HUD sidebar)")
    start_manual_control_at(spawn_transform, host=args.host, port=args.port,
                            autopilot=args.autopilot, width=args.width, height=args.height)