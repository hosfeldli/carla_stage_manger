#!/usr/bin/env python3
import glob
import os
import sys
import random
import math
import logging
import weakref
import json

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import pygame
try:
    from logger import WorldLogger
except ImportError:
    WorldLogger = None
import numpy as np

logging.basicConfig(level=logging.INFO)

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    if len(name) > truncate:
        return name[:truncate - 1] + u'\u2026'
    return name

def load_ego_position(filename='ego_position.json'):
    if not os.path.isfile(filename):
        logging.warning(f"{filename} not found. Will use default spawn transform.")
        return None
    with open(filename, 'r') as f:
        pos = json.load(f)
    required = ['x', 'y', 'z', 'yaw', 'pitch', 'roll']
    if not all(k in pos for k in required):
        logging.warning(f"Invalid format in {filename}. Ignoring it.")
        return None
    logging.info(f"Loaded ego spawn position: {pos}")
    return pos

class CollisionSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))
        self.history = []

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        print(f'Collision with {actor_type}')
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append(intensity)
        # keep recent collision events only
        if len(self.history) > 100:
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
        self.invasion_events = 0

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        print(f'Lane Invasion: Crossed line {" and ".join(text)}')
        self.invasion_events += 1
        if self.invasion_events > 100:
            self.invasion_events = 100  # cap

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
        self.width = width
        self.height = height
        self._camera_transforms = [
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            carla.Transform(carla.Location(x=1.6, z=1.7))
        ]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', carla.ColorConverter.Raw, 'Camera RGB'],
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(self.width))
                bp.set_attribute('image_size_y', str(self.height))
            item.append(bp)
        self.index = None
        self.set_sensor(0, notify=False)

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

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(self.sensors[self.index][1])
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

class World(object):
    def __init__(self, carla_world, actor_filter='vehicle.*', ego_position=None):
        self.world = carla_world
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self.actor_filter = actor_filter
        self._weather_presets = [getattr(carla.WeatherParameters, wp) for wp in [
            'ClearNoon', 'CloudyNoon', 'WetNoon', 'MidRainyNoon', 'HardRainNoon', 'ClearSunset']]
        self._weather_index = 0
        self.ego_position = ego_position

    def _get_spawn_transform(self):
        if not self.ego_position:
            return None
        return carla.Transform(
            carla.Location(
                x=self.ego_position['x'],
                y=self.ego_position['y'],
                z=self.ego_position['z']
            ),
            carla.Rotation(
                pitch=self.ego_position.get('pitch', 0.0),
                yaw=self.ego_position.get('yaw', 0.0),
                roll=self.ego_position.get('roll', 0.0)
            )
        )

    def restart(self):
        if self.player is not None:
            self.destroy()
        spawn_transform = self._get_spawn_transform()
        if spawn_transform is None:
            logging.error("No ego spawn position loaded. Exiting.")
            sys.exit(1)

        ego_vehicles = [actor for actor in self.world.get_actors().filter(self.actor_filter)
                        if actor.attributes.get('role_name') == 'hero']
        if ego_vehicles:
            self.player = ego_vehicles[0]
            logging.info(f"Found existing ego vehicle: {get_actor_display_name(self.player)} at {self.player.get_transform().location}")
            self.player.set_simulate_physics(False)
            self.player.set_transform(spawn_transform)
            self.world.tick()
            self.player.set_simulate_physics(True)
            logging.info(f"Vehicle teleported to {self.player.get_transform()}")
        else:
            blueprint = random.choice(self.world.get_blueprint_library().filter(self.actor_filter))
            blueprint.set_attribute('role_name', 'hero')
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            self.player = self.world.try_spawn_actor(blueprint, spawn_transform)
            if self.player is None:
                logging.error("Failed to spawn ego vehicle at loaded position. Exiting.")
                sys.exit(1)
            logging.info(f"Spawned {get_actor_display_name(self.player)} at {self.player.get_transform().location}")

        self.collision_sensor = CollisionSensor(self.player)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player)
        self.gnss_sensor = GnssSensor(self.player)
        self.world.set_weather(self._weather_presets[self._weather_index])

    def destroy(self):
        sensors = [self.collision_sensor, self.lane_invasion_sensor, self.gnss_sensor]
        for sensor in sensors:
            if sensor and sensor.sensor:
                sensor.sensor.stop()
                sensor.sensor.destroy()
        if self.camera_manager and self.camera_manager.sensor:
            self.camera_manager.sensor.stop()
            self.camera_manager.sensor.destroy()
        if self.player:
            self.player.destroy()

def draw_overlay(display, world):
    font = pygame.font.SysFont('Consolas', 18)
    player = world.player
    if player is None or not player.is_alive:
        return

    vel = player.get_velocity()
    speed_kph = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

    control = player.get_control()
    throttle = control.throttle
    steer = control.steer
    brake = control.brake

    # Draw background panel
    panel_width, panel_height = 320, 140
    s = pygame.Surface((panel_width, panel_height), pygame.SRCALPHA)
    s.fill((0, 0, 0, 140))  # semi-transparent black
    display.blit(s, (10, 10))

    lines = [
        f"Speed: {speed_kph:.1f} km/h",
        f"Throttle: {throttle:.2f}",
        f"Steer: {steer:.2f}",
        f"Brake: {brake:.2f}",
    ]

    # Collision warning
    collision_hist = getattr(world.collision_sensor, 'history', [])
    collision_alert = "YES" if len(collision_hist) > 0 else "No"
    lines.append(f"Collision: {collision_alert}")

    # Lane invasion warning
    lane_inv_count = getattr(world.lane_invasion_sensor, 'invasion_events', 0)
    lane_alert = "YES" if lane_inv_count > 0 else "No"
    lines.append(f"Lane Invasion: {lane_alert}")

    gnss = world.gnss_sensor
    if gnss:
        lines.append(f"GPS: {gnss.lat:.5f}, {gnss.lon:.5f}")

    # Draw text lines
    y = 20
    for line in lines:
        text_surf = font.render(line, True, (255, 255, 255))
        display.blit(text_surf, (20, y))
        y += 22

    # Draw simple compass circle
    yaw = player.get_transform().rotation.yaw % 360
    center = (panel_width - 60, 30)
    radius = 20
    pygame.draw.circle(display, (255, 255, 255), center, radius, 1)
    angle_rad = -math.radians(yaw - 90)
    pointer_length = radius - 5
    end_pos = (
        center[0] + pointer_length * math.cos(angle_rad),
        center[1] + pointer_length * math.sin(angle_rad)
    )
    pygame.draw.line(display, (255, 0, 0), center, end_pos, 2)

def main():
    pygame.init()
    pygame.joystick.quit()
    pygame.joystick.init()
    width, height = 1280, 720
    display = pygame.display.set_mode((width, height), pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption("CARLA Autodrive (Camera + Overlay)")
    clock = pygame.time.Clock()

    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    carla_world = client.get_world()

    ego_position = load_ego_position()
    world = World(carla_world, ego_position=ego_position)
    world.restart()
    world.camera_manager = CameraManager(world.player, width, height)

    world.player.set_autopilot(True)
    tm = client.get_trafficmanager()
    tm.ignore_lights_percentage(world.player, 0)
    tm.auto_lane_change(world.player, True)
    tm.vehicle_percentage_speed_difference(world.player, 0)

    world_logger = None
    if WorldLogger:
        try:
            world_logger = WorldLogger(world)
            carla_world.on_tick(world_logger.on_world_tick)
            logging.info("WorldLogger initialized.")
        except Exception as e:
            logging.warning(f"WorldLogger init failed: {e}")

    running = True
    last_flush = 0
    flush_interval = 5.0

    while running:
        clock.tick_busy_loop(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

        world.camera_manager.render(display)
        draw_overlay(display, world)
        pygame.display.flip()

        if world_logger:
            now = pygame.time.get_ticks() / 1000.0
            if now - last_flush > flush_interval:
                world_logger.flush()
                last_flush = now

        if not world.player.is_alive:
            logging.warning("Ego vehicle destroyed or no longer alive. Exiting.")
            running = False

    if world_logger:
        try:
            carla_world.remove_on_tick(world_logger.on_world_tick)
        except Exception:
            pass
        world_logger.flush()
        world_logger.close()

    world.destroy()
    pygame.quit()
    logging.info("Autodrive script shut down cleanly.")

if __name__ == '__main__':
    main()