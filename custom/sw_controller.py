#!/usr/bin/env python
# Simplified CARLA manual control focusing only on steering wheel car controls.
# No HUD, no other key functionality, camera rendering only.

from __future__ import print_function
import glob
import os
import sys
import random
import collections
import math
import weakref
import pygame
from configparser import ConfigParser

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import numpy as np

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

# Sensors simplified for vehicle

class CollisionSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        print(f'Collision with {actor_type}')

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
            ['sensor.camera.rgb', carla.ColorConverter.Raw, 'Camera RGB']
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            bp.set_attribute('image_size_x', str(self.width))
            bp.set_attribute('image_size_y', str(self.height))
            item.append(bp)
        self.index = None
        self.set_sensor(0, notify=False)

    def set_sensor(self, index, notify=True):
        index = index % len(self.sensors)
        needs_respawn = False
        if self.index is None:
            needs_respawn = True
        else:
            if self.sensors[index][0] != self.sensors[self.index][0]:
                needs_respawn = True
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

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(self.sensors[self.index][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))


class DualControl(object):
    def __init__(self, world, start_in_autopilot=True):
        self._autopilot_enabled = start_in_autopilot
        self.world = world
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0

        # Initialize joystick wheel
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Joystick")
        elif joystick_count == 0:
            print("Warning: No joystick detected, joystick control will not be used.")
            self._joystick = None
        else:
            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()

        # Read wheel config
        self._parser = ConfigParser()
        if not self._parser.read('wheel_config.ini'):
            raise FileNotFoundError("wheel_config.ini not found or unreadable")
        self._steer_idx = int(self._parser.get('G29 Racing Wheel', 'steering_wheel'))
        self._throttle_idx = int(self._parser.get('G29 Racing Wheel', 'throttle'))
        self._brake_idx = int(self._parser.get('G29 Racing Wheel', 'brake'))
        self._handbrake_idx = int(self._parser.get('G29 Racing Wheel', 'handbrake'))

    def parse_events(self, clock, world, camera_manager):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True  # signal to quit
        if not self._autopilot_enabled:
            if self._joystick:
                self._parse_vehicle_wheel()
            self._control.reverse = self._control.gear < 0
            world.player.apply_control(self._control)
        return False

    def _parse_vehicle_wheel(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        jsButtons = [float(self._joystick.get_button(i)) for i in range(self._joystick.get_numbuttons())]
        K1 = 1.0
        steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])
        K2 = 1.6
        throttleCmd = K2 + (2.05 * math.log10(-0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        throttleCmd = max(0, min(throttleCmd, 1))
        brakeCmd = 1.6 + (2.05 * math.log10(-0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        brakeCmd = max(0, min(brakeCmd, 1))
        self._control.steer = steerCmd
        self._control.brake = brakeCmd
        self._control.throttle = throttleCmd
        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

class World(object):
    def __init__(self, carla_world, actor_filter):
        self.world = carla_world
        self.player = None
        self.collision_sensor = None
        self.camera_manager = None
        self._actor_filter = actor_filter
        self._weather_presets = [getattr(carla.WeatherParameters, wp) for wp in [
            'ClearNoon', 'CloudyNoon', 'WetNoon', 'MidRainyNoon', 'HardRainNoon', 'ClearSunset']]
        self._weather_index = 0

    def restart(self, spawn_transform):
        if self.player is not None:
            self.destroy()

        ego_vehicles = [actor for actor in self.world.get_actors().filter(self._actor_filter)
                        if actor.attributes.get('role_name') == 'hero']

        if ego_vehicles:
            self.player = ego_vehicles[0]
            print(f"Found existing ego vehicle: {get_actor_display_name(self.player)} at {self.player.get_transform().location}")
            self.player.set_simulate_physics(False)
            self.player.set_transform(spawn_transform)
            self.player.set_target_velocity(carla.Vector3D(0,0,0))
            self.player.set_target_angular_velocity(carla.Vector3D(0,0,0))
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
                spawn_points = self.world.get_map().get_spawn_points()
                spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
                self.player = self.world.try_spawn_actor(blueprint, spawn_point)
                if self.player is None:
                    raise RuntimeError("Failed to spawn ego vehicle")
            print(f"Spawned new ego vehicle {get_actor_display_name(self.player)} at {self.player.get_transform().location}")

        # Sensors
        self.collision_sensor = CollisionSensor(self.player)
        # Setup camera manager
        self.camera_manager = CameraManager(self.player, 1280, 720)
        self.world.set_weather(self._weather_presets[self._weather_index])
        self.player.set_transform(spawn_transform)

    def destroy(self):
        sensors = []
        if self.collision_sensor and self.collision_sensor.sensor:
            sensors.append(self.collision_sensor.sensor)
        if self.camera_manager and self.camera_manager.sensor:
            sensors.append(self.camera_manager.sensor)
        for sensor in sensors:
            sensor.stop()
            sensor.destroy()
        if self.player is not None:
            self.player.destroy()

def run_manual_control(world, spawn_transform):
    pygame.init()
    pygame.joystick.init()
    width, height = 1280, 720
    display = pygame.display.set_mode((width, height), pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption("CARLA Manual Control - Steering Wheel Only")
    clock = pygame.time.Clock()

    world.restart(spawn_transform)

    controller = DualControl(world, start_in_autopilot=False)

    running = True
    while running:
        clock.tick_busy_loop(60)
        running = not controller.parse_events(clock, world, world.camera_manager)
        world.camera_manager.render(display)
        pygame.display.flip()

    pygame.quit()
    world.destroy()

def start_manual_control_at(spawn_transform, host='127.0.0.1', port=2000,
                            actor_filter='vehicle.*'):
    client = None
    world = None
    try:
        client = carla.Client(host, port)
        client.set_timeout(10.0)
        carla_world = client.get_world()
        world = World(carla_world, actor_filter)
        run_manual_control(world, spawn_transform)
    finally:
        if world is not None:
            world.destroy()
        pygame.quit()

if __name__ == '__main__':
    import argparse
    argparser = argparse.ArgumentParser(description='CARLA manual control - steering wheel car controls only')
    argparser.add_argument('--host', default='127.0.0.1', help='CARLA host IP')
    argparser.add_argument('-p', '--port', default=2000, type=int, help='CARLA port')
    argparser.add_argument('--spawn-x', default=0.0, type=float, help='Spawn X location')
    argparser.add_argument('--spawn-y', default=0.0, type=float, help='Spawn Y location')
    argparser.add_argument('--spawn-z', default=0.0, type=float, help='Spawn Z location')
    argparser.add_argument('--yaw', default=0.0, type=float, help='Spawn yaw rotation')
    args = argparser.parse_args()

    spawn_transform = carla.Transform(
        carla.Location(x=args.spawn_x, y=args.spawn_y, z=args.spawn_z),
        carla.Rotation(yaw=args.yaw)
    )

    print("Starting manual control - steering wheel car controls only")
    start_manual_control_at(spawn_transform, host=args.host, port=args.port)