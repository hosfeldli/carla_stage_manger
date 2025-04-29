#!/usr/bin/env python3
import carla
import json
import pygame
import math
import sys
from collections import deque

# Visualization settings
WINDOW_WIDTH = 900
WINDOW_HEIGHT = 900
FPS = 60
CENTER_OFFSET = (WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2)
EGO_COLOR = (0, 128, 255)
EGO_SHADOW_COLOR = (0, 70, 150, 100)
OTHER_COLOR = (255, 64, 64)
OTHER_SHADOW_COLOR = (150, 20, 20, 100)
MAP_WAYPOINT_COLOR = (200, 200, 200)
BACKGROUND_COLOR = (20, 20, 40)
GRID_COLOR = (40, 40, 60)
EVENT_COLOR = (255, 255, 0, 120)  # Yellow transparent
VEHICLE_LENGTH = 4.5  # meters approx
VEHICLE_WIDTH = 2.0   # meters approx
ZOOM_STEP = 1.1
MIN_SCALE = 1.0
MAX_SCALE = 30.0
TRAIL_LENGTH = 30  # frames of trail history


def load_log(filename):
    try:
        with open(filename, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Failed parsing JSON log file {filename}: {e}")
        return []


def world_to_screen(x, y, center_loc, scale):
    dx = x - center_loc[0]
    dy = y - center_loc[1]
    screen_x = CENTER_OFFSET[0] + dx * scale
    screen_y = CENTER_OFFSET[1] - dy * scale
    return int(screen_x), int(screen_y)


def draw_vehicle(surface, color, shadow_color, x, y, yaw, scale, length=VEHICLE_LENGTH, width=VEHICLE_WIDTH):
    rect_length = int(length * scale)
    rect_width = int(width * scale)
    # Draw shadow as translucent ellipse below
    shadow_surf = pygame.Surface((rect_length + 6, rect_width + 6), pygame.SRCALPHA)
    pygame.draw.ellipse(shadow_surf, shadow_color, shadow_surf.get_rect())
    surface.blit(shadow_surf, (x - shadow_surf.get_width()//2 + 3, y - shadow_surf.get_height()//2 + 3))
    # Vehicle surface
    vehicle_surface = pygame.Surface((rect_length, rect_width), pygame.SRCALPHA)
    pygame.draw.rect(vehicle_surface, color, vehicle_surface.get_rect(), border_radius=3)
    # Outline
    pygame.draw.rect(vehicle_surface, (255, 255, 255), vehicle_surface.get_rect(), width=1, border_radius=3)
    rotated = pygame.transform.rotate(vehicle_surface, -yaw)
    rotated_rect = rotated.get_rect(center=(x, y))
    surface.blit(rotated, rotated_rect.topleft)


def draw_velocity_arrow(surface, x, y, yaw, speed_mps, scale):
    if speed_mps <= 0.01:
        return  # Too slow, no arrow
    # Arrow length scaled by speed and zoom
    arrow_len = min(30, speed_mps * 3 * scale)
    angle_rad = math.radians(-yaw)
    end_x = x + arrow_len * math.cos(angle_rad)
    end_y = y + arrow_len * math.sin(angle_rad)
    pygame.draw.line(surface, (0, 255, 0), (x, y), (end_x, end_y), 3)
    # Draw arrow head
    left_angle = angle_rad + math.radians(135)
    right_angle = angle_rad - math.radians(135)
    head_len = arrow_len * 0.25
    left_x = end_x + head_len * math.cos(left_angle)
    left_y = end_y + head_len * math.sin(left_angle)
    right_x = end_x + head_len * math.cos(right_angle)
    right_y = end_y + head_len * math.sin(right_angle)
    pygame.draw.polygon(surface, (0, 255, 0), [(end_x, end_y), (left_x, left_y), (right_x, right_y)])


def smooth_lerp(a, b, t):
    return a + (b - a) * t


class EventPopup:
    """Visual popup for events with fade-out animation."""
    def __init__(self, location, text, scale, duration=2.0):
        self.loc = location
        self.text = text
        self.duration = duration
        self.elapsed = 0.0
        self.scale = scale
        self.alpha = 255

    def update(self, dt):
        self.elapsed += dt
        # Fade out alpha after 0.5 seconds
        if self.elapsed > 0.5:
            fade_t = min(1.0, (self.elapsed - 0.5) / (self.duration - 0.5))
            self.alpha = int(255 * (1 - fade_t))

    def draw(self, surface, center_loc):
        if self.alpha <= 0:
            return
        x, y = world_to_screen(self.loc[0], self.loc[1], center_loc, self.scale)
        font = pygame.font.SysFont(None, max(18, int(14 * self.scale / 5)))
        text_surf = font.render(self.text, True, (255, 255, 0))
        text_surf.set_alpha(self.alpha)
        rect = text_surf.get_rect(center=(x, y - 20))
        # Draw glowing background oval
        glow_surf = pygame.Surface((rect.width + 12, rect.height + 8), pygame.SRCALPHA)
        pygame.draw.ellipse(glow_surf, (255, 255, 100, self.alpha // 2), glow_surf.get_rect())
        glow_rect = glow_surf.get_rect(center=(x, y - 20))
        surface.blit(glow_surf, glow_rect)
        surface.blit(text_surf, rect)


def draw_grid(surface, center_loc, scale):
    grid_spacing_meters = 10
    pixel_spacing = grid_spacing_meters * scale
    if pixel_spacing < 20:
        return  # Skip too dense grid
    # Number of vertical and horizontal lines to draw around center
    lines_hor = int(WINDOW_HEIGHT / pixel_spacing) + 2
    lines_ver = int(WINDOW_WIDTH / pixel_spacing) + 2

    start_x = CENTER_OFFSET[0] % pixel_spacing
    start_y = CENTER_OFFSET[1] % pixel_spacing

    for i in range(lines_ver):
        x = int(i * pixel_spacing + start_x)
        pygame.draw.line(surface, GRID_COLOR, (x, 0), (x, WINDOW_HEIGHT), 1)
    for i in range(lines_hor):
        y = int(i * pixel_spacing + start_y)
        pygame.draw.line(surface, GRID_COLOR, (0, y), (WINDOW_WIDTH, y), 1)


def main(logfilename):
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    try:
        world = client.get_world()
    except Exception as e:
        print(f"Failed to connect/get CARLA world: {e}")
        return

    carla_map = world.get_map()
    waypoints = carla_map.generate_waypoints(distance=5.0)

    log_data = load_log(logfilename)
    if not log_data:
        print(f"No data found in {logfilename}")
        return

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption('CARLA World Log 2D Replay - Cool Mode')
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)

    first_ego = log_data[0]['ego']['transform']['location']
    center_x = first_ego['x']
    center_y = first_ego['y']
    scale = 5.0

    paused = True
    frame_idx = 0
    running = True
    dragging = False
    last_mouse_pos = None

    # Vehicle trails: dict vehicle_id -> deque of last positions [(x,y),...]
    trails = {}

    # Active event popups
    active_events = []

    while running:
        dt = clock.tick(FPS) / 1000.0  # seconds since last frame

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_RIGHT:
                    frame_idx = min(frame_idx + 1, len(log_data) - 1)
                elif event.key == pygame.K_LEFT:
                    frame_idx = max(frame_idx - 1, 0)
                elif event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    # Reset view to ego vehicle
                    if frame_idx < len(log_data):
                        ego_loc = log_data[frame_idx]['ego']['transform']['location']
                        center_x, center_y = ego_loc['x'], ego_loc['y']
                        scale = 5.0
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:  # scroll up (zoom in)
                    scale = min(scale * ZOOM_STEP, MAX_SCALE)
                elif event.button == 5:  # scroll down (zoom out)
                    scale = max(scale / ZOOM_STEP, MIN_SCALE)
                elif event.button == 1:  # Left mouse button down -> start dragging
                    dragging = True
                    last_mouse_pos = event.pos
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    dragging = False
            elif event.type == pygame.MOUSEMOTION:
                if dragging:
                    current_pos = event.pos
                    dx = current_pos[0] - last_mouse_pos[0]
                    dy = current_pos[1] - last_mouse_pos[1]
                    # Pan the view center by inversely mapping mouse move to world coordinates
                    center_x -= dx / scale
                    center_y += dy / scale
                    last_mouse_pos = current_pos

        if not paused:
            frame_idx += 1
            if frame_idx >= len(log_data):
                frame_idx = 0  # Loop replay
                trails.clear()
                active_events.clear()

        screen.fill(BACKGROUND_COLOR)
        draw_grid(screen, (center_x, center_y), scale)

        # Draw map lane polygons for more detail
        # We'll approximate roads by thick lines connecting waypoints with same road_id and lane_id
        # Make a grouped dict by road and lane
        lane_points = {}
        for wp in waypoints:
            rd = wp.road_id
            ln = wp.lane_id
            lane_points.setdefault((rd, ln), []).append(wp.transform.location)

        for (road_id, lane_id), points in lane_points.items():
            if len(points) < 2:
                continue
            pts_screen = [world_to_screen(p.x, p.y, (center_x, center_y), scale) for p in points]
            # Draw polyline with width based on lane type (sidewalk thinner, main lane thicker)
            # Lane type codes: 1 for driving, 3 for sidewalk in CARLA API
            # For simplicity use color intensity for lane type
            lane_type = None
            # Take lane type from first waypoint (not always accurate but approx)
            for wp in waypoints:
                if wp.road_id == road_id and wp.lane_id == lane_id:
                    lane_type = wp.lane_type
                    break
            color = (100, 100, 100)
            width = 4
            if lane_type == carla.LaneType.Sidewalk:
                color = (150, 150, 150)
                width = 2
            elif lane_type == carla.LaneType.Shoulder:
                color = (120, 120, 120)
                width = 1
            elif lane_type == carla.LaneType.Driving:
                color = (60, 60, 60)
                width = 6
            pygame.draw.lines(screen, color, False, pts_screen, width)

        # Animate event popups (fade out)
        for ev in active_events[:]:
            ev.update(dt)
            if ev.alpha <= 0:
                active_events.remove(ev)

        # Draw current frame vehicles and collect trail points
        entry = log_data[frame_idx]

        # Ego vehicle
        ego_tr = entry['ego']['transform']
        ego_loc = ego_tr['location']
        ego_rot = ego_tr['rotation']
        ego_speed = math.sqrt(sum(v ** 2 for v in (entry['ego']['velocity'].values()))) if 'velocity' in entry['ego'] else 0.0

        ego_x, ego_y = world_to_screen(ego_loc['x'], ego_loc['y'], (center_x, center_y), scale)
        draw_vehicle(screen, EGO_COLOR, EGO_SHADOW_COLOR, ego_x, ego_y, ego_rot['yaw'], scale)
        draw_velocity_arrow(screen, ego_x, ego_y, ego_rot['yaw'], ego_speed, scale)

        # Update ego trail
        trails.setdefault(entry['ego']['id'], deque(maxlen=TRAIL_LENGTH)).append((ego_loc['x'], ego_loc['y']))

        # Other vehicles
        for vehicle in entry.get('other_vehicles', []):
            tr = vehicle['transform']
            loc = tr['location']
            rot = tr['rotation']
            vehicle_id = vehicle['id']
            speed = math.sqrt(vehicle['velocity']['x'] ** 2 + vehicle['velocity']['y'] ** 2 + vehicle['velocity']['z'] ** 2)
            scr_x, scr_y = world_to_screen(loc['x'], loc['y'], (center_x, center_y), scale)

            draw_vehicle(screen, OTHER_COLOR, OTHER_SHADOW_COLOR, scr_x, scr_y, rot['yaw'], scale)
            draw_velocity_arrow(screen, scr_x, scr_y, rot['yaw'], speed, scale)

            trails.setdefault(vehicle_id, deque(maxlen=TRAIL_LENGTH)).append((loc['x'], loc['y']))

        # Draw trails as semi-transparent lines
        for vehicle_id, points in trails.items():
            if len(points) > 1:
                pts_screen = [world_to_screen(p[0], p[1], (center_x, center_y), scale) for p in points]
                pygame.draw.lines(screen, (150, 150, 255, 120), False, pts_screen, 3)

        # Draw event popups for current frame (if any)
        if 'events' in entry:
            for ev in entry['events']:
                loc = ev.get('location', None)
                if loc is None:
                    continue
                text = ev.get('type', 'Event')
                descr = ev.get('description', '')
                full_text = f"{text}: {descr}" if descr else text
                # Add popup
                active_events.append(EventPopup((loc['x'], loc['y']), full_text, scale))

        # Draw existing active events
        for ev in active_events:
            ev.draw(screen, (center_x, center_y))

        # Draw center marker
        pygame.draw.circle(screen, (0, 255, 0), CENTER_OFFSET, 7, 3)

        # Info panel
        info_lines = [
            f'Frame: {frame_idx + 1}/{len(log_data)} {"Paused" if paused else "Playing"}',
            'SPACE: pause/play | L/R: step frame | ESC: quit | Scroll: zoom | Drag: pan | R: reset',
            f'Zoom: {scale:.2f}x | Center: ({center_x:.1f}, {center_y:.1f})',
            f'Ego Speed: {ego_speed:.2f} m/s'
        ]
        for i, line in enumerate(info_lines):
            txt_surf = font.render(line, True, (220, 220, 220))
            screen.blit(txt_surf, (15, 15 + 22 * i))

        pygame.display.flip()

    pygame.quit()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python replay_2d_with_events_cool.py <carla_world_log.json>")
        sys.exit(1)
    log_file = sys.argv[1]
    main(log_file)