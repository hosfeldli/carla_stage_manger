# Carla Scene Manager

This document provides an in-depth explanation of the inner workings of `scene_manager.py`, clarifying its architecture, core components, and interactions with CARLA and the GUI.

---

## Architecture Overview

`CarlaSceneManager` is a PyQt5 `QMainWindow` subclass forming the main GUI application. It acts as a controller that:

- Connects to the CARLA simulator API.
- Maintains state for the CARLA world, map, spawned actors, and UI selections.
- Handles user inputs for map navigation, object spawning, and vehicle control.
- Updates the graphical 2D map visualization continuously.
- Interfaces with external processes like SUMO and manual control scripts.

---

## Core Components

### 1. **Initialization and Setup**

- **CARLA Client and World Setup:**

  ```python
  self.client = carla.Client(CARLA_HOST, CARLA_PORT)
  self.client.set_timeout(10.0)
  self.world = self.client.get_world()
  self.map = self.world.get_map()
  self.blueprint_library = self.world.get_blueprint_library()
  ```

  This establishes connection to CARLA server and loads the current world and map data.

- **Scene Cleanup:**

  `_clean_world()` removes existing vehicles, pedestrians, and static props to start with a fresh environment. This ensures no leftover actors interfere with the tool.

- **Static Data Retrieval:**

  - `_get_static_buildings_and_props()` distinguishes between buildings and other statics for visualization.
  - `_get_sidewalks()` collects sidewalk waypoints (lane type `Sidewalk`) from the map for rendering pedestrian paths.

- **Spawn Points:**

  The map’s spawn points are cached and used to place vehicles or props. `selected_spawn_point` and `placement_transform` track the currently selected spawn location and orientation.

---

### 2. **User Interface Setup**

- The window layout splits horizontally:
  - **Left pane (`self.map_view`)** displays a pixmap rendering the map and actors.
  - **Right pane** contains controls such as dropdowns for maps and blueprints, sliders for map view manipulation, buttons for spawning, AI controls, vehicle list, and SUMO controls.

- **Sliders for View Center and Rotation**

  Control the map's center position in world coordinates and rotation yaw:

  - View center sliders manipulate panning.
  - Rotation slider rotates the placement transform’s yaw.

- **Vehicle List**

  Continuously updated with vehicles currently spawned, showing info like ID, type, speed, and location.

- **Dark Theme**

  A stylesheet is applied for consistent dark UI design.

---

### 3. **Map Visualization**

Rendering occurs in `update_scene()` which creates a QPixmap and draws using QPainter:

- **Static Features:**
  - Sidewalks drawn as light gray squares at waypoints.
  - Buildings drawn as medium gray squares around static building actors.
  - Props drawn as ellipses.

- **Spawn Points:**
  - Shown as green circles.
  - Selected spawn point is highlighted with cyan larger circle.

- **Placement Transform:**
  - Marked with a red circle and an oriented red triangle indicating yaw.

- **Waypoints:**
  - Connected by blue lines to show possible driving paths.

- **Vehicles:**
  - Drawn as oriented red (NPC) or blue (ego) rectangles.
  - Velocity shown as green arrow pointing in travel direction.
  
- **Traffic Lights:**
  - Visualized with colored lines showing the light state.
  - Stop locations connected with lines to the traffic light.

- **Follow Camera:**
  - If following a vehicle, renders a purple oriented triangle where the camera sensor is located.

- Conversion between CARLA coordinates and map pixels is handled by `carla_transform_to_qpoint()`, factoring in the current view center, scale, and window size.

---

### 4. **Input and Interaction Logic**

- **Mouse Controls on Map:**
  - Clicking near spawn points selects one (`map_mouse_press`).
  - Left-button drag pans the map (`mouseMoveEvent`).
  - Mouse wheel zooms in/out by changing `self.scale`.

- **Sliders:**
  - Adjust view center or rotation, triggering `slider_changed()` and `update_scene()`.

- **Vehicle List:**
  - Selecting a vehicle enables AI-control button.
  - Double-clicking follows the vehicle with a camera sensor (`follow_selected_vehicle`).

---

### 5. **Vehicle and Actor Management**

- **Spawn Selected Actor:**

  `spawn_selected_actor()` spawns actor based on blueprint currently selected in the combo box (`self.bp_combo`) at the `placement_transform` location.

- **Spawn Ego Vehicle:**

  `spawn_ego()` spawns the ego vehicle with role `'hero'` and saves its position in `ego_position.json` for reuse.

- **Spawn Random Vehicles:**

  Spawns vehicles randomly chosen from blueprints at free spawn points filtered to avoid colliding with existing vehicles.

- **AI Control:**

  Enables the CARLA traffic manager autopilot on the selected vehicle or all NPC vehicles (except ego) with sensible defaults on lane change, speed, and distance.

---

### 6. **External Interaction**

- **Manual Control:**

  Launches an external manual control script (`controller.py`) via subprocess, allowing keyboard/joystick driving of the ego vehicle.

- **SUMO Traffic Simulation:**

  Runs an external SUMO traffic synchronization process, integrating external traffic flow with CARLA.

---

### 7. **Utility Methods**

- **`filtered_spawn_points()`**

  Returns spawn points not currently occupied within a 2-meter radius by any vehicle to reduce spawn conflicts.

- **Drawing Helpers:**

  `draw_vehicle_rect()` and `draw_velocity_arrow()` simplify repetitive drawing logic for vehicles.

---

### 8. **Scene Updates and Clean-up**

- A QTimer runs at FPS (default 20) calling `update()` which updates the map visualization and UI vehicle list continuously.

- On window close (`closeEvent`), all spawned actors and sensors are destroyed gracefully.

---

### 9. **Script Management via UI**

– Scripts can be called from the UI using the scripts folder.

– All files placed within that file can be called from the UI.

---

## Summary of Data Flow

1. The app initializes and connects to CARLA.
2. The UI is set up with controls and the map view.
3. User interacts via:
   - Mouse on map for spawn selection.
   - Controls to change map, spawn vehicles, enable AI, etc.
4. Actions update internal state (`spawned_actors`, `selected_vehicle`, etc.).
5. The timer triggers scene redraw:
   - Fetches latest world state.
   - Draws static environment, spawn points, vehicles, traffic lights.
6. External processes launched (manual control, SUMO) as needed.
7. All data flow is coordinated to provide smooth scene management and visualization.

---

---

This detailed breakdown should help developers understand the overall design and specific feature implementations in the scene manager, facilitating easier modifications and extensions.
