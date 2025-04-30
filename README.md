# Carla Stage Manager

`carla_stage_manager` is a project designed to enable manual driving in the Carla simulator using a Logitech steering wheel and pedal set.

---

## Table of Contents

- [Overview](#overview)
- [Carla Setup](#carla-setup)
- [Environment Setup](#environment-setup)
- [Manual Driving with Logitech Steering Wheel and Pedal](#manual-driving-with-logitech-steering-wheel-and-pedal)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This project integrates manual driving controls into the Carla autonomous driving simulator, specifically supporting Logitech steering wheel and pedal sets for an immersive driving experience. All of this is wrapped up into the scene manager that interfaces with SUMO, CARLA, manual controls, and logging capabilities.

---

## Carla Setup

1. **Download Carla Simulator**

   Visit the [Carla Releases](https://github.com/carla-simulator/carla/releases), this was built on Version 0.9.15

2. **Install Carla**

   - For Linux or Windows, follow the installation instructions provided in the official [Carla Documentation](https://carla.readthedocs.io/en/latest/start_quickstart/).
   - Ensure that your system meets the necessary hardware requirements to run Carla smoothly.

3. **Run Carla Simulator**

   After installation, start the Carla simulator server:

   ```bash
   ./CarlaUE4.sh
   ```
   or on Windows:
   ```cmd
   CarlaUE4.exe
   ```

4. **Verify Carla is running**

   Ensure Carla is running properly by connecting to the simulator via a Python client or the built-in manual control.

---

## Environment Setup

1. **Clone the Repository**

   ```bash
   git clone https://github.com/hosfeldli/carla_stage_manager.git
   cd carla_stage_manager
   ```
   
2. **Install Dependencies**

   ```bash
   pip install -r requirements.txt
   ```

---
## Scene Manager

The `CarlaSceneManager` is a PyQt5-based GUI application that provides a comprehensive control interface for managing a CARLA simulation scene. It offers detailed visualization and interaction capabilities to manipulate the simulation environment, spawn and control vehicles, and manage AI behavior.

### Key Features

- **CARLA Client Initialization**: Connects to the CARLA server at `127.0.0.1:2000` and loads the current world and map.
- **World Cleanup**: Automatically cleans up the simulation world by removing all vehicles, pedestrians, and static objects to start fresh.
- **Spawn Point Selection**: Allows users to select and visualize spawn points on the map to place vehicles or static objects.
- **Map Visualization**:
  - Displays sidewalks, buildings, static props, spawn points, and vehicles on a scalable map.
  - Highlights the selected spawn point and ego vehicle.
  - Shows vehicle directions and speed with velocity arrows.
  - Displays traffic lights state and stop lines.
- **Vehicle Management**:
  - Spawn vehicles (random or selected blueprint) at chosen spawn points.
  - Spawn an ego vehicle with the "hero" role.
  - Enable AI autopilot on selected vehicles or all vehicles except ego.
  - Follow a vehicle with a camera sensor that is visualized on the map.
  - Maintain a vehicle list with details such as ID, type, speed, and location.
- **AI Control**:
  - Toggle AI autopilot on selected or all vehicles.
  - Traffic manager integration for speed and lane management.
- **SUMO Traffic Control Integration**: Supports launching SUMO co-simulation for traffic synchronization.
- **Manual Control Support**: Launches a manual control script in a separate process for driving the ego vehicle interactively.

### Technical Details

- Uses CARLA's Python API for world and actor management.
- Implements real-time visualization with PyQt5 graphics.
- Handles complex coordinate transformations between CARLA world coordinates and the UI map.
- Provides efficient filtering of spawn points to avoid collisions on spawn.
- Supports dynamic camera sensor attachment to vehicles for in-simulation camera views.
- Manages actors lifecycle including safe destruction on exit.

### Usage

1. **Launching the Scene Manager**
   - Run the `CarlaSceneManager` application script.
   - The main window will open with a split view: a map visualization on the left and control panels on the right.

2. **Map Selection**
   - Pick the desired CARLA map from the "Select Map" dropdown.
   - The environment will reset and load the chosen map.
   - Spawn points and static environment elements will be updated accordingly.

3. **Navigating the Map View**
   - Use the mouse wheel to zoom in and out on the map.
   - Click and drag the map to pan and explore different areas.
   - Click on green spawn point circles to select spawn locations for actors.
   - The selected spawn point is highlighted with a cyan circle.

4. **Spawning Actors**
   - Choose a blueprint from the "Select Blueprint" dropdown. This can be a vehicle, static prop, or building.
   - Click "Spawn Selected Actor" to place it at the selected spawn point on the map.
   - Spawn multiple random vehicles by entering a count and pressing "Spawn Random Vehicles".
   - Spawn the ego vehicle by selecting a suitable vehicle blueprint and clicking "Spawn Ego Vehicle".
   - Ego vehicle spawn position is saved to `ego_position.json` for persistence.

5. **Vehicle List and Controls**
   - The "Vehicles in World" list shows all spawned vehicles with their ID, type, speed, and location.
   - Select a vehicle from the list to enable AI control buttons.
   - Double-click a vehicle in the list to attach a camera sensor and follow it in the scene.
   
6. **AI Autopilot Management**
   - Enable AI autopilot on the currently selected vehicle with "Enable AI on Selected Vehicle".
   - Enable or disable AI autopilot on all vehicles except the ego vehicle via the checkbox.
   - AI vehicles will operate autonomously with traffic manager support.

7. **Manual Driving Control**
   - After spawning the ego vehicle, press "Start Manual Control" to run the manual driving interface in a separate process.
   - Use your Logitech steering wheel and pedal set to manually drive the ego vehicle within the CARLA simulation.
   - The manual control process allows real-time interaction without blocking the scene manager UI.

8. **SUMO Synchronization**
   - Press "Start SUMO Traffic Control" to launch co-simulation with SUMO for advanced traffic scenarios.
   - Ensure SUMO and required scripts are correctly set up in the configured working directory prior to use.

9. **Scene Updates**
   - The map view automatically refreshes at 20 FPS with updates on vehicle positions, velocities, traffic light states, and cameras.
   - UI sliders allow fine control of view center and rotation yaw to adjust the displayed map region and orientation.

10. **Exiting**
    - On closing the window, all spawned actors (vehicles, cameras, static objects) are safely destroyed.
    - Ego vehicle cleanup and camera destruction are handled gracefully.

---

**Note:** For best performance, ensure the CARLA server is running and accessible at the configured host and port (`127.0.0.1:2000`). Also, have the Logitech steering wheel and pedal drivers installed with necessary calibration for smooth manual driving control.

This scene manager serves as a powerful development tool for setting up and managing CARLA simulation scenarios.

---

## Manual Driving with Logitech Steering Wheel and Pedal

This project supports manual driving using a Logitech steering wheel and pedal set.

### Steps to Use

1. **Connect Your Logitech Steering Wheel and Pedals**

2. **Run the Manual Driving Script**

   With Carla running, execute:

   ```bash
   python controller.py 
   ```

3. **Controls**

   - Steering wheel controls steering.
   - Pedals control acceleration and braking.
   - L2 on the wheel controls reverse.

---

## Usage

- Make sure Carla simulator is running.
- Activate your Python environment.
- Run the manual driving script.

```bash
python manual_driving.py --input_device logitech
```

Replace `logitech` with your device identifier if different.

---

## Troubleshooting

- Ensure that the Carla server is running and accessible.
- Verify that your Logitech steering wheel drivers are correctly installed and configured.
- Check Python dependencies and Carla API compatibility.
- If the wheel or pedals are not responding, test them on another application to ensure functionality.

---

## Contributing

Contributions are welcome! Please fork the repository and submit pull requests.

---

## License

This project is licensed under the MIT License.

---

For more information, visit the [Carla official documentation](https://carla.readthedocs.io/).


Would you like me to customize this further with specific commands or details from your project?
