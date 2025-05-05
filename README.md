# Carla Stage Manager

`carla_stage_manager` is a project designed to enable manual driving in the CARLA simulator using a Logitech steering wheel and pedal set on Linux systems for a realistic driving experience.

---

## Table of Contents

- [Overview](#overview)
- [Carla Setup](#carla-setup)
- [Environment Setup](#environment-setup)
- [Scene Manager](#scene-manager)
- [Manual Driving with Logitech Steering Wheel and Pedal](#manual-driving-with-logitech-steering-wheel-and-pedal)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This project integrates manual driving controls into the CARLA autonomous driving simulator, specifically supporting Logitech steering wheel and pedal sets on Linux. It offers a comprehensive scene manager that interfaces with CARLA, SUMO co-simulation, manual controls, and logging capabilities, making it an effective toolset for autonomous vehicle research and development on Linux platforms.

---

## Carla Setup

1. **Download CARLA Simulator**

   Download version 0.9.15 from the official [CARLA Releases page](https://github.com/carla-simulator/carla/releases).

2. **Install CARLA on Linux**

   Follow the Linux installation instructions provided in the official [CARLA Quickstart Guide](https://carla.readthedocs.io/en/latest/start_quickstart/).

3. **Run CARLA Simulator**

   Launch CARLA server on Linux:

   ```bash
   ./CarlaUE4.sh
   ```

4. **Verify CARLA is Running**

   Confirm the server is running by connecting via a Python client or using the built-in manual control before running this project.

---

## Environment Setup

1. **Clone the Repository**

   ```bash
   git clone https://github.com/hosfeldli/carla_stage_manager.git
   cd carla_stage_manager
   ```

2. **Install Python Dependencies**

   ```bash
   pip install -r requirements.txt
   ```

3. **Verify Logitech Drivers**

   Ensure your Logitech steering wheel and pedals are connected, drivers are installed, and devices are recognized on your Linux system.

---

## Scene Manager

The `CarlaSceneManager` is a PyQt5-based graphical user interface that enables detailed control over the CARLA simulation environment including visualization, actor management, AI toggling, and manual driving setup.

https://youtu.be/H2guuxdLKOw

## How to Run

Run the scene manager with `python scene_manager.py`, run this from inside of the custom directory within the home directory of the project.

### Key Features

- Allows selecting spawn points directly on an interactive map for precise vehicle and object placement.
- Visualizes sidewalks, buildings, props, vehicles (including ego vehicle), traffic lights, and cameras on a scalable and zoomable map interface.
- Supports spawning vehicles or static props, including random vehicle spawns.
- Spawns an ego vehicle.
- Enables AI autopilot on selected vehicles or all vehicles except ego.
- Launches manual driving control in a separate process to interactively drive the ego vehicle.
- Integrates with SUMO for traffic co-simulation and synchronization.

### Technical Details

- Built with PyQt5 for responsive GUI and real-time visualization.
- Utilizes CARLA's Python API for world and actor lifecycle management.
- Converts CARLA world coordinates to UI map coordinates with appropriate scaling and panning.
- Filters spawn points to avoid collisions on vehicle spawn.
- Manages traffic manager parameters for AI vehicle behaviors.
- Saves ego vehicle spawn location to a JSON file for persistence.

---

## Manual Driving with Logitech Steering Wheel and Pedal

This project supports manual driving of the ego vehicle using Logitech steering wheels and pedal sets on Linux.

### Instructions

1. Connect your Logitech steering wheel and pedals to your Linux system and verify they are detected and calibrated.

2. Start the CARLA simulator server:

   ```bash
   ./CarlaUE4.sh
   ```

3. Spawn the ego vehicle using the Scene Manager interface.

4. Run the manual driving controller script:

   ```bash
   python controller.py
   ```

5. **Controls:**

   - Steering wheel controls steering.
   - Pedals control throttle and braking.
   - The L2 pedal/button controls reverse gear.

---

## Usage

1. **Launch the Scene Manager**

   Run the main application script of `CarlaSceneManager`. The window splits into a map visualization on the left and control panels on the right.

2. **Select Map**

   Choose a CARLA map from the "Select Map" dropdown to load the desired environment.

3. **Explore the Map**

   - Zoom with mouse wheel.
   - Pan by clicking and dragging.
   - Click green circles representing spawn points to select a location for spawning vehicles or objects.
   - The selected spawn point is visually highlighted.

4. **Spawn Actors**

   - Select a vehicle or static blueprint from the dropdown.
   - Click "Spawn Selected Actor" to place it at the chosen spawn point.
   - For bulk spawning, specify number of vehicles and click "Spawn Random Vehicles".
   - Spawn the ego vehicle with "Spawn Ego Vehicle"; this vehicle is ready for manual control.

5. **Manage Vehicles**

   - View vehicles in the "Vehicles in World" list with detailed information.
   - Select a vehicle to enable AI control options.
   - Double-click a vehicle to attach a camera and follow it in the simulation.

6. **Toggle AI Autopilot**

   - Enable autopilot on the selected vehicle or all non-ego vehicles.
   - AI vehicles navigate using CARLA's traffic manager.

7. **Start Manual Driving**

   Once the ego vehicle is spawned, click "Start Manual Control" to run the driving interface in a separate process. Use your Logitech wheel and pedals to drive manually.

8. **SUMO Traffic Integration**

   Use "Start SUMO Traffic Control" to begin synchronized traffic simulation if SUMO environment is configured.

9. **Dynamic Scene Updates**

   The map view updates at 20 FPS, reflecting vehicle movements, traffic lights, and sensor views.

10. **Exit and Cleanup**

    Closing the Scene Manager safely destroys all spawned actors and sensors including the ego vehicle.



