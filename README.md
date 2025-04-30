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

This project integrates manual driving controls into the Carla autonomous driving simulator, specifically supporting Logitech steering wheel and pedal sets for an immersive driving experience.

---

## Carla Setup

1. **Download Carla Simulator**

   Visit the [Carla Releases](https://github.com/carla-simulator/carla/releases) page and download the latest stable version suitable for your OS.

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
   git clone https://github.com/yourusername/carla_stage_manager.git
   cd carla_stage_manager
   ```

2. **Create a Python Virtual Environment**

   ```bash
   python3 -m venv venv
   source venv/bin/activate   # On Windows use `venv\Scripts\activate`
   ```

3. **Install Dependencies**

   ```bash
   pip install -r requirements.txt
   ```

4. **Install Carla Python API**

   Make sure you have the compatible Python API for Carla installed, usually available in the Carla package or via pip:

   ```bash
   pip install carla
   ```

---

## Manual Driving with Logitech Steering Wheel and Pedal

This project supports manual driving using a Logitech steering wheel and pedal set.

### Steps to Use

1. **Connect Your Logitech Steering Wheel and Pedals**

2. **Calibrate Devices**

   Depending on the OS, calibrate your wheel and pedals via system settings or Logitech software.

3. **Run the Manual Driving Script**

   With Carla running, execute:

   ```bash
   python manual_driving.py --input_device logitech
   ```

4. **Controls**

   - Steering wheel controls steering.
   - Pedals control acceleration and braking.
   - Additional buttons on the steering wheel may correspond to other actions (refer to the script documentation).

---

## Usage

- Make sure Carla simulator is running.
- Activate your Python environment.
- Run the manual driving script with your input device as argument.

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
