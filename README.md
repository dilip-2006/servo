# Servo Control Package

This ROS 2 package provides a comprehensive interface for controlling a servo motor via an Arduino Serial connection. It includes a user-friendly custom GUI and RViz2 visualization for simulation and real-time state feedback.

## Features

- **Custom Python GUI Panel (`servo_panel.py`)**: A centralized interface allowing users to adjust servo joint angles using intuitive sliders and predefined action buttons.
- **Serial Node (`servo_node.py`)**: Establishes a seamless, low-latency serial connection with an Arduino. It subcribes to joint states and commands the physical servo exactly to the desired coordinate.
- **RViz2 Integration**: Visualizes the servo movement in a simulated 3D environment in real-time, side-by-side with the physical movement.
- **Stylized Console Feedback**: Prints beautiful custom ASCII art and logs to the console upon launch.

## Requirements

- ROS 2 Humble (or newer)
- Python 3.10+
- `pyserial`
- An Arduino configured for serial communication on `/dev/ttyUSB0`

## Installation

1. Clone this repository into your workspace `src` directory:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/dilip-2006/servo.git
   ```

2. Build the package using `colcon`:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select servo
   ```

3. Source the setup file:
   ```bash
   source install/setup.bash
   ```

## Usage

To launch the unified Graphical User Interface (GUI), start the serial node, and open the RViz visualization all at once, simply run the launch file:

```bash
ros2 launch servo servo_gui.launch.py
```

Move the slider in the popup UI to actuate both the simulated URDF joint and the real, physical servo!

---
*Created by Dilip Kumar*
