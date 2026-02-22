# 🚀 Servo Control Interface for ROS 2

A comprehensive, low-latency interface for controlling a physical servo motor via an Arduino Serial connection within the ROS 2 ecosystem. This package provides a unified custom Graphical User Interface (GUI), real-time physical actuation, and simultaneous RViz2 3D visualization.

![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.10%2B-brightgreen)
![Hardware](https://img.shields.io/badge/Hardware-Arduino-teal)

## ✨ Features

- **🎛️ Unified Custom GUI Panel (`servo_panel.py`)** 
  A centralized, intuitive graphical interface allowing users to adjust servo joint angles using precise sliders and predefined quick-action buttons.
  
- **🔌 Low-Latency Serial Node (`servo_node.py`)**
  Establishes a seamless serial connection with an Arduino microcontroller. It subscribes to simulated joint states and commands the physical hardware with precise, clamped degree mapping.
  
- **👁️ Real-Time RViz2 Integration** 
  Visualizes the servo movement in a simulated 3D environment, moving in perfect lockstep with the physical hardware.
  
- **🎨 Stylized Console Feedback** 
  Enrich your development experience with beautiful custom ASCII art and clean, structured logging to the console upon launch.

---

## 📋 Requirements

To run this package, you will need:
- **ROS 2** (Built and tested on Humble Hawksbill)
- **Python 3.10+**
- Python Dependencies: `pyserial`
- **Hardware**: An Arduino configured for serial communication, connected typically on `/dev/ttyUSB0`

---

## 🛠️ Installation

1. **Clone this repository** into your ROS 2 workspace `src` directory:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/dilip-2006/servo.git
   ```

2. **Build the package** using `colcon`:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select servo
   ```

3. **Source the setup file**:
   ```bash
   source install/setup.bash
   ```

---

## 🚀 Usage

To launch the unified Graphical User Interface (GUI), start the serial communication node, and open the RViz 3D visualization simulator simultaneously, run the main launch file:

```bash
ros2 launch servo servo_gui.launch.py
```

*Move the slider in the popup UI window to actuate both the simulated URDF joint model and the real, physical servo at the same time!*

---

## 👤 Author

<div align="center">

### **Dilip Kumar S**
*Robotics Developer*

[![Email](https://img.shields.io/badge/Email-letsmaildilip%40gmail.com-red?style=for-the-badge&logo=gmail)](mailto:letsmaildilip@gmail.com)
[![GitHub](https://img.shields.io/badge/GitHub-dilip--2006-black?style=for-the-badge&logo=github)](https://github.com/dilip-2006)

---

*If this project helps or inspires your robotics journey, a ⭐ on the repository would mean a lot!*

**[⭐ Star this Repository](https://github.com/dilip-2006/gesture_mobile_robot)**

</div>
