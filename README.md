# 🦆 Duckie Autonomous Robot (ROS 2)

A modular **ROS 2** robotics project for autonomous lane following, ArUco junction navigation, and serial motor control.

This repository is structured for maintainability and practical ROS 2 development on Ubuntu-based embedded platforms.

---

## Overview

The current production entry point is the `followlaneesp_node` ROS 2 node in `duckie_perception`.
It reads a USB camera stream, detects red lane markings and ArUco junction markers, and sends motor commands over serial to an ESP32 controller.

Legacy standalone Python scripts remain in the repository for offline testing and reference.

---

## Repository Structure

- `duckie_bringup/`
  - ROS 2 package containing the launch file for the active node
- `duckie_perception/`
  - Main ROS 2 perception package with `followlaneesp_node`
- `duckie_motor/`
  - ROS 2 motor control package
- `duckie_safety/`
  - ROS 2 motor watchdog package
- `duckie_simulation/`
  - Simulation resources, Gazebo configuration, and launch assets
- Root Python scripts:
  - `aruco.py`, `followlane*.py`, `followlaneesp.py`
  - Preserved for legacy or experimentation, not required by the current ROS 2 launch path

---

## Current Active Node

- `duckie_perception/duckie_perception/followlaneesp_node.py`
- Console entrypoint: `followlaneesp_node`
- Launched via: `duckie_bringup/launch/bringup.launch.py`

---

## Requirements

- Ubuntu with a ROS 2 installation
- `colcon` build tool
- `python3`
- OpenCV
- `pyserial`
- ROS 2 packages: `rclpy`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`, `launch_ros`
- USB camera
- ESP32 or serial motor driver on `/dev/ttyS0`

---

## Build Instructions

From the workspace root:

```bash
cd /home/pi/ak_ws/src/duckie
source /opt/ros/<ros2-distro>/setup.bash
colcon build --packages-select duckie_perception duckie_bringup
source install/setup.bash
```

Replace `<ros2-distro>` with your ROS 2 distribution, e.g. `humble`, `iron`, or `galactic`.

---

## Run Instructions

Launch the robot with:

```bash
ros2 launch duckie_bringup bringup.launch.py
```

Or run the perception node directly:

```bash
ros2 run duckie_perception followlaneesp_node
```

---

## Notes

- The legacy `perception_node.py` file was removed and is not part of the current ROS 2 workflow.
- The active node uses serial communication to the ESP32 rather than onboard GPIO PWM.
- Keep the root scripts for diagnostics and standalone experiments if needed.

---

## License

No license is included in this repository. Add one before publishing or distributing the project.
