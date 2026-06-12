# 🦆 Duckie Autonomous Robot (ROS 2)

A beginner-friendly ROS 2 project for autonomous lane following, ArUco junction control, and serial motor control using an ESP32.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Repository Structure](#repository-structure)
- [Prerequisites](#prerequisites)
- [Setup](#setup)
  - [Install system packages](#install-system-packages)
  - [Prepare the workspace](#prepare-the-workspace)
  - [Build the packages](#build-the-packages)
  - [Source the workspace](#source-the-workspace)
- [Run](#run)
  - [Launch with ROS 2](#launch-with-ros-2)
  - [Run the node directly](#run-the-node-directly)
- [Expected behavior](#expected-behavior)
- [Troubleshooting](#troubleshooting)
- [Notes](#notes)
- [License](#license)

---

## Overview

The active ROS 2 node in this repository is:

- `duckie_perception/duckie_perception/followlaneesp_node.py`

This node:

- reads a USB camera stream
- detects red lane markings using OpenCV
- detects ArUco junction markers
- sends motor commands over serial to an ESP32

Legacy Python scripts are kept for reference only.

---

## Features

- Beginner-friendly ROS 2 project structure
- USB camera lane detection with OpenCV
- ArUco marker junction handling
- Serial motor commands to ESP32
- ROS 2 launch file workflow
- Package-level documentation

---

## Repository Structure

- `duckie_bringup/` — ROS 2 launch package. See [duckie_bringup/README.md](duckie_bringup/README.md)
- `duckie_perception/` — Active perception package. See [duckie_perception/README.md](duckie_perception/README.md)
- `duckie_motor/` — Motor control package. See [duckie_motor/README.md](duckie_motor/README.md)
- `duckie_safety/` — Safety watchdog package. See [duckie_safety/README.md](duckie_safety/README.md)
- `duckie_simulation/` — Simulation assets. See [duckie_simulation/README.md](duckie_simulation/README.md)

Root Python scripts:

- `aruco.py`, `followlane.py`, `followlane2.py`, `followlane3.py`, `followlane4.py`, `followlaneesp.py`

---

## Prerequisites

Before starting, make sure you have:

- Ubuntu or Raspberry Pi OS based on Ubuntu
- ROS 2 installed (`humble`, `iron`, or `galactic`)
- `colcon` build tool
- USB camera connected
- ESP32 or serial motor driver connected at `/dev/ttyS0`

---

## Setup

### Install system packages

```bash
sudo apt update
sudo apt install -y python3-pip python3-opencv
python3 -m pip install --user pyserial
```

Install required ROS 2 packages for your distro:

```bash
sudo apt install -y ros-<ros2-distro>-cv-bridge ros-<ros2-distro>-launch-ros
```

Replace `<ros2-distro>` with your ROS 2 distribution name.

### Prepare the workspace

```bash
cd /home/pi/ak_ws/src/duckie
```

### Build the packages

```bash
source /opt/ros/<ros2-distro>/setup.bash
colcon build --packages-select duckie_perception duckie_bringup
```

### Source the workspace

```bash
source install/setup.bash
```

---

## Run

### Launch with ROS 2

```bash
ros2 launch duckie_bringup bringup.launch.py
```

### Run the node directly

```bash
ros2 run duckie_perception followlaneesp_node
```

---

## Expected behavior

- Camera frames are captured and processed
- Red lane lines are detected
- ArUco markers are detected at junctions
- Motor commands are sent to the ESP32
- The robot pauses briefly after ArUco detection before acting

---

## Troubleshooting

- **Camera issues:** verify the USB camera is connected and accessible
- **Serial issues:** verify the ESP32 is available at `/dev/ttyS0`
- **ROS 2 node missing:** ensure `source install/setup.bash` was run after build

---

## Notes

- The legacy `perception_node.py` file is no longer used.
- Root Python scripts are preserved for reference and offline testing.

---

## License

No license is included in this repository. Add one before sharing or publishing.
