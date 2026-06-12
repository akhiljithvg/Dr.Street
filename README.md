# 🦆 Duckie Autonomous Robot (ROS 2)

A beginner-friendly ROS 2 robotics project for autonomous lane following, ArUco marker navigation, and serial motor control via an ESP32.

This README explains every step from installing prerequisites to launching the active ROS 2 node.

---

## What this project does

The current active ROS 2 node is `duckie_perception/duckie_perception/followlaneesp_node.py`.
It:

- reads a USB camera stream
- detects red lane markings using OpenCV
- detects ArUco junction markers
- sends motor commands to an ESP32 over serial

The project also includes legacy Python scripts in the repo root for reference, but the current ROS 2 workflow uses the `followlaneesp_node` node.

---

## Prerequisites

Before you begin, make sure you have:

- Ubuntu (desktop or Raspberry Pi OS based on Ubuntu)
- ROS 2 installed (`humble`, `iron`, or `galactic`)
- `colcon` build tool installed
- a USB camera connected
- an ESP32 or serial motor driver connected at `/dev/ttyS0`

---

## Step 1: Install required system packages

Open a terminal and run:

```bash
sudo apt update
sudo apt install -y python3-pip python3-opencv
```

Next install the Python dependency:

```bash
python3 -m pip install --user pyserial
```

If you are missing ROS 2 packages, install them with your ROS 2 distro name:

```bash
sudo apt install -y ros-<ros2-distro>-cv-bridge ros-<ros2-distro>-launch-ros
```

Replace `<ros2-distro>` with your ROS 2 version, for example `humble`, `iron`, or `galactic`.

---

## Step 2: Prepare the workspace

From the repository root:

```bash
cd /home/pi/ak_ws/src/duckie
```

This project assumes your workspace is located at `/home/pi/ak_ws/src/duckie`.

---

## Step 3: Source ROS 2

Source your ROS 2 installation before building or running anything:

```bash
source /opt/ros/<ros2-distro>/setup.bash
```

If you are using a ROS 2 overlay workspace, source the overlay after building too.

---

## Step 4: Build the packages

Build only the packages currently used by the active ROS 2 node:

```bash
colcon build --packages-select duckie_perception duckie_bringup
```

If you see errors during build, read the terminal message carefully and install any missing ROS 2 dependencies.

---

## Step 5: Source the build output

After building, source the local install workspace:

```bash
source install/setup.bash
```

This makes the new ROS 2 node available to `ros2 run` and `ros2 launch`.

---

## Step 6: Run the active ROS 2 node

Start the project with the launch file:

```bash
ros2 launch duckie_bringup bringup.launch.py
```

This runs the `followlaneesp_node` node by default.

---

## Optional: Run the node directly

If you want to run just the perception node without the launch file:

```bash
ros2 run duckie_perception followlaneesp_node
```

---

## What to expect

- The node opens the camera and reads frames.
- It detects red lane markings and ArUco markers.
- It sends motor commands to the ESP32 over `/dev/ttyS0`.
- If an ArUco marker is detected, the robot pauses briefly, then performs the required action.

---

## Troubleshooting

- If the camera does not open, confirm the USB camera is connected and accessible.
- If serial communication fails, confirm the ESP32 is connected to `/dev/ttyS0` or update the parameter.
- If ROS 2 cannot find the node, make sure you sourced `install/setup.bash` after building.

---

## Notes

- The legacy `perception_node.py` file has been removed from the current workflow.
- Root scripts like `aruco.py`, `followlane.py`, and `followlaneesp.py` are kept for reference only.
- The current active ROS 2 node is `followlaneesp_node`.

---

## License

No license file is included. Add one before sharing or publishing this project.
