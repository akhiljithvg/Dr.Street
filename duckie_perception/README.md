# duckie_perception

This ROS 2 package contains the active perception node for the Duckie robot.

The current node in use is:

- `duckie_perception/duckie_perception/followlaneesp_node.py`

It is responsible for:

- reading the USB camera stream
- detecting red lane markings
- detecting ArUco junction markers
- sending motor commands over serial to an ESP32

## Main README

See the main project README for build and launch instructions:

[../README.md](../README.md)
