# Video Streaming Node

The `video_stream_node` captures video from your robot's camera, applies OpenCV overlays (lane detection and ArUco marker detection), and streams it live as an MJPEG stream that you can view in a web browser.

## Features

- **Live Video Stream**: Real-time video capture from USB camera
- **Lane Detection**: Detects red lane markings and overlays lane center position
- **ArUco Marker Detection**: Detects and displays ArUco junction markers
- **HTTP Streaming**: Access the stream from any device on your network
- **Configurable Parameters**: Adjust camera settings, resolution, FPS, and port via ROS parameters

## Installation & Build

1. Build the package:
```bash
cd ~/ak_ws
colcon build --packages-select duckie_perception
```

2. Install Flask dependency (if not already installed):
```bash
pip install flask
```

## Running the Node

### Option 1: Using Launch File (Recommended)
```bash
ros2 launch duckie_perception video_stream.launch.py
```

### Option 2: Direct Execution
```bash
ros2 run duckie_perception video_stream_node
```

## Viewing the Stream

Once the node is running, open a web browser and navigate to:
```
http://<robot-ip>:5000/stream
```

Examples:
- **Local access**: `http://localhost:5000/stream`
- **From another machine**: `http://192.168.1.100:5000/stream` (replace with your robot's IP)

## Configuration Parameters

You can override default parameters when launching:

```bash
ros2 run duckie_perception video_stream_node \
  --ros-args \
  -p video_device:=0 \
  -p frame_width:=320 \
  -p frame_height:=240 \
  -p stream_port:=5000 \
  -p stream_host:=0.0.0.0 \
  -p fps:=30
```

Or modify the launch file:

```python
# launch/video_stream.launch.py
parameters=[
    {'video_device': 0},        # Camera device index (0 for first USB camera)
    {'frame_width': 320},       # Frame width in pixels
    {'frame_height': 240},      # Frame height in pixels
    {'stream_port': 5000},      # HTTP server port
    {'stream_host': '0.0.0.0'}, # HTTP server listen address
    {'fps': 30},                # Target frames per second
],
```

## Overlays Displayed

### Lane Detection
- **Green contours**: Detected red lane markings
- **Red circle**: Center of detected lane
- **Text**: Lane center coordinates (x, y)

### ArUco Markers
- **Blue outline**: Detected ArUco marker corners
- **Yellow text**: Marker ID number

### System Info
- **Frame rate**: Current FPS in bottom-right corner

## Health Check

You can verify the node is running with:
```bash
curl http://localhost:5000/health
```

Response will be `OK` if healthy.

## Troubleshooting

### Stream not accessible
- Check if the robot is reachable: `ping <robot-ip>`
- Verify the port is correct (default 5000)
- Check firewall settings on the robot

### Camera not detected
- Verify the camera is connected: `ls -la /dev/video*`
- Check the video_device parameter (usually 0 for first camera)

### Poor performance
- Reduce resolution (decrease frame_width/frame_height)
- Reduce FPS (decrease fps parameter)
- Use a faster network connection

### No lane/marker detection
- Check lighting conditions
- Verify lane markers are red (in HSV color space)
- Adjust color thresholds in the source code if needed

## Performance Tips

For better performance on low-power devices:
1. Reduce resolution: `frame_width: 160, frame_height: 120`
2. Lower FPS: `fps: 15`
3. Reduce JPEG quality in code (line ~220): change 80 to 60
4. Run on wired network instead of WiFi

## Integration with Other Nodes

The video streaming node runs independently but shares the same camera. If you're running the `followlaneesp_node` simultaneously:
- Both will access the camera (may work depending on driver support)
- You might get better performance by running them serially or on different camera instances

## Advanced Usage

### Using with VLC Media Player
```bash
vlc http://<robot-ip>:5000/stream
```

### Recording the stream
```bash
ffmpeg -i http://localhost:5000/stream -c copy output.mp4
```

### Embedding in a web dashboard
```html
<img src="http://<robot-ip>:5000/stream" alt="Duckie Video Stream" />
```

## Code Structure

- `duckie_perception/video_stream_node.py`: Main node implementation
- `launch/video_stream.launch.py`: ROS 2 launch configuration

The node uses Flask for HTTP streaming and OpenCV for image processing. It runs the camera capture loop in a separate thread for non-blocking operation.
