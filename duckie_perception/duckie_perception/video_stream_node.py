#!/usr/bin/env python3
"""
VideoStreamNode — MJPEG HTTP stream served over Flask.

Receives camera frames from /camera/image_raw (published by followlaneesp_node)
so the physical camera device is only opened once, avoiding device contention.

Stream URL: http://<robot-ip>:5000/stream
Health URL:  http://<robot-ip>:5000/health
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from flask import Flask, Response
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class VideoStreamNode(Node):
    def __init__(self):
        super().__init__('video_stream_node')

        # --- ROS2 parameters ---
        self.declare_parameter('frame_width', 320)
        self.declare_parameter('frame_height', 240)
        self.declare_parameter('stream_port', 5000)
        self.declare_parameter('stream_host', '0.0.0.0')
        self.declare_parameter('fps', 30)

        self.FRAME_WIDTH = self.get_parameter('frame_width').value
        self.FRAME_HEIGHT = self.get_parameter('frame_height').value
        self.STREAM_PORT = self.get_parameter('stream_port').value
        self.STREAM_HOST = self.get_parameter('stream_host').value
        self.FPS = self.get_parameter('fps').value

        # Lane detection colors (HSV) — for optional overlay
        self.lower_red1 = np.array([0, 110, 70])
        self.upper_red1 = np.array([8, 255, 255])
        self.lower_red2 = np.array([165, 110, 70])
        self.upper_red2 = np.array([180, 255, 255])
        self.vertical_kernel = np.ones((25, 5), np.uint8)

        # ArUco detection
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        try:
            aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, aruco_params)
            self.old_aruco = False
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.old_aruco = True

        # Frame storage (updated by ROS subscriber)
        self.frame_lock = threading.Lock()
        self.current_frame = None
        self.last_frame_time = 0.0

        # Telemetry & control state
        self.robot_enabled = False
        self.lane_detected = False
        self.lane_error = 0
        self.detected_aruco_id = None

        # Publisher to control robot enable/disable
        self.enabled_pub = self.create_publisher(Bool, '/robot_enabled', 10)

        # Subscribe to frames published by followlaneesp_node
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            1  # Only keep the latest frame — no backlog
        )

        # Flask app for MJPEG streaming
        self.app = Flask(__name__)
        self.setup_routes()

        # Start Flask in a daemon thread
        self.flask_thread = threading.Thread(
            target=lambda: self.app.run(
                host=self.STREAM_HOST,
                port=self.STREAM_PORT,
                debug=False,
                use_reloader=False,
                threaded=True
            ),
            daemon=True
        )
        self.flask_thread.start()

        self.get_logger().info(
            f'Video Stream Node started. '
            f'Stream at http://0.0.0.0:{self.STREAM_PORT}/stream'
        )

    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        """Receive a raw BGR frame from followlaneesp_node and store it."""
        try:
            # Convert ROS Image message to numpy array (bgr8)
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            ).copy()

            # Apply overlays
            frame = self.detect_lane(frame)
            frame = self.detect_aruco(frame)

            # Add FPS label
            cv2.putText(
                frame,
                f'FPS: {self.FPS}',
                (self.FRAME_WIDTH - 80, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1
            )

            with self.frame_lock:
                self.current_frame = frame
                self.last_frame_time = time.time()

        except Exception as exc:
            self.get_logger().error(f'Error processing image: {exc}')

    # ------------------------------------------------------------------
    def detect_lane(self, frame):
        """Detect red lane markings and return annotated frame."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, self.vertical_kernel)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(
                    frame, f'Lane: ({cx}, {cy})', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
                )
                self.lane_detected = True
                self.lane_error = cx - (self.FRAME_WIDTH // 2)
            else:
                self.lane_detected = False
                self.lane_error = 0
        else:
            self.lane_detected = False
            self.lane_error = 0
        return frame

    def detect_aruco(self, frame):
        """Detect ArUco markers and annotate frame."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.old_aruco:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
        else:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        if ids is not None:
            self.detected_aruco_id = int(ids[0][0])
            frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for marker_id in ids.flatten():
                cv2.putText(
                    frame, f'ID: {marker_id}', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1
                )
        else:
            self.detected_aruco_id = None
        return frame

    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    def setup_routes(self):
        @self.app.route('/stream')
        def stream():
            return Response(
                self.generate_frames(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )

        @self.app.route('/health')
        def health():
            return 'OK', 200

        @self.app.route('/api/telemetry')
        def telemetry():
            return {
                'robot_enabled': self.robot_enabled,
                'lane_detected': self.lane_detected,
                'lane_error': int(self.lane_error),
                'aruco_id': self.detected_aruco_id
            }

        @self.app.route('/api/start', methods=['POST'])
        def api_start():
            self.robot_enabled = True
            msg = Bool()
            msg.data = True
            self.enabled_pub.publish(msg)
            self.get_logger().info('Sent START command to robot')
            return {'status': 'started'}

        @self.app.route('/api/stop', methods=['POST'])
        def api_stop():
            self.robot_enabled = False
            msg = Bool()
            msg.data = False
            self.enabled_pub.publish(msg)
            self.get_logger().info('Sent STOP command to robot')
            return {'status': 'stopped'}

        @self.app.route('/')
        def index():
            html_page = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>DR. Street - Autonomous Control Hub</title>
    <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600;800&family=JetBrains+Mono:wght@400;700&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg-base: #0B0F19;
            --bg-surface: rgba(20, 26, 45, 0.6);
            --bg-card: rgba(25, 33, 56, 0.85);
            --text-main: #F3F4F6;
            --text-muted: #9CA3AF;
            --primary: #10B981; /* DR. Street Green */
            --success: #10B981;
            --error: #EF4444;
            --border-glow: rgba(16, 185, 129, 0.15);
            --glass-border: rgba(255, 255, 255, 0.08);
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }

        body {
            background-color: var(--bg-base);
            color: var(--text-main);
            font-family: 'Outfit', sans-serif;
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            align-items: center;
            overflow-x: hidden;
            background-image: 
                radial-gradient(at 0% 0%, rgba(16, 185, 129, 0.05) 0px, transparent 50%),
                radial-gradient(at 100% 100%, rgba(16, 185, 129, 0.04) 0px, transparent 50%);
        }

        header {
            width: 100%;
            max-width: 1200px;
            padding: 2.5rem 1.5rem 1rem;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }

        .logo-container {
            display: flex;
            align-items: center;
            gap: 0.75rem;
        }

        .logo-icon {
            filter: drop-shadow(0 0 8px rgba(16, 185, 129, 0.4));
        }

        .logo-text {
            display: flex;
            flex-direction: column;
            justify-content: center;
        }

        .brand-name {
            font-size: 1.5rem;
            font-weight: 800;
            line-height: 1.1;
            color: #10B981;
            letter-spacing: 0.05rem;
        }

        .brand-tagline {
            font-size: 0.75rem;
            font-weight: 400;
            color: var(--text-muted);
            letter-spacing: 0.08rem;
            margin-top: 0.1rem;
        }

        .connection-badge {
            display: flex;
            align-items: center;
            gap: 0.5rem;
            background: rgba(255, 255, 255, 0.05);
            padding: 0.5rem 1rem;
            border-radius: 9999px;
            font-size: 0.875rem;
            font-weight: 600;
            border: 1px solid var(--glass-border);
        }

        .connection-indicator {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            background-color: var(--success);
            box-shadow: 0 0 10px var(--success);
        }

        main {
            width: 100%;
            max-width: 1200px;
            padding: 1.5rem;
            display: grid;
            grid-template-columns: 7fr 5fr;
            gap: 2rem;
            flex-grow: 1;
        }

        @media (max-width: 900px) {
            main {
                grid-template-columns: 1fr;
            }
        }

        .panel {
            background: var(--bg-card);
            border-radius: 20px;
            border: 1px solid var(--glass-border);
            backdrop-filter: blur(20px);
            padding: 2rem;
            display: flex;
            flex-direction: column;
            gap: 1.5rem;
            box-shadow: 0 20px 40px rgba(0, 0, 0, 0.3);
            position: relative;
            overflow: hidden;
        }

        .panel::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            height: 2px;
            background: linear-gradient(90deg, transparent, var(--glass-border), transparent);
        }

        .panel-title {
            font-size: 1.25rem;
            font-weight: 600;
            letter-spacing: 0.05rem;
            color: var(--text-muted);
            text-transform: uppercase;
            display: flex;
            align-items: center;
            justify-content: space-between;
        }

        /* Live Stream Section */
        .video-container {
            width: 100%;
            position: relative;
            border-radius: 12px;
            overflow: hidden;
            border: 1px solid rgba(255, 255, 255, 0.1);
            background: #000;
            aspect-ratio: 4/3;
            display: flex;
            justify-content: center;
            align-items: center;
        }

        .video-feed {
            width: 100%;
            height: 100%;
            object-fit: contain;
        }

        /* Controls Section */
        .status-card {
            background: rgba(255, 255, 255, 0.03);
            border-radius: 15px;
            padding: 1.5rem;
            border: 1px solid var(--glass-border);
            display: flex;
            justify-content: space-between;
            align-items: center;
        }

        .status-info {
            display: flex;
            flex-direction: column;
            gap: 0.25rem;
        }

        .status-label {
            font-size: 0.875rem;
            color: var(--text-muted);
            text-transform: uppercase;
            letter-spacing: 0.05rem;
        }

        .status-value {
            font-size: 1.5rem;
            font-weight: 800;
            display: flex;
            align-items: center;
            gap: 0.75rem;
        }

        .status-pulse {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            transition: all 0.5s ease;
        }

        .status-pulse.idle {
            background-color: var(--text-muted);
            box-shadow: 0 0 10px rgba(156, 163, 175, 0.5);
        }

        .status-pulse.running {
            background-color: var(--success);
            box-shadow: 0 0 15px var(--success);
            animation: pulse-glow 1.5s infinite alternate;
        }

        @keyframes pulse-glow {
            0% { transform: scale(0.9); opacity: 0.6; }
            100% { transform: scale(1.15); opacity: 1; }
        }

        .button-group {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 1rem;
            margin-top: 0.5rem;
        }

        .btn {
            border: none;
            padding: 1.25rem;
            border-radius: 14px;
            font-family: 'Outfit', sans-serif;
            font-size: 1.1rem;
            font-weight: 700;
            cursor: pointer;
            transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
            text-transform: uppercase;
            letter-spacing: 0.05rem;
            display: flex;
            justify-content: center;
            align-items: center;
            gap: 0.5rem;
        }

        .btn-start {
            background: linear-gradient(135deg, #10B981 0%, #059669 100%);
            color: white;
            box-shadow: 0 4px 20px rgba(16, 185, 129, 0.2);
        }

        .btn-start:hover:not(:disabled) {
            transform: translateY(-2px);
            box-shadow: 0 6px 25px rgba(16, 185, 129, 0.4);
            background: linear-gradient(135deg, #34D399 0%, #059669 100%);
        }

        .btn-start:disabled {
            background: rgba(16, 185, 129, 0.25);
            color: rgba(255, 255, 255, 0.4);
            cursor: not-allowed;
            box-shadow: none;
        }

        .btn-stop {
            background: linear-gradient(135deg, #EF4444 0%, #DC2626 100%);
            color: white;
            box-shadow: 0 4px 20px rgba(239, 68, 68, 0.2);
        }

        .btn-stop:hover:not(:disabled) {
            transform: translateY(-2px);
            box-shadow: 0 6px 25px rgba(239, 68, 68, 0.4);
            background: linear-gradient(135deg, #F87171 0%, #DC2626 100%);
        }

        .btn-stop:disabled {
            background: rgba(239, 68, 68, 0.25);
            color: rgba(255, 255, 255, 0.4);
            cursor: not-allowed;
            box-shadow: none;
        }

        .btn:active {
            transform: translateY(1px);
        }

        /* Telemetry Section */
        .telemetry-grid {
            display: grid;
            grid-template-columns: 1fr;
            gap: 1rem;
        }

        .telemetry-card {
            background: rgba(255, 255, 255, 0.02);
            border-radius: 12px;
            padding: 1.25rem;
            border: 1px solid rgba(255, 255, 255, 0.04);
            display: flex;
            flex-direction: column;
            gap: 0.75rem;
        }

        .telemetry-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
        }

        .telemetry-name {
            font-size: 0.875rem;
            color: var(--text-muted);
            text-transform: uppercase;
            letter-spacing: 0.05rem;
        }

        .telemetry-indicator {
            font-size: 0.9rem;
            font-weight: 700;
        }

        .telemetry-value {
            font-size: 1.25rem;
            font-family: 'JetBrains Mono', monospace;
            font-weight: 700;
        }

        /* Custom alignment slider */
        .offset-bar-container {
            width: 100%;
            height: 10px;
            background: rgba(255, 255, 255, 0.05);
            border-radius: 5px;
            position: relative;
            overflow: hidden;
            border: 1px solid rgba(255, 255, 255, 0.05);
        }

        .offset-bar-center {
            position: absolute;
            left: 50%;
            top: 0;
            bottom: 0;
            width: 2px;
            background: rgba(255, 255, 255, 0.3);
            z-index: 2;
        }

        .offset-bar-fill {
            position: absolute;
            top: 0;
            bottom: 0;
            height: 100%;
            background: var(--primary);
            transition: all 0.1s ease;
            box-shadow: 0 0 8px var(--primary);
        }

        footer {
            margin-top: auto;
            padding: 2rem;
            font-size: 0.875rem;
            color: var(--text-muted);
            letter-spacing: 0.05rem;
        }
    </style>
</head>
<body>
    <header>
        <div class="logo-container">
            <svg class="logo-icon" width="44" height="44" viewBox="0 0 100 100" fill="none" xmlns="http://www.w3.org/2000/svg">
                <!-- Outer Rim (dark grey) -->
                <circle cx="50" cy="50" r="40" stroke="#374151" stroke-width="8" />
                <!-- Green highlights on the rim -->
                <path d="M14.6 30 A 36 36 0 0 1 25 18" stroke="#10B981" stroke-width="8" stroke-linecap="round" />
                <path d="M85.4 30 A 36 36 0 0 0 75 18" stroke="#10B981" stroke-width="8" stroke-linecap="round" />
                <!-- Center hub -->
                <circle cx="50" cy="50" r="14" fill="#374151" stroke="#10B981" stroke-width="4" />
                <circle cx="50" cy="50" r="5" fill="#10B981" />
                <!-- Spokes -->
                <path d="M18 50 L 36 50" stroke="#374151" stroke-width="8" />
                <path d="M64 50 L 82 50" stroke="#374151" stroke-width="8" />
                <path d="M50 64 L 50 82" stroke="#374151" stroke-width="8" />
                <path d="M38 58 L 26 70" stroke="#374151" stroke-width="8" />
                <path d="M62 58 L 74 70" stroke="#374151" stroke-width="8" />
            </svg>
            <div class="logo-text">
                <span class="brand-name">DR. Street</span>
                <span class="brand-tagline">Code your path</span>
            </div>
        </div>
        <div class="connection-badge">
            <div class="connection-indicator"></div>
            Online
        </div>
    </header>

    <main>
        <div class="panel">
            <div class="panel-title">
                Live Video Feed (CV Overlay)
            </div>
            <div class="video-container">
                <img class="video-feed" src="/stream" alt="Live Video Feed">
            </div>
        </div>

        <div class="panel">
            <div class="panel-title">
                Control Console
            </div>

            <div class="status-card">
                <div class="status-info">
                    <span class="status-label">Robot State</span>
                    <span class="status-value" id="statusText">
                        <span class="status-pulse idle" id="statusPulse"></span>
                        <span id="statusLabelText">IDLE</span>
                    </span>
                </div>
            </div>

            <div class="button-group">
                <button class="btn btn-start" id="startBtn" onclick="sendCmd('start')">
                    Start Robot
                </button>
                <button class="btn btn-stop" id="stopBtn" onclick="sendCmd('stop')" disabled>
                    Stop Robot
                </button>
            </div>

            <hr style="border: 0; border-top: 1px solid rgba(255, 255, 255, 0.08); margin: 0.5rem 0;">

            <div class="panel-title">
                Live Telemetry
            </div>

            <div class="telemetry-grid">
                <div class="telemetry-card">
                    <div class="telemetry-header">
                        <span class="telemetry-name">Lane Tracking</span>
                        <span class="telemetry-indicator" id="laneStatusIndicator" style="color: var(--error);">NO LANE</span>
                    </div>
                    <div class="offset-bar-container">
                        <div class="offset-bar-center"></div>
                        <div class="offset-bar-fill" id="offsetFill" style="left: 50%; width: 0%;"></div>
                    </div>
                    <div style="display:flex; justify-content:space-between; font-size:0.8rem; color:var(--text-muted)">
                        <span>Left</span>
                        <span id="offsetText" style="font-family:'JetBrains Mono'">0 px offset</span>
                        <span>Right</span>
                    </div>
                </div>

                <div class="telemetry-card">
                    <div class="telemetry-header">
                        <span class="telemetry-name">Road Signs (ArUco ID)</span>
                    </div>
                    <div class="telemetry-value" id="arucoValue" style="color: var(--primary);">
                        NONE
                    </div>
                </div>
            </div>
        </div>
    </main>

    <footer>
        🚗 DR. Street Autonomous Mobile Robot System | Version 1.0.0
    </footer>

    <script>
        const startBtn = document.getElementById('startBtn');
        const stopBtn = document.getElementById('stopBtn');
        const statusPulse = document.getElementById('statusPulse');
        const statusLabelText = document.getElementById('statusLabelText');
        const laneStatusIndicator = document.getElementById('laneStatusIndicator');
        const offsetFill = document.getElementById('offsetFill');
        const offsetText = document.getElementById('offsetText');
        const arucoValue = document.getElementById('arucoValue');

        function updateUI(data) {
            // Update state
            if (data.robot_enabled) {
                statusPulse.className = 'status-pulse running';
                statusLabelText.textContent = 'RUNNING';
                statusLabelText.style.color = 'var(--success)';
                startBtn.disabled = true;
                stopBtn.disabled = false;
            } else {
                statusPulse.className = 'status-pulse idle';
                statusLabelText.textContent = 'IDLE';
                statusLabelText.style.color = 'var(--text-muted)';
                startBtn.disabled = false;
                stopBtn.disabled = true;
            }

            // Update lane tracking telemetry
            if (data.lane_detected) {
                laneStatusIndicator.textContent = 'TRACKING';
                laneStatusIndicator.style.color = 'var(--success)';
                
                // Represent offset. Max width is ~50% from center. Frame width is 320, center is 160. Max offset is ~160px.
                const pct = (data.lane_error / 160) * 50; // -50% to +50%
                if (pct < 0) {
                    offsetFill.style.left = (50 + pct) + '%';
                    offsetFill.style.width = Math.abs(pct) + '%';
                } else {
                    offsetFill.style.left = '50%';
                    offsetFill.style.width = pct + '%';
                }
                offsetText.textContent = (data.lane_error > 0 ? '+' : '') + data.lane_error + ' px offset';
            } else {
                laneStatusIndicator.textContent = 'NO LANE';
                laneStatusIndicator.style.color = 'var(--error)';
                offsetFill.style.width = '0%';
                offsetFill.style.left = '50%';
                offsetText.textContent = 'Searching...';
            }

            // Update ArUco marker telemetry
            if (data.aruco_id !== null && data.aruco_id !== undefined) {
                arucoValue.textContent = 'ID: ' + data.aruco_id;
                arucoValue.style.color = 'var(--primary)';
            } else {
                arucoValue.textContent = 'NONE';
                arucoValue.style.color = 'var(--text-muted)';
            }
        }

        async function sendCmd(action) {
            try {
                const response = await fetch('/api/' + action, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' }
                });
                const data = await response.json();
                console.log(action + ' request response:', data);
                pollTelemetry();
            } catch (err) {
                console.error('Failed to send command:', err);
            }
        }

        async function pollTelemetry() {
            try {
                const response = await fetch('/api/telemetry');
                if (response.ok) {
                    const data = await response.json();
                    updateUI(data);
                }
            } catch (err) {
                console.error('Failed to poll telemetry:', err);
            }
        }

        // Poll every 300ms for responsiveness
        setInterval(pollTelemetry, 300);
        pollTelemetry();
    </script>
</body>
</html>"""
            return html_page

    def generate_frames(self):
        """Generate MJPEG frames for streaming."""
        while True:
            try:
                with self.frame_lock:
                    frame = self.current_frame

                if frame is None:
                    # No frame yet — send a placeholder so the browser doesn't stall
                    placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
                    cv2.putText(
                        placeholder, 'Waiting for camera...', (30, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1
                    )
                    frame = placeholder

                ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ret:
                    time.sleep(0.033)
                    continue

                frame_bytes = buffer.tobytes()
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n'
                    b'Content-Length: ' + str(len(frame_bytes)).encode() + b'\r\n\r\n'
                    + frame_bytes + b'\r\n'
                )
                time.sleep(1.0 / self.FPS)
            except Exception as exc:
                self.get_logger().error(f'Error generating frames: {exc}')
                time.sleep(0.033)

    def destroy_node(self):
        super().destroy_node()


def main():
    rclpy.init()
    node = VideoStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
