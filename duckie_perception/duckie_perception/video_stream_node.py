#!/usr/bin/env python3

import threading
import time
from io import BytesIO

import cv2
import numpy as np
import rclpy
from flask import Flask, Response
from rclpy.node import Node


class VideoStreamNode(Node):
    def __init__(self):
        super().__init__('video_stream_node')

        # --- ROS2 parameters ---
        self.declare_parameter('video_device', 0)
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

        # Lane detection colors (HSV)
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

        # Initialize camera
        self.cap = cv2.VideoCapture(self.get_parameter('video_device').value)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_HEIGHT)
        time.sleep(0.5)

        # Frame lock for thread safety
        self.frame_lock = threading.Lock()
        self.current_frame = None

        # Flask app for streaming
        self.app = Flask(__name__)
        self.setup_routes()

        # Start capture thread
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

        # Start Flask server in separate thread
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
            f'Video Stream Node started. Stream available at http://0.0.0.0:{self.STREAM_PORT}/stream'
        )

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

    def detect_lane(self, frame):
        """Detect red lane markings and return annotated frame."""
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create red mask
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Morphological operations
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, self.vertical_kernel)

        # Find contours
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contours on frame
        cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)

        # Calculate lane position (if lane detected)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(
                    frame,
                    f'Lane: ({cx}, {cy})',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1
                )

        return frame

    def detect_aruco(self, frame):
        """Detect ArUco markers and annotate frame."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.old_aruco:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
            if ids is not None:
                frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for i, marker_id in enumerate(ids.flatten()):
                    cv2.putText(
                        frame,
                        f'ID: {marker_id}',
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 255),
                        1
                    )
        else:
            detector_result = self.aruco_detector.detectMarkers(gray)
            corners = detector_result[0]
            ids = detector_result[1]
            if ids is not None:
                frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for i, marker_id in enumerate(ids.flatten()):
                    cv2.putText(
                        frame,
                        f'ID: {marker_id}',
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 255),
                        1
                    )

        return frame

    def capture_loop(self):
        """Continuously capture frames and apply overlays."""
        while True:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().warn('Failed to read frame from camera')
                    time.sleep(0.1)
                    continue

                # Apply OpenCV overlays
                frame = self.detect_lane(frame)
                frame = self.detect_aruco(frame)

                # Add FPS counter
                cv2.putText(
                    frame,
                    f'FPS: {self.FPS}',
                    (self.FRAME_WIDTH - 80, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1
                )

                # Update current frame with lock
                with self.frame_lock:
                    self.current_frame = frame.copy()

                # Control frame rate
                time.sleep(1.0 / self.FPS)
            except Exception as e:
                self.get_logger().error(f'Error in capture loop: {e}')
                time.sleep(0.1)

    def generate_frames(self):
        """Generate MJPEG frames for streaming."""
        while True:
            try:
                with self.frame_lock:
                    if self.current_frame is None:
                        time.sleep(0.01)
                        continue
                    frame = self.current_frame.copy()

                # Encode frame as JPEG
                ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                frame_bytes = buffer.tobytes()

                # Yield frame in MJPEG format
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n'
                    b'Content-Length: ' + str(len(frame_bytes)).encode() + b'\r\n\r\n'
                    + frame_bytes + b'\r\n'
                )
            except Exception as e:
                self.get_logger().error(f'Error generating frames: {e}')
                time.sleep(0.01)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = VideoStreamNode()
    
    # Don't spin - Flask runs in separate thread
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
