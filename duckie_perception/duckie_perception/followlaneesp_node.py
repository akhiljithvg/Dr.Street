#!/usr/bin/env python3

import time

import cv2
import numpy as np
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class FollowLaneESPNode(Node):
    def __init__(self):
        super().__init__('followlaneesp_node')

        # --- ROS2 parameters ---
        # These values can be overridden by launch parameters or by ros2 run arguments.
        self.declare_parameter('video_device', 0)
        self.declare_parameter('serial_port', '/dev/ttyS0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_width', 320)
        self.declare_parameter('frame_height', 240)

        self.FRAME_WIDTH = self.get_parameter('frame_width').value
        self.FRAME_HEIGHT = self.get_parameter('frame_height').value
        self.BASE_SPEED = 45
        self.MIN_SPEED = 30
        self.STEER_GAIN = 0.50
        self.STEER_D = 0.60
        self.APPROACH_SPEED = 30
        self.ARUCO_TRIGGER_AREA = 2000

        self.lower_red1 = np.array([0, 110, 70])
        self.upper_red1 = np.array([8, 255, 255])
        self.lower_red2 = np.array([165, 110, 70])
        self.upper_red2 = np.array([180, 255, 255])

        self.vertical_kernel = np.ones((25, 5), np.uint8)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        try:
            aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, aruco_params)
            self.old_aruco = False
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.old_aruco = True

        self.tag_map = {1: 'STRAIGHT', 2: 'STRAIGHT', 3: 'RIGHT', 4: 'RIGHT', 5: 'LEFT'}

        self.last_turn_time = 0.0
        self.turn_cooldown = 1.0
        self.last_error = 0.0
        self.ema_error = 0.0
        self.error_alpha = 0.4
        self.ema_speed = 0.0
        self.speed_alpha = 0.2
        self.last_steer_value = 0.0

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value

        # --- Motor enable gate (controlled via web UI) ---
        # Starts as False — motors will not move until /robot_enabled publishes True.
        self.robot_enabled = False
        self.control_mode = 'auto'

        # Subscribe to the enable/disable topic published by video_stream_node
        self.create_subscription(Bool, '/robot_enabled', self._robot_enabled_cb, 10)
        self.create_subscription(String, '/control_mode', self._control_mode_cb, 10)
        self.create_subscription(Twist, '/cmd_manual', self._manual_cmd_cb, 10)

        # Publisher for raw camera frames to avoid device contention
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            self.get_logger().info(f'Opened serial port: {self.serial_port}@{self.baudrate}')
        except Exception as exc:
            self.get_logger().error(f'Failed to open serial port {self.serial_port}: {exc}')
            raise

        self.cap = cv2.VideoCapture(self.get_parameter('video_device').value)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_HEIGHT)
        time.sleep(0.5)

        # Timer callback runs at ~30 Hz and drives the camera/process loop.
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info('FollowLaneESP ROS2 node started — waiting for START from web UI')

    # ------------------------------------------------
    def _robot_enabled_cb(self, msg: Bool):
        """Called when the web UI Start/Stop button is pressed."""
        prev = self.robot_enabled
        self.robot_enabled = msg.data
        if self.robot_enabled and not prev:
            self.get_logger().info('🟢 Robot ENABLED — starting lane following')
        elif not self.robot_enabled and prev:
            self.stop_motor(auto_cmd=False)
            self.get_logger().info('🔴 Robot DISABLED — motors stopped')

    def _control_mode_cb(self, msg: String):
        prev = getattr(self, 'control_mode', 'auto')
        self.control_mode = msg.data
        if self.control_mode != prev:
            self.stop_motor(auto_cmd=False)
            self.get_logger().info(f'Control mode switched to {self.control_mode.upper()}')

    def _manual_cmd_cb(self, msg: Twist):
        if getattr(self, 'control_mode', 'auto') == 'manual' and self.robot_enabled:
            speed = msg.linear.x
            turn = msg.angular.z
            left = speed - turn
            right = speed + turn
            self.set_motor(right, left, auto_cmd=False)

    # ------------------------------------------------
    def clamp(self, x, lo=0, hi=100):
        return max(lo, min(hi, int(x)))

    def set_motor(self, right, left, auto_cmd=True):
        if auto_cmd and getattr(self, 'control_mode', 'auto') == 'manual':
            return
        # Convert requested motion values into the ESP32 PWM protocol.
        # The node uses a symmetric command format: left and right values are sent as integers.
        actual_left = right
        actual_right = left

        l_val = self.clamp(abs(actual_left)) / 100.0
        r_val = self.clamp(abs(actual_right)) / 100.0

        l_pwm = int(l_val * 255)
        r_pwm = int(r_val * 255)

        if actual_left < 0:
            l_pwm = -l_pwm
        if actual_right < 0:
            r_pwm = -r_pwm

        command = f"{l_pwm},{r_pwm}\n"
        try:
            self.ser.write(command.encode('utf-8'))
        except Exception as exc:
            self.get_logger().warning(f'Failed to write to serial: {exc}')

    def stop_motor(self, auto_cmd=True):
        self.set_motor(0, 0, auto_cmd=auto_cmd)

    def has_red_lane(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, self.lower_red1, self.upper_red1),
            cv2.inRange(hsv, self.lower_red2, self.upper_red2),
        )
        roi_mask = mask[self.FRAME_HEIGHT // 2 :, :]
        M = cv2.moments(roi_mask)
        return M['m00'] > 500

    def turn_until_red(self, direction):
        # Phase 1: move slightly forward to reach the intersection center.
        self.set_motor(self.BASE_SPEED, self.BASE_SPEED)
        time.sleep(0.3)

        # Phase 2: execute a smooth Ackermann-style turn arc.
        outer_speed = 60
        inner_speed = -40
        self.get_logger().info(f'Executing smooth {direction} Ackermann arc')

        if direction == 'LEFT':
            self.set_motor(inner_speed, outer_speed)
        else:
            self.set_motor(outer_speed, inner_speed)

        time.sleep(0.6)

        # Phase 3: if the red lane was not detected by the arc, pivot until the lane appears again.
        self.get_logger().info('Searching for red lane...')
        search_start = time.time()
        pivot_speed = 40

        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                continue

            if self.has_red_lane(frame):
                break

            if time.time() - search_start > 20.0:
                break

            if direction == 'LEFT':
                self.set_motor(-pivot_speed, pivot_speed)
            else:
                self.set_motor(pivot_speed, -pivot_speed)

            time.sleep(0.01)

        self.stop_motor()
        time.sleep(0.5)

    def timer_callback(self):
        # Main loop: read a camera frame, detect ArUco or red line, and command the motors.
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Camera read failed')
            return

        # Publish the raw frame to prevent device contention and allow video streaming
        try:
            img_msg = Image()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'
            img_msg.height = frame.shape[0]
            img_msg.width = frame.shape[1]
            img_msg.encoding = 'bgr8'
            img_msg.is_bigendian = 0
            img_msg.step = frame.shape[1] * 3
            img_msg.data = frame.tobytes()
            self.image_pub.publish(img_msg)
        except Exception as exc:
            self.get_logger().error(f'Failed to publish camera frame: {exc}')

        # Gate: do not move motors until the web UI sends the START command.
        if not self.robot_enabled:
            self.stop_motor(auto_cmd=True)
            return

        # If manual mode, skip all autonomous logic (ArUco, PID, Junctions)
        if getattr(self, 'control_mode', 'auto') == 'manual':
            return

        action_to_execute = None
        approaching_junction = False

        if (time.time() - self.last_turn_time) > self.turn_cooldown:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if self.old_aruco:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            else:
                corners, ids, _ = self.aruco_detector.detectMarkers(gray)

            if ids is not None:
                tid = int(ids[0][0])
                if tid in self.tag_map:
                    marker_corners = corners[0][0]
                    area = cv2.contourArea(marker_corners)
                    if area >= self.ARUCO_TRIGGER_AREA:
                        action_to_execute = self.tag_map[tid]
                    else:
                        approaching_junction = True

        if action_to_execute is not None:
            # Stop briefly after ArUco detection so the ID is confirmed before moving.
            self.stop_motor()
            time.sleep(1.0)

            if action_to_execute == 'STRAIGHT':
                self.set_motor(self.BASE_SPEED, self.BASE_SPEED)
                time.sleep(1.0)
            elif action_to_execute == 'LEFT':
                self.turn_until_red('LEFT')
            elif action_to_execute == 'RIGHT':
                self.turn_until_red('RIGHT')

            self.last_turn_time = time.time()
            self.ema_speed = 0.0
            return

        # If we are approaching a junction, slow down to make the turn detection more stable.
        current_base_speed = self.APPROACH_SPEED if approaching_junction else self.BASE_SPEED

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        raw_mask = cv2.bitwise_or(
            cv2.inRange(hsv, self.lower_red1, self.upper_red1),
            cv2.inRange(hsv, self.lower_red2, self.upper_red2),
        )

        clean_mask = cv2.morphologyEx(raw_mask, cv2.MORPH_CLOSE, self.vertical_kernel)
        steer_slice = clean_mask[80 : self.FRAME_HEIGHT, :]
        
        # --- Gap Detection for Junctions (Distance between consecutive dashes) ---
        def get_centroid(c):
            M_c = cv2.moments(c)
            if M_c['m00'] > 0:
                return np.array([M_c['m10']/M_c['m00'], M_c['m01']/M_c['m00']])
            x, y, w, h = cv2.boundingRect(c)
            return np.array([x + w/2.0, y + h/2.0])

        full_contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [c for c in full_contours if cv2.contourArea(c) > 50]
        
        is_junction_by_gap = False
        if len(valid_contours) > 1:
            contours_sorted = sorted(valid_contours, key=lambda c: get_centroid(c)[1])
            max_dist = 0
            for i in range(len(contours_sorted) - 1):
                c1 = get_centroid(contours_sorted[i])
                c2 = get_centroid(contours_sorted[i+1])
                dist = np.linalg.norm(c1 - c2)
                if dist > max_dist:
                    max_dist = dist
            
            # If the distance between any two consecutive dashes is larger than 40% of the frame height
            if max_dist > self.FRAME_HEIGHT * 0.40:
                is_junction_by_gap = True

        M = cv2.moments(steer_slice)

        if is_junction_by_gap:
            self.get_logger().info('Junction gap detected! Moving slow, straight to check ArUco.')
            self.set_motor(self.MIN_SPEED, self.MIN_SPEED)
            self.ema_error = 0.0
            self.last_error = 0.0
            self.last_steer_value = 0.0
        elif M['m00'] > 100:
            cX = int(M['m10'] / M['m00'])
            raw_error = cX - (self.FRAME_WIDTH // 2)
            self.ema_error = (self.error_alpha * raw_error) + ((1.0 - self.error_alpha) * self.ema_error)
            derivative = self.ema_error - self.last_error
            self.last_error = self.ema_error

            steer = (self.STEER_GAIN * self.ema_error) + (self.STEER_D * derivative)
            self.last_steer_value = steer

            target_speed = max(self.MIN_SPEED, current_base_speed - abs(steer) * 0.8)
            if self.ema_speed == 0:
                self.ema_speed = target_speed
            self.ema_speed = (self.speed_alpha * target_speed) + ((1.0 - self.speed_alpha) * self.ema_speed)

            self.set_motor(max(0, self.ema_speed + steer), max(0, self.ema_speed - steer))
        else:
            self.get_logger().info('Line lost, maintaining trajectory...')
            if self.control_mode == 'auto':
                reduced_speed = 25
                self.set_motor(reduced_speed + self.last_steer_value, reduced_speed - self.last_steer_value)

    def destroy_node(self):
        self.stop_motor()
        if hasattr(self, 'ser') and self.ser is not None and self.ser.is_open:
            self.ser.close()
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    # Initialize ROS2 and start the node.
    rclpy.init(args=args)
    node = FollowLaneESPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupt received, shutting down')
    finally:
        # Ensure the node cleans up the camera and serial port.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
