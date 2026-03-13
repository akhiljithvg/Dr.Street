#!/usr/bin/env python3
"""
Simulation Perception Node — Centroid-based red line follower

Detects red color in the camera image, finds the centroid of the red region,
and steers toward it.

Speed adjustable at runtime:
  ros2 topic pub /speed_input std_msgs/msg/Float64 "{data: 80.0}" --once
  ros2 param set /sim_perception_node base_speed 80.0 --no-daemon
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult

import cv2
import numpy as np


class SimPerceptionNode(Node):
    def __init__(self):
        super().__init__('sim_perception_node')

        # ---------- ROS ----------
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_cb, 10
        )
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_motor_raw', 10
        )
        self.speed_sub = self.create_subscription(
            Float64, '/speed_input', self.speed_cb, 10
        )

        # ---------- IMAGE ----------
        self.W = 320
        self.H = 240

        # ---------- LANE DETECTION (HSV red) ----------
        self.lower_red1 = np.array([0, 80, 50])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 80, 50])
        self.upper_red2 = np.array([180, 255, 255])
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        # ---------- CONTROL PARAMS ----------
        self.declare_parameter('base_speed', 15.0)
        self.declare_parameter('steer_gain', 0.50)
        self.declare_parameter('max_steer', 80.0)
        self.declare_parameter('roi_top_pct', 0.10)

        self.base_speed = self.get_parameter('base_speed').value
        self.steer_gain = self.get_parameter('steer_gain').value
        self.max_steer = self.get_parameter('max_steer').value
        self.roi_top_pct = self.get_parameter('roi_top_pct').value

        self.add_on_set_parameters_callback(self.param_cb)
        self.frame_count = 0

        self.get_logger().info(
            f'✅ Centroid perception | speed={self.base_speed} '
            f'| gain={self.steer_gain}'
        )

    # ------------------------------------------------
    def param_cb(self, params):
        for p in params:
            if p.name == 'base_speed':
                self.base_speed = p.value
                self.get_logger().info(f'🔧 Speed → {self.base_speed}')
            elif p.name == 'steer_gain':
                self.steer_gain = p.value
                self.get_logger().info(f'🔧 Gain → {self.steer_gain}')
            elif p.name == 'max_steer':
                self.max_steer = p.value
            elif p.name == 'roi_top_pct':
                self.roi_top_pct = p.value
        return SetParametersResult(successful=True)

    def speed_cb(self, msg: Float64):
        self.base_speed = msg.data
        self.get_logger().info(f'🔧 Speed → {self.base_speed} via /speed_input')

    def publish_cmd(self, speed, steer):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(steer)
        self.cmd_pub.publish(msg)

    # ------------------------------------------------
    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.frame_count += 1

        # ===== RED MASK =====
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        m2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(m1, m2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)

        # ===== CENTROID DETECTION =====
        roi_top = int(self.H * self.roi_top_pct)
        roi = mask[roi_top:, :]
        moments = cv2.moments(roi)

        if moments['m00'] > 100:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00']) + roi_top

            error = cx - (self.W // 2)
            steer = self.steer_gain * error
            steer = max(-self.max_steer, min(self.max_steer, steer))

            self.publish_cmd(self.base_speed, steer)

            cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
            cv2.line(frame, (self.W // 2, self.H), (cx, cy), (0, 255, 255), 2)
            cv2.putText(frame, f'err:{error} steer:{steer:.1f}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        else:
            self.publish_cmd(self.base_speed, 0.0)
            cv2.putText(frame, 'NO RED DETECTED', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.putText(frame, f'spd:{self.base_speed:.0f}', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.line(frame, (0, roi_top), (self.W, roi_top), (128, 128, 128), 1)

        cv2.imshow('Centroid Lane Follower', frame)
        cv2.waitKey(1)

    # ------------------------------------------------
    def destroy_node(self):
        self.publish_cmd(0.0, 0.0)
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = SimPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
