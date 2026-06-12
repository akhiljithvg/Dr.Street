#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from gpiozero import PWMOutputDevice


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # ---------- MOTOR PINS (BCM) ----------
        # wiring: in1=20, in2=21, in3=16, in4=12
        # right motor uses 20/21 and left motor uses 16/12
        self.lf = PWMOutputDevice(16, frequency=100)
        self.lb = PWMOutputDevice(12, frequency=100)
        self.rf = PWMOutputDevice(20, frequency=100)
        self.rb = PWMOutputDevice(21, frequency=100)

        # ---------- ROS ----------
        self.sub = self.create_subscription(
            Twist,
            '/cmd_motor',
            self.cmd_cb,
            10
        )

        self.get_logger().info("✅ Motor node (gpiozero) started")

    # ------------------------------------------------
    def clamp(self, val):
        return max(0.0, min(1.0, val))

    # ------------------------------------------------
    def set_motor(self, left, right):
        # LEFT
        self.lf.value = self.clamp(left if left > 0 else 0)
        self.lb.value = self.clamp(-left if left < 0 else 0)

        # RIGHT
        self.rf.value = self.clamp(right if right > 0 else 0)
        self.rb.value = self.clamp(-right if right < 0 else 0)

    # ------------------------------------------------
    def cmd_cb(self, msg: Twist):
        # Expecting values like 0–100 from perception
        base = msg.linear.x / 100.0
        steer = msg.angular.z / 100.0

        left = base - steer
        right = base + steer

        self.set_motor(left, right)

    # ------------------------------------------------
    def destroy_node(self):
        self.set_motor(0.0, 0.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
