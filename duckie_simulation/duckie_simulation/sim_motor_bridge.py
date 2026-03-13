#!/usr/bin/env python3
"""
Simulation Motor Bridge Node

Replaces the hardware duckie_motor node in simulation.
Subscribes to /cmd_motor (Twist with 0-100 range values from perception/safety)
and converts to standard /cmd_vel (m/s and rad/s) for the Gazebo diff-drive plugin.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimMotorBridge(Node):
    def __init__(self):
        super().__init__('sim_motor_bridge')

        # Conversion factors: perception sends speed/steer in 0-100 range
        # Convert to reasonable m/s and rad/s for the simulated robot
        self.declare_parameter('linear_scale', 0.008)    # 100 -> 3.0 m/s
        self.declare_parameter('angular_scale', 0.01)    # 100 -> 5.0 rad/s

        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value

        self.sub = self.create_subscription(
            Twist, '/cmd_motor', self.cmd_cb, 10
        )
        self.pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.get_logger().info(
            f'🚀 Sim motor bridge started '
            f'(linear_scale={self.linear_scale}, '
            f'angular_scale={self.angular_scale})'
        )

    def cmd_cb(self, msg: Twist):
        out = Twist()
        out.linear.x = msg.linear.x * self.linear_scale
        out.angular.z = -msg.angular.z * self.angular_scale  # negative to match convention
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SimMotorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
