#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math
import time


class SerialMotorNode(Node):

    def __init__(self):
        super().__init__('serial_motor_node')

        # ---------------- ROBOT PARAMETERS ----------------
        self.declare_parameter('wheel_radius', 0.05)      # meters
        self.declare_parameter('wheel_separation', 0.30)  # meters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_separation').value
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # ---------------- SERIAL ----------------
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2)
        self.get_logger().info('Connected to Arduino')

        # ---------------- SUBSCRIBER ----------------
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        v = msg.linear.x       # m/s
        w = msg.angular.z      # rad/s

        # Differential drive inverse kinematics
        v_left = (v - (w * self.L / 2.0)) / self.R
        v_right = (v + (w * self.L / 2.0)) / self.R

        cmd = f"V {v_left:.3f} {v_right:.3f}\n"
        self.ser.write(cmd.encode())

        self.get_logger().info(cmd.strip())


def main(args=None):
    rclpy.init(args=args)
    node = SerialMotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
