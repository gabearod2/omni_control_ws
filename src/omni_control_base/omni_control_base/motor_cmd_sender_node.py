#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from adafruit_motorkit import MotorKit


class MotorCommandSenderNode(Node):
    def __init__(self):
        super().__init__('motor_command_sender')
        # Creating motor message subscription
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_cmds', 
            self.motor_cmd_callback,
            10
        )

        # Initialize motor cmd to send 
        self.motor_cmd = [0.0, 0.0, 0.0, 0.0]

    def motor_cmd_callback(self, msg):
        self.motor_cmd = msg.data
        # Initializing motor control
        kit = MotorKit()

        # Sending commands to the motors
        kit.motor1.throttle = max(min(self.motor_cmd(0), 0.9), -0.9)
        kit.motor2.throttle = max(min(self.motor_cmd(1), 0.9), -0.9)
        kit.motor3.throttle = max(min(self.motor_cmd(2), 0.9), -0.9)
        kit.motor4.throttle = max(min(self.motor_cmd(3), 0.9), -0.9)


def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandSenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()