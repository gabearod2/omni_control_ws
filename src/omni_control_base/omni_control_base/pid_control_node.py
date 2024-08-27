#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from msg import Box
import numpy as np
import time


# TODO: Convert to state space for better controller. 
class PIDControlNode(Node):
    def __init__(self):
        super().__init__("pid_control")

        # Creating publisher and subscription
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'motor_cmds',
            10
        )
        self.subscription = self.create_subscription(
            Box,
            'detections', 
            self.detection_callback,
            10
        )

        # Linear Control Params - Based on initial vehicle
        self.declare_parameter('use_zig_nic_lin', False)
        self.zig_nic_lin = self.get_parameter('use_zig_nic_lin').get_parameter_value().bool_value
        if self.zig_nic_lin:
            self.declare_parameter('Tu_lin', 0.00) # Ziegler-Nichols Ultimate Period (TUNING)
            self.declare_parameter('Ku_lin', 0.00) # Ziegler-Nichols Ultimate Gain (TUNING)
            self.Tu_lin = self.get_parameter('Tu_lin').get_parameter_value().double_value
            self.Ku_lin = self.get_parameter('Ku_lin').get_parameter_value().double_value
        else:
            self.declare_parameter('Kp_lin', 0.95) # Proportional Gain
            self.declare_parameter('Ki_lin', 0.00) # Integral Gain
            self.declare_parameter('Kd_lin', 0.60) # Derivative Gain
            self.Kp_lin = self.get_parameter('Kp_lin').get_parameter_value().double_value
            self.Ki_lin = self.get_parameter('Ki_lin').get_parameter_value().double_value
            self.Kd_lin = self.get_parameter('Kd_lin').get_parameter_value().double_value
### Policy Information
        # Rotational Control Params - Based on initial vehicle
        self.declare_parameter('use_zig_nic_rot', False)
        self.zig_nic_rot = self.get_parameter('use_zig_nic_rot').get_parameter_value().bool_value
        if self.zig_nic_rot:
            self.declare_parameter('Tu_rot', 1.30) # Ziegler-Nichols Ultimate Period (TUNING)
            self.declare_parameter('Ku_rot', 0.09) # Ziegler-Nichols Ultimate Gain (TUNING)
            self.Tu_rot = self.get_parameter('Tu_rot').get_parameter_value().double_value
            self.Ku_rot = self.get_parameter('Ku_rot').get_parameter_value().double_value
        else:
            self.declare_parameter('Kp_rot', 0.06) # Proportional Gain 
            self.declare_parameter('Ki_rot', 0.00) # Integral Gain
            self.declare_parameter('Kd_rot', 0.01) # Derivative Gain
            self.Kp_rot = self.get_parameter('Kp_rot').get_parameter_value().double_value
            self.Ki_rot = self.get_parameter('Ki_rot').get_parameter_value().double_value
            self.Kd_rot = self.get_parameter('Kd_rot').get_parameter_value().double_value

        # Initializing physical constants (for physical model)
        self.L = 115.54/2 # Center to axle along x [mm]
        self.H = 148.5/2 # Center to wheel along y [mm]
        self.d = 69.0 # Diameter of ball [mm]
        self.R = 67.0/2 # Radius of wheels [mm]
        self.net_pos = 100 # Net positions from center along y [mm]

        # Initializing error and time variables
        self.error = np.zeros(3) # [x, y, alpha]
        self.prev_time = time.time()
        self.last_error = np.zeros(3)
        self.integral = np.zeros(3)

    def detection_callback(self, msg):
        # Getting bounding box attributes
        self.box_center = np.array(msg.center, dtype=np.float32)
        self.box_width = np.array(msg.width, dtype=np.float32)
        self.box_height = np.array(msg.height, dtype=np.float32)

        # Accounting for close ball case
        if self.box_height > 500 or self.box_width > 500:
            self.box_center = np.zeros(2)

        # Using height or width depending on location
        if abs(self.box_center(0)-640)/640 > abs(self.box_center(1)-360)/360:
            self.mm_per_pix = self.d/self.box_height
            # Accounting for corner/half ball case
            if abs(self.box_center(0)-640)/640 > 0.95 and abs(self.box_center(1)-360)/360 > 0.95:
                self.mm_per_pix = self.d/(self.box_height*2)
        else:
            self.mm_per_pix = self.d/self.box_width
            # Accounting for corner/half ball case
            if abs(self.box_center(0)-640)/640 > 0.95 and abs(self.box_center(1)-360)/360 > 0.95:
                self.mm_per_pix = self.d/(self.box_width*2)

        # Gathering x and y errors (positional)
        self.error(0) = (self.box_center(0)-360)*self.mm_per_pix # ERROR IN THE X [mm]
        self.error(1) = -1*(self.box_center(1)-360)*self.mm_per_pix - self.net_pos# ERROR IN THE Y [mm]

        # Gathering alpha error (directional/rotational)
        if self.error(1) == 0:
            if self.error(0) == 0:
                self.error(2) == 0
            else:
                self.error(2) = np.arctan(self.error(0)/self.net_pos)
        elif (self.error(1) + self.net_pos) < 0 and self.error(0) < 0:
            self.error(2) = np.arctan(self.error(0) / (self.error(1) + self.net_pos)) - np.pi # [radians]
        elif (self.error(1) + self.net_pos) < 0 and self.error(0) > 0:
            self.error(2) = np.arctan(self.error(0) / (self.error(1) + self.net_pos)) + np.pi # [radians]
        else:
            self.error(2) = np.arctan(self.error(0) / (self.error(1) + self.net_pos)) # [radians]

        self.publish_motor_cmds()

    def publish_motor_cmds(self):
        # Calculate time elapsed
        current_time = time.time()
        dt = current_time - self.prev_time

        # Accounting for Ziegler-Nichols method
        if self.zig_nic_lin:
            self.Kp_lin = 0.6*self.Ku_lin
            self.Ki_lin = 0.00
            self.Kd_lin = self.Tu_lin/8

        if self.zig_nic_rot:
            self.Kp_rot = 0.6*self.Ku_rot
            self.Ki_rot = 0.00
            self.Kd_rot = self.Tu_rot/8

        # Accounting for the
        if dt > 0:
            # Converting alpha to omega error
            self.error(2) = -1*self.error(2)/dt

            # PID Inputs:
            Vx_p = self.error(0) * self.Kp_lin
            Vx_i += self.error(0) * dt * self.Ki_lin
            Vx_d = (self.error(0)-self.last_error(0)) * self.Kd_lin
            Vy_p = self.error(1) * self.Kp_lin
            Vy_i = self.error(1) * dt * self.Ki_lin
            Vy_d = (self.error(1)-self.last_error(1)) * self.Kd_lin
            omega_p = self.error(2) * self.Kp_rot
            omega_i = self.error(2) * dt * self.Ki_rot
            omega_d = (self.error(2)-self.last_error(2)) * self.Kd_rot

            # Summing inputs from PID
            Vx = (Vx_p + Vx_d + Vx_i) # [mm/s]
            Vy = (Vy_p + Vy_d + Vy_i) # [mm/s]
            omega = (omega_p + omega_d + omega_i) # [rad/s]

            # Necessary rotation of each wheel (Normalized)
            rot_1 = 1/self.R * (Vy + Vx - (self.L + self.H) * omega) / 18
            rot_2 = 1/self.R * (Vy - Vx + (self.L + self.H) * omega) / 18
            rot_3 = 1/self.R * (Vy - Vx - (self.L + self.H) * omega) / 18
            rot_4 = 1/self.R * (Vy + Vx + (self.L + self.H) * omega) / 18

            # Create and publish messages
            msg = Float32MultiArray()
            msg.data = [rot_1, rot_2, rot_3, rot_4]
            self.publisher.publish(msg)

            # Updating last error:
            self.last_error(0) = self.error(0)
            self.last_error(1) = self.error(1)
            self.last_error(2) = self.error(2)


def main(args=None):
    rclpy.init(args=args)
    node = PIDControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
