#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np


from adafruit_motorkit import MotorKit

# Initializing motor control
kit = MotorKit()