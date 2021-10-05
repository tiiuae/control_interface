#!/usr/bin/python3

## Debug script for easier nav_msgs.msg.Path publishing

import os
import numpy as np
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import time as pythontime

from px4_msgs.msg import VehicleControlMode

class ControlModePublisherNode(Node):

    def __init__(self):

        DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')

        super().__init__("control_mode_publisher")
        self.publisher = self.create_publisher(VehicleControlMode, "/" + DRONE_DEVICE_ID + "/VehicleControlMode_PubSubTopic", 10)
        
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0

    def timer_callback(self):
        print('Publishing message %d' % self.i)
        self.publishManualControl()
        self.i += 1
        if self.i > 1:
            self.timer.destroy()

    def publishManualControl(self):
        msg = VehicleControlMode()
        msg.flag_armed = True
        msg.flag_control_manual_enabled = True
        msg.flag_control_auto_enabled = False
        msg.flag_control_offboard_enabled = False
        msg.flag_control_rates_enabled = False
        msg.flag_control_attitude_enabled = True
        msg.flag_control_acceleration_enabled = False
        msg.flag_control_velocity_enabled = True
        msg.flag_control_position_enabled = True
        msg.flag_control_altitude_enabled = True
        msg.flag_control_climb_rate_enabled = True
        msg.flag_control_termination_enabled = False
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlModePublisherNode()

    rclpy.spin(node)

    print('Job done')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
