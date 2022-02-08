#!/usr/bin/python3

## Debug script for easier fog_msgs.action.ControlInterfaceAction publishing

import rclpy
import math
import os
from rclpy.action import ActionClient
from rclpy.node import Node

from fog_msgs.action import ControlInterfaceAction

from nav_msgs.msg import Path as NavPath
from fog_msgs.srv import Path as SetPath
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

WAYPOINTS_LOCAL=[
    [0,0,3,1],
    [-5,4,2,2],
    [7,-5,2,3],
    [0,0,2,4]
]

WAYPOINTS_GPS=[
    [47.397708, 8.5456038, 4, 0],
    [47.3977, 8.5456038, 2, 1.5708],
    [47.39775, 8.5456, 2, -1.5708],
    [47.39758, 8.545706, 1.5, 3.14],
    [47.397708, 8.5456038, 2, -3.14]
]

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Code below should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr

    return q

class ControlInterfaceActionClient(Node):

    def __init__(self):
        super().__init__('control_interface_action_client')

        DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')

        self._action_client = ActionClient(self, ControlInterfaceAction, "/" + DRONE_DEVICE_ID + "/control_interface")

    def send_goal(self):
        path = NavPath()
        path.header.stamp = rclpy.clock.Clock().now().to_msg()
        path.header.frame_id = "world"

        for wp in WAYPOINTS_LOCAL:
            pose = PoseStamped()
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = "world"
            point = Point()
            point.x = float(wp[0])
            point.y = float(wp[1])
            point.z = float(wp[2])
            pose.pose.position = point
            q = quaternion_from_euler(0,0,wp[3])
            pose.pose.orientation = q
            path.poses.append(pose)

        goal_msg = ControlInterfaceAction.Goal()
        goal_msg.path = path
        print('Waiting for action server')
        self._action_client.wait_for_server()
        print('Action server detected')

        print('Calling action server')
        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = ControlInterfaceActionClient()

    future = action_client.send_goal()

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
