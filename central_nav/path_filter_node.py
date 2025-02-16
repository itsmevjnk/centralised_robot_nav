#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy import qos

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path

import numpy as np

class PathFilterNode(Node):
    def __init__(self):
        super().__init__('path_filter')

        self.frame_id = self.declare_parameter('frame_id', '').get_parameter_value().string_value
        self.min_length_sq = self.declare_parameter('min_length', 0.05).get_parameter_value().double_value ** 2 # pre-square for performance

        self.path_pub = self.create_publisher(Path, 'path_out', qos.qos_profile_system_default)
        self.create_subscription(Path, 'path_in', self.path_cb, qos.qos_profile_system_default)

    @staticmethod
    def point_to_ndarray(point: Point) -> np.ndarray:
        return np.array([
            point.x,
            point.y,
            # point.z # 2D point only, but we can do 3D if we want to
        ])
    
    @staticmethod
    def distance_sq(a: Point, b: Point):
        vect: np.ndarray = PathFilterNode.point_to_ndarray(a) - PathFilterNode.point_to_ndarray(b)
        return vect.dot(vect) # TODO: check

    def path_cb(self, data: Path):
        filtered_poses: list[PoseStamped] = []
        for i, pose in enumerate(data.poses):
            if len(filtered_poses) > 0: # always add first waypoint in
                if PathFilterNode.distance_sq(pose.pose.position, filtered_poses[-1].pose.position) < self.min_length_sq: # shorter than min distance
                    if i == len(data.poses) - 1: # last waypoint gets priority and will be added instead
                        filtered_poses[-1] = pose
                    continue
            filtered_poses.append(pose)
        data.poses = filtered_poses # replace poses array
        if self.frame_id != '': data.header.frame_id = self.frame_id # replace frame_id (if specified) - this also makes this node a replacement for path_pub_node
        self.path_pub.publish(data) # and finally republish            

def main():
    rclpy.init()
    node = PathFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
