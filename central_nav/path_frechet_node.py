#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path

import threading

import numpy as np
from .frechetdist import frdist

class RobotPath:
    def __init__(self, path: Path | None = None):
        self.path = path
        self.lock = threading.RLock()
    
    @staticmethod
    def path_to_points(path: Path) -> list[tuple[float, float]]:
        return [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in path.poses
        ]
    
    def distance(self, other_path: Path) -> float:
        if len(other_path.poses) < 2: return float('inf') # so that we'll immediately accept the new path
        with self.lock:
            if self.path is None or len(self.path.poses) < 2: return float('inf')
            return frdist(
                RobotPath.path_to_points(self.path),
                RobotPath.path_to_points(other_path)
            )

class PathFrechetNode(Node):
    def __init__(self):
        super().__init__('path_frechet')

        self.threshold = self.declare_parameter('threshold', 0.5).get_parameter_value().double_value
        
        self.pub = self.create_publisher(Path, 'paths_out', qos.qos_profile_system_default)

        self.cb_group = ReentrantCallbackGroup()
        self.create_subscription(Path, 'paths_in', self.path_cb, qos.qos_profile_system_default, callback_group=self.cb_group)

        self.paths: dict[str, RobotPath] = dict()
    
    def path_cb(self, data: Path):
        robot_name = data.header.frame_id 

        publish = False
        if robot_name not in self.paths: # first path
            self.get_logger().info(f'first path received for robot {robot_name}')
            self.paths[robot_name] = RobotPath(path=data)
            publish = True
        else: # subsequent path - calculate Frechet distance and swap if necessary
            robot_path = self.paths[robot_name]
            with robot_path.lock: # remember to acquire lock!!!
                distance = robot_path.distance(data)
                self.get_logger().info(f'received new path for robot {robot_name}, df = {distance}')
                if distance > self.threshold:
                    publish = False if len(robot_path.path.poses) < 2 and len(data.poses) < 2 else True # do not repeatedly publish empty paths
                    robot_path.path = data
        
        if publish:
            self.pub.publish(data)

def main():
    rclpy.init()
    node = PathFrechetNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
