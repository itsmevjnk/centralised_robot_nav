#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy import qos

import random

from .poly_point_isect import isect_segments

from geometry_msgs.msg import TransformStamped, Transform, Pose, Point
from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker

IX_RADIUS = 0.2 # intersection radius in metres

class Robot:
    def __init__(self, pose: TransformStamped | None = None, path: Path | None = None, min_path_length: float = 0.05):
        if pose is not None:
            self.set_pose(pose)
        else:
            self.pose: Transform | None = None

        if path is not None:
            self.set_path(path, min_path_length)
        else:
            self.waypoints: list[tuple[float, float]] = []
    
    @property
    def has_pose(self) -> bool:
        return self.pose is not None
    
    @property
    def has_path(self) -> bool:
        return len(self.waypoints) > 1
    
    def set_pose(self, data: TransformStamped):
        self.pose = data.transform
    
    def set_path(self, data: Path, min_length: float = 0.05):
        self.waypoints = [] # clear out list of waypoints
        for pose in data.poses:
            pose: Pose = pose.pose
            point = (pose.position.x, pose.position.y) # point
            if len(self.waypoints) > 0:
                if ((point[0] - self.waypoints[-1][0])**2 + (point[1] - self.waypoints[-1][1])**2)**0.5 < min_length: continue # too short
            self.waypoints.append(point) # otherwise add our point in

    @property
    def path(self) -> list[tuple[tuple[float, float], tuple[float, float]]]:
        if len(self.waypoints) < 2: return None
        return [(self.waypoints[i], self.waypoints[i + 1]) for i in range(len(self.waypoints) - 1)] # connect points together into segments

class CentralNavigationNode(Node):
    def __init__(self):
        super().__init__('central_nav')
        
        self.markers_pub = self.create_publisher(MarkerArray, 'markers', qos.qos_profile_system_default)
        self.pass_pub = self.create_publisher('pass', String, qos.qos_profile_system_default)
        self.stop_pub = self.create_publisher('stop', String, qos.qos_profile_system_default)

        self.robots: dict[str, Robot] = dict()
        self.create_subscription(TransformStamped, 'robot_poses', self.pose_cb)
        self.create_subscription(Path, 'robot_paths', self.path_cb)

        self.collision_points: list[tuple[float, float]] = []

    def pose_cb(self, data: TransformStamped):
        robot_name = data.child_frame_id # as per pose_publisher node
        if robot_name not in self.robots:
            self.robots[robot_name] = Robot(pose=data)
        else:
            self.robots[robot_name].set_pose(data)

    def path_cb(self, data: Path):
        robot_name = data.header.frame_id # reused field, not up to spec
        if robot_name not in self.robots:
            self.robots[robot_name] = Robot(path=data)
        else:
            self.robots[robot_name].set_path(data)
        
        self.find_collisions()
    
    def find_collisions(self):
        segments = []
        for robot in self.robots.values(): segments.extend(robot.path()) # add segments

        self.collision_points = isect_segments(segments) # save collision points
        
        markers = [Marker(action=Marker.DELETEALL)] # delete all markers
        # send collision points out for visualisation
        for point in self.collision_points:
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            marker.type = Marker.CYLINDER
            marker.pose.position.x, marker.pose.position.y = point
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = IX_RADIUS * 2; marker.scale.z = 0.1
            marker.color.r = 1.0; marker.color.a = 1.0
            marker.frame_locked = True
            markers.append(marker)
        # send paths out
        for robot in self.robots.values():
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            marker.type = Marker.LINE_STRIP
            marker.points = [Point(x=x, y=y) for (x, y) in robot.waypoints]
            marker.scale.x = 0.1
            marker.color.r, marker.color.g, marker.color.b = [random.random() for i in range(3)]; marker.color.a = 1.0
            marker.frame_locked = True
            markers.append(marker)
        self.markers_pub.publish(MarkerArray(markers))

def main():
    rclpy.init()
    node = CentralNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
