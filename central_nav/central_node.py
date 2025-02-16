#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy import qos

import random

import rclpy.publisher

from .poly_point_isect import isect_segments_include_segments

from geometry_msgs.msg import TransformStamped, Transform, Pose, Point
from nav_msgs.msg import Path
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

import numpy as np
from sklearn.cluster import DBSCAN

try:
    from typing import Self
except ImportError: # <= Python 3.10
    Self = object

IX_RADIUS = 0.6 # intersection radius in metres

IX_MIN_DIST = IX_RADIUS # minimum distance between intersections
IX_MIN_SAMPLES = 2 # minimum number of samples to form a cluster (Copilot said 2, but let's try 1 for now)

class Intersection:
    def __init__(self, position: tuple[float, float], occupant: str | None = None):
        self.position = position
        self.occupant = occupant
    
    @property
    def occupied(self) -> bool:
        return self.occupant is not None
    
    def can_enter(self, robot: str) -> bool:
        return self.occupant is None or self.occupant == robot # either unoccupied, or occupied by us

    def enter(self, robot: str) -> bool: # returns whether robot can proceed
        if self.can_enter(robot):
            self.occupant = robot
            return True
        else:
            return False
    
    def leave(self, robot: str):
        if self.occupant == robot: # we're originally occupying the intersection
            self.occupant = None
    
    def distance(self, position: tuple[float, float]) -> float:
        x, y = position
        x0, y0 = self.position
        return ((x-x0)**2 + (y-y0)**2)**0.5
    
    def closest_intersection(self, intersections: list[Self]) -> tuple[int, Self]:
        min_distance = float('inf')
        min_ix = None
        for (idx, ix) in enumerate(intersections):
            distance = self.distance(ix.position)
            if distance < min_distance:
                min_distance = distance
                min_ix = (idx, ix)
        
        return min_ix

    @property
    def marker(self):
        marker = Marker()
        # marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.type = Marker.CYLINDER
        marker.pose.position.x, marker.pose.position.y = self.position   
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = IX_RADIUS * 2; marker.scale.z = 0.01
        marker.color.a = 1.0
        if self.occupied:
            marker.color.r = 0.5
        else:
            marker.color.g = 0.5
        marker.frame_locked = True
        return marker
    
    def __repr__(self) -> str:
        return f'Intersection({self.position}, occupant: {self.occupant})'

class Robot:
    def __init__(self, pose: TransformStamped | None = None, path: Path | None = None, min_path_length: float = 0.05):
        if path is not None:
            self.set_path(path, min_path_length)
        else:
            self.waypoints: list[tuple[float, float]] = []

        if pose is not None:
            self.set_pose(pose)
        else:
            self.pose: Transform | None = None

        # for marker representation
        self.colour = ColorRGBA(a=1.0)
        self.colour.r, self.colour.g, self.colour.b = [random.random() for i in range(3)]

        self.move = True # for visualisation
        self.accept_path = True
    
    @property
    def position(self) -> tuple[float, float] | None:
        if self.pose is not None:
            return (self.pose.translation.x, self.pose.translation.y)
        else:
            return None

    @property
    def has_pose(self) -> bool:
        return self.pose is not None
    
    @property
    def has_path(self) -> bool:
        return len(self.waypoints) > 1
    
    def set_pose(self, data: TransformStamped):
        self.pose = data.transform
            
    @staticmethod
    def path_to_waypoints(data: Path, min_length: float = 0.05) -> list[tuple[float, float]]:
        # path filtering has been moved to path_marker_node
        return [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in data.poses
        ]

    def set_path(self, data: Path, min_length: float = 0.05):
        self.waypoints = Robot.path_to_waypoints(data, min_length)

    @property
    def path(self) -> list[tuple[tuple[float, float], tuple[float, float]]]:
        if len(self.waypoints) < 2: return None
        return [(self.waypoints[i], self.waypoints[i + 1]) for i in range(len(self.waypoints) - 1)] # connect points together into segments
    
    @staticmethod
    def waypoints_to_marker(waypoints: list[tuple[float, float]], scale: float = 0.01, colour: ColorRGBA | None = None) -> Marker:
        marker = Marker()
        # marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.type = Marker.LINE_STRIP
        marker.points = [Point(x=x, y=y) for (x, y) in waypoints]
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        if colour is not None: marker.color = colour
        marker.frame_locked = True
        return marker
    
    def __repr__(self) -> str:
        return f'Robot({self.position}, {len(self.waypoints)} wpt)'
    
    def in_intersection(self, ix: Intersection) -> bool:
        return ix.distance(self.position) <= IX_RADIUS

    def clear_path(self):
        self.waypoints = []

class CentralNavigationNode(Node):
    def __init__(self):
        super().__init__('central_nav')

        self.telemetry = self.declare_parameter('telemetry', True).get_parameter_value().bool_value
        if self.telemetry:
            self.telemetry_pub = self.create_publisher(String, 'telemetry', qos.qos_profile_system_default)
        
        self.ix_markers_pub = self.create_publisher(MarkerArray, 'ix_markers', qos.qos_profile_system_default)

        self.pass_pub = self.create_publisher(String, 'robot_pass', qos.qos_profile_system_default)
        self.stop_pub = self.create_publisher(String, 'robot_stop', qos.qos_profile_system_default)

        self.robots: dict[str, Robot] = dict()
        self.create_subscription(TransformStamped, 'robot_poses', self.pose_cb, qos.qos_profile_system_default)
        self.create_subscription(Path, 'robot_paths', self.path_cb, qos.qos_profile_system_default)

        self.path_oneshot = self.declare_parameter('oneshot', False).get_parameter_value().bool_value

        self.intersections: list[Intersection] = []

    def pose_cb(self, data: TransformStamped):
        robot_name = data.child_frame_id # as per pose_publisher node
        if robot_name not in self.robots:
            self.robots[robot_name] = Robot(pose=data)
        else:
            self.robots[robot_name].set_pose(data)

        self.check_intersections()
    
    def check_intersections(self):
        for robot_name in self.robots:
            robot = self.robots[robot_name]
            move = True
            stop_ixes = []
            for ix in self.intersections: # go through intersections
                if robot.in_intersection(ix): # entering
                    if not ix.can_enter(robot_name):
                        move = False # cannot enter - stop now
                        stop_ixes.append(ix)
                    else:
                        ix.enter(robot_name) # enter intersection
                else: # leaving
                    ix.leave(robot_name)
            
            # command robot to move or stop
            if robot.move != move:
                self.get_logger().info(f'commanding robot {robot_name} to ' + ('move' if move else f'STOP (against intersection(s) {stop_ixes})')) # avoid polluting logs
                if self.telemetry:
                    self.telemetry_pub.publish(String(data=f'{self.get_clock().now().nanoseconds}:central_nav:{robot_name},{move}')) # telemetry format: (nanosec):central_nav:(robot name),(True if commanded to move, else False)
            msg = String(data=robot_name)
            if move:
                self.pass_pub.publish(msg)
            else:
                self.stop_pub.publish(msg)
            robot.move = move # for visualisation
    
        self.publish_ix_markers()

    def publish_ix_markers(self):
        markers = [Marker(action=Marker.DELETEALL)] # delete all markers
        stamp = self.get_clock().now().to_msg()
        # send collision points out for visualisation
        for point in self.intersections:
            marker = point.marker
            marker.header.stamp = stamp
            marker.id = len(markers)
            markers.append(marker)
        self.ix_markers_pub.publish(MarkerArray(markers=markers))

    def path_cb(self, data: Path):
        robot_name = data.header.frame_id # reused field, not up to spec
        if robot_name not in self.robots:
            self.robots[robot_name] = Robot(path=data)
        else:
            robot = self.robots[robot_name]
            if len(data.poses) == 0:
                if not robot.move: # empty path sent out while the robot is commanded to stop - goal cancellation
                    self.get_logger().info(f'ignoring empty path from {robot_name} caused by goal cancellation')
                    robot.accept_path = True # so we accept the next path
                    return
                else:
                    self.get_logger().info(f'{robot_name} navigation ended')
                    robot.clear_path()
                    # return
            else:
                if self.path_oneshot and not robot.accept_path:
                    return
                robot.set_path(data)
                robot.accept_path = False

        # self.get_logger().info(f'received path of robot {robot_name} with {len(self.robots[robot_name].waypoints)} waypoint(s)')
        self.robots[robot_name].accept_path = len(robot.waypoints) < 2 # if we have less than 2 waypoints, we'll want to get new path
        self.find_intersections()

    def find_intersections(self):
        segments = []
        robot_seg_idx: dict[str, tuple[int, int]] = dict() # robot: (start, count)
        for robot_name in self.robots:
            robot = self.robots[robot_name]
            path = robot.path
            if path is not None:
                robot_seg_idx[robot_name] = (len(segments), len(path))
                segments.extend(path) # add segments

        def robot_from_seg_idx(idx):
            for robot_name in robot_seg_idx:
                start, count = robot_seg_idx[robot_name]
                if idx >= start and idx < start + count:
                    return robot_name
            
            self.get_logger().error(f'cannot get robot corresponding to segment index {idx}')
            return None

        # save collision points
        intersections: list[Intersection] = []
        for (point, segment_idxs) in isect_segments_include_segments(segments):
            colliding_robots = set([robot_from_seg_idx(i) for i in segment_idxs])
            if len(colliding_robots) > 1: # count number of robots in collision zone
                # self.get_logger().info(f'intersection at {point}: {colliding_robots}')
                intersections.append(Intersection(point))

        intersections = self.cluster_intersections(intersections) # filter intersections

        # migrate intersections
        old_intersections = filter(lambda ix: ix.occupied, self.intersections) # only consider occupied intersections
        for ix in intersections:
            old_ix = ix.closest_intersection(old_intersections)
            if old_ix is not None:
                old_ix_idx, old_ix = old_ix
                ix_dist = ix.distance(old_ix.position)
                if ix_dist <= 2 * IX_RADIUS: # intersections overlap
                    ix.occupant = old_ix.occupant if self.robots[old_ix.occupant].in_intersection(ix) else None # existing robot in intersection gets priority (since it's moving already) - TODO
        self.intersections = intersections

        if self.telemetry:
            self.telemetry_pub.publish(String(data=f'{self.get_clock().now().nanoseconds}:central_nav:ix,{len(self.intersections)}'))
    
    def cluster_intersections(self, intersections: list[Intersection]) -> list[Intersection]: # DBSCAN clustering
        if len(intersections) < 2: return intersections # no intersections to cluster

        positions = np.array([ix.position for ix in intersections])
        algo = DBSCAN(eps=IX_MIN_DIST, min_samples=IX_MIN_SAMPLES)
        labels = algo.fit_predict(positions).tolist()

        output: list[Intersection] = []

        # group intersections by labels
        unique_labels = set(labels)
        ix_labels: dict[int, Intersection] = dict()
        for label in unique_labels:
            label_ixs = [
                intersections[i]
                for i in range(len(intersections))
                if labels[i] == label
            ] # intersections with this label
            # self.get_logger().info(f'label {label}: {label_ixs}')
            if label == -1: # noise - we'll still include it anyway
                output.extend(label_ixs)
            else: # not noise - take mean of their positions
                mean_pos = np.mean([ix.position for ix in label_ixs], axis=0).tolist()
                # self.get_logger().info(f'mean position: {mean_pos} (positions: {[ix.position for ix in label_ixs]})')
                output.append(Intersection(tuple(mean_pos)))
        
        return output

def main():
    rclpy.init()
    node = CentralNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
