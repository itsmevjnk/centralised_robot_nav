#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy import qos

import random

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

IX_RADIUS = 0.3 # intersection radius in metres

IX_MIN_DIST = 0.05 # minimum distance between intersections
IX_MIN_SAMPLES = 1 # minimum number of samples to form a cluster (Copilot said 2, but let's try 1 for now)

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
        if pose is not None:
            self.set_pose(pose)
        else:
            self.pose: Transform | None = None

        if path is not None:
            self.set_path(path, min_path_length)
        else:
            self.waypoints: list[tuple[float, float]] = []

        # for marker representation
        self.colour = ColorRGBA(a=1.0)
        self.colour.r, self.colour.g, self.colour.b = [random.random() for i in range(3)]

        self.move = True # for visualisation
    
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
    
    @property
    def path_marker(self) -> Marker:
        marker = Marker()
        # marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.type = Marker.LINE_STRIP
        marker.points = [Point(x=x, y=y) for (x, y) in self.waypoints]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.color = self.colour
        marker.frame_locked = True
        return marker
    
    @property
    def robot_marker(self) -> Marker:
        marker = Marker()
        # marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.type = Marker.ARROW
        marker.pose.position.x = self.pose.translation.x
        marker.pose.position.y = self.pose.translation.y
        marker.pose.position.z = self.pose.translation.z
        marker.pose.orientation = self.pose.rotation
        marker.scale.x = 0.25; marker.scale.y = marker.scale.z = 0.05
        marker.color.a = 1.0
        if self.move: marker.color.g = 1.0
        else: marker.color.r = 1.0
        marker.frame_locked = True
        return marker
    
    def __repr__(self) -> str:
        return f'Robot({self.position}, {len(self.waypoints)} wpt)'
    
    def in_intersection(self, ix: Intersection) -> bool:
        return ix.distance(self.position) <= 2 * IX_RADIUS

class CentralNavigationNode(Node):
    def __init__(self):
        super().__init__('central_nav')
        
        self.path_markers_pub = self.create_publisher(MarkerArray, 'path_markers', qos.qos_profile_system_default)
        self.robot_markers_pub = self.create_publisher(MarkerArray, 'robot_markers', qos.qos_profile_system_default)
        self.pass_pub = self.create_publisher(String, 'robot_pass', qos.qos_profile_system_default)
        self.stop_pub = self.create_publisher(String, 'robot_stop', qos.qos_profile_system_default)

        self.robots: dict[str, Robot] = dict()
        self.create_subscription(TransformStamped, 'robot_poses', self.pose_cb, qos.qos_profile_system_default)
        self.create_subscription(Path, 'robot_paths', self.path_cb, qos.qos_profile_system_default)

        self.intersections: list[Intersection] = []

    def pose_cb(self, data: TransformStamped):
        robot_name = data.child_frame_id # as per pose_publisher node
        if robot_name not in self.robots:
            self.robots[robot_name] = Robot(pose=data)
        else:
            self.robots[robot_name].set_pose(data)

        self.check_intersections()
        self.publish_robot_markers()
    
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
            msg = String(data=robot_name)
            if move:
                self.pass_pub.publish(msg)
            else:
                self.stop_pub.publish(msg)
            robot.move = move # for visualisation
    
    def publish_robot_markers(self):
        markers = [Marker(action=Marker.DELETEALL)] # delete all markers
        stamp = self.get_clock().now().to_msg()
        for robot in self.robots.values():
            marker = robot.robot_marker
            marker.header.stamp = stamp
            marker.id = len(markers)
            markers.append(marker)
        self.robot_markers_pub.publish(MarkerArray(markers=markers))

    def path_cb(self, data: Path):
        robot_name = data.header.frame_id # reused field, not up to spec
        if robot_name not in self.robots:
            self.robots[robot_name] = Robot(path=data)
        else:
            self.robots[robot_name].set_path(data)

        # self.get_logger().info(f'received path of robot {robot_name} with {len(self.robots[robot_name].waypoints)} waypoint(s)')
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

        self.publish_path_markers()
    
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


    def publish_path_markers(self):
        markers = [Marker(action=Marker.DELETEALL)] # delete all markers
        stamp = self.get_clock().now().to_msg()
        # send collision points out for visualisation
        for point in self.intersections:
            marker = point.marker
            marker.header.stamp = stamp
            marker.id = len(markers)
            markers.append(marker)
        # send paths out
        for robot in self.robots.values():
            marker = robot.path_marker
            marker.header.stamp = stamp
            marker.id = len(markers)
            markers.append(marker)
        self.path_markers_pub.publish(MarkerArray(markers=markers))

def main():
    rclpy.init()
    node = CentralNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
