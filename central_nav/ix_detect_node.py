import rclpy
from rclpy.node import Node
from rclpy import qos

from .poly_point_isect import isect_segments_include_segments

from std_msgs.msg import Header
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker # reused message type

import uuid
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors

class IntersectionDetection(Node):
    def __init__(self):
        super().__init__('ix_detect')

        self.min_path_dist = self.declare_parameter('min_path_dist', 0.1).get_parameter_value().double_value
        self.ix_radius = self.declare_parameter('ix_radius', 0.6).get_parameter_value().double_value; self.ix_diameter = self.ix_radius * 2
        self.min_ix_dist = self.declare_parameter('min_ix_dist', self.ix_radius).get_parameter_value().double_value
        self.min_ix_samples = self.declare_parameter('min_ix_samples', 2).get_parameter_value().integer_value
        self.marker_height = self.declare_parameter('marker_height', 0.01).get_parameter_value().double_value
        self.map_frame = self.declare_parameter('map_frame', 'map').get_parameter_value().string_value

        self.pub = self.create_publisher(MarkerArray, 'intersections', qos.qos_profile_system_default)

        self.paths: dict[str, list[tuple[tuple[float, float], tuple[float, float]]]] = dict()
        self.create_subscription(Path, 'paths', self.path_cb, qos.qos_profile_system_default)

        self.intersections: dict[str, tuple[float, float]] = dict()

    def path_cb(self, data: Path):
        robot_name = data.header.frame_id
        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in data.poses
        ]
        self.paths[robot_name] = [
            (points[i], points[i + 1])
            for i in range(len(points) - 1)
        ]
        self.get_logger().info(f'received path of robot {robot_name} ({len(self.paths[robot_name])} segments(s))')

        self.find_intersections()

    def find_intersections(self):
        segments = []
        robot_seg_idx: dict[str, tuple[int, int]] = dict() # robot: (start, count)
        for robot_name in self.paths:
            path = self.paths[robot_name]
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
        intersections: list[tuple[float, float]] = []
        for (point, segment_idxs) in isect_segments_include_segments(segments, threshold=self.min_path_dist):
            colliding_robots = set([robot_from_seg_idx(i) for i in segment_idxs])
            if len(colliding_robots) > 1: # count number of robots in collision zone
                # self.get_logger().info(f'intersection at {point}: {colliding_robots}')
                intersections.append(point)

        intersections = self.cluster_intersections(intersections) # filter intersections
        self.replace_intersections(intersections)
        
        self.publish_intersections()
                    
    def cluster_intersections(self, intersections: list[tuple[float, float]]) -> list[tuple[float, float]]: # DBSCAN clustering
        if len(intersections) < 2: return intersections # no intersections to cluster

        algo = DBSCAN(eps=self.min_ix_dist, min_samples=self.min_ix_samples)
        labels = algo.fit_predict(intersections).tolist()

        output: list[tuple[float, float]] = []

        # group intersections by labels
        unique_labels = set(labels)
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
                mean_pos = np.mean(label_ixs, axis=0).tolist()
                # self.get_logger().info(f'mean position: {mean_pos} (positions: {[ix.position for ix in label_ixs]})')
                output.append(tuple(mean_pos))
        
        return output

    def replace_intersections(self, intersections: list[tuple[float, float]]):
        if len(intersections) == 0: # no intersections
            self.intersections = dict()
            return
        
        if len(self.intersections) == 0: # no intersections yet
            for i, point in enumerate(intersections):
                self.intersections[str(uuid.uuid4())] = point
            return
        
        old_ix_keys, old_ix = zip(*self.intersections.items())
        self.intersections = dict()
        nearest_dist, nearest_idx = NearestNeighbors(n_neighbors=1).fit(old_ix).kneighbors(intersections)
        nearest_idx = nearest_idx.astype(np.int64).flatten().tolist()
        for i, dist in enumerate(nearest_dist.flatten().tolist()):
            point = intersections[i]
            if dist <= self.ix_diameter: # if intersection is close enough to existing intersection
                self.intersections[old_ix_keys[nearest_idx[i]]] = point
            else: # create new intersection
                self.intersections[str(uuid.uuid4())] = point
    
    def publish_intersections(self):
        header = Header(
            frame_id=self.map_frame,
            stamp=self.get_clock().now().to_msg()
        )
        markers = [
            Marker(
                header=header,
                ns='', id=0,
                action=Marker.DELETEALL
            )
        ]
        for i, (id, (x, y)) in enumerate(self.intersections.items()):
            marker = Marker(
                header=header,
                type=Marker.CYLINDER,
                ns=id,
                id=i + 1,
                frame_locked=True
            )
            marker.pose.position.x = x; marker.pose.position.y = y
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = self.ix_diameter; marker.scale.z = self.marker_height
            marker.color.a = 1.0; marker.color.r = marker.color.g = marker.color.b = 0.5
            markers.append(marker)
        self.pub.publish(MarkerArray(markers=markers))

def main():
    rclpy.init()
    node = IntersectionDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
