#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy import qos
import rclpy.publisher

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String, Header
from visualization_msgs.msg import MarkerArray, Marker

class Intersection:
    def __init__(self, position: tuple[float, float], diameter: float, occupant: str | None = None):
        self.position = position
        self.radius_sq = (diameter / 2)**2
        self.occupant = occupant

    @property
    def radius(self) -> float:
        return (self.radius_sq)**0.5
    
    @radius.setter
    def radius(self, value: float):
        self.radius_sq = value**2
    
    @property
    def diameter(self) -> float:
        return self.radius * 2
    
    @diameter.setter
    def diameter(self, value: float):
        self.radius = value / 2

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
    
    def distance_sq(self, position: tuple[float, float]) -> float:
        x, y = position
        x0, y0 = self.position
        return (x-x0)**2 + (y-y0)**2

    def distance(self, position: tuple[float, float]) -> float:
        return self.distance_sq(position)**0.5

    def __repr__(self) -> str:
        return f'Intersection({self.position}, radius: {self.radius}, occupant: {self.occupant})'
    
    @property
    def marker(self) -> Marker:
        marker = Marker(
            type=Marker.CYLINDER,
            frame_locked=True
        )
        marker.pose.position.x, marker.pose.position.y = self.position
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = self.diameter
        marker.color.a = 1.0
        if self.occupied: marker.color.r = 1.0
        else: marker.color.g = 1.0
        return marker

class CentralNavigationNode(Node):
    def __init__(self):
        super().__init__('central_nav')

        self.telemetry = self.declare_parameter('telemetry', True).get_parameter_value().bool_value
        if self.telemetry:
            self.telemetry_pub = self.create_publisher(String, 'telemetry', qos.qos_profile_system_default)
        
        self.map_frame = self.declare_parameter('map_frame', 'map').get_parameter_value().string_value
        self.marker_height = self.declare_parameter('marker_height', 0.01).get_parameter_value().double_value
        self.markers_pub = self.create_publisher(MarkerArray, 'ix_markers_out', qos.qos_profile_system_default)
        self.create_subscription(MarkerArray, 'ix_markers_in', self.intersection_cb, qos.qos_profile_system_default)

        self.pass_pub = self.create_publisher(String, 'robot_pass', qos.qos_profile_system_default)
        self.stop_pub = self.create_publisher(String, 'robot_stop', qos.qos_profile_system_default)

        self.last_cmd: dict[str, bool] = dict() # this is pretty much for logging only
        self.create_subscription(TransformStamped, 'robot_poses', self.pose_cb, qos.qos_profile_system_default)
        
        self.intersections: dict[str, Intersection] = dict()
    
    def update_marker(self, id: str):
        header = Header(
            stamp = self.get_clock().now().to_msg(),
            frame_id = self.map_frame
        )

        marker = self.intersections[id].marker
        marker.header = header
        marker.ns = id; marker.id = 1; marker.scale.z = self.marker_height

        self.markers_pub.publish(MarkerArray(markers=[
            Marker(
                header=header,
                ns=id, id=0,
                action=Marker.DELETEALL
            ),
            marker
        ]))

    def update_all_markers(self):
        header = Header(
            stamp = self.get_clock().now().to_msg(),
            frame_id = self.map_frame
        )

        markers = [
            Marker(
                header=header,
                ns='', id=0,
                action=Marker.DELETEALL
            )
        ]

        for id in self.intersections:
            marker = self.intersections[id].marker
            marker.header = header
            marker.ns = id; marker.id = len(markers); marker.scale.z = self.marker_height
            markers.append(marker)
        
        self.markers_pub.publish(MarkerArray(markers=markers))


    def pose_cb(self, data: TransformStamped):
        robot_name = data.child_frame_id # as per pose_publisher node
        if robot_name not in self.last_cmd:
            self.last_cmd[robot_name] = True # moving by default
        
        position = (data.transform.translation.x, data.transform.translation.y) # robot position

        # check through all intersections
        stop_ixes: list[str] = []
        for ix_id in self.intersections:
            ix = self.intersections[ix_id]
            prev_occupied = ix.occupied
            if ix.distance_sq(position) < ix.radius_sq: # entering
                # self.get_logger().info(f'robot {robot_name} is in intersection {ix_id}')
                if not ix.can_enter(robot_name):
                    stop_ixes.append(ix_id)
                else:
                    ix.enter(robot_name)
            else: # leaving
                ix.leave(robot_name)
            if ix.occupied != prev_occupied:
                self.update_marker(ix_id)
        
        move = len(stop_ixes) == 0
        if self.last_cmd[robot_name] != move:
            self.get_logger().info(f'commanding robot {robot_name} to ' + ('move' if move else f'STOP (against intersection(s) {stop_ixes})')) # avoid polluting logs
            if self.telemetry:
                self.telemetry_pub.publish(String(data=f'{self.get_clock().now().nanoseconds}:central_nav:{robot_name},{move}')) # telemetry format: (nanosec):central_nav:(robot name),(True if commanded to move, else False)
        
        msg = String(data=robot_name)
        if move:
            self.pass_pub.publish(msg)
        else:
            self.stop_pub.publish(msg)
        self.last_cmd[robot_name] = move
    
    def intersection_cb(self, data: MarkerArray):
        new_intersections: dict[str, Intersection] = dict()
        for marker in data.markers:
            marker: Marker
            id = marker.ns
            ix_object = None
            if id in self.intersections: # existing intersection
                ix_object = self.intersections[id]
                ix_object.position = (marker.pose.position.x, marker.pose.position.y)
                ix_object.diameter = marker.scale.x
            else: # new intersection
                ix_object = Intersection(
                    position=(marker.pose.position.x, marker.pose.position.y),
                    diameter=marker.scale.x
                )
                self.get_logger().info(f'new intersection {id} at {ix_object.position}')
            new_intersections[id] = ix_object
        self.intersections = new_intersections

        self.update_all_markers()

        if self.telemetry:
            self.telemetry_pub.publish(String(data=f'{self.get_clock().now().nanoseconds}:central_nav:ix,{len(self.intersections)}'))

def main():
    rclpy.init()
    node = CentralNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
