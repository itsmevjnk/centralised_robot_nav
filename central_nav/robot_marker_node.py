#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy import qos

from geometry_msgs.msg import TransformStamped, Vector3
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg import Time

class Robot:
    def __init__(self, pose: TransformStamped | None = None, state: bool = True, arr_length: float = 0.25, arr_width = 0.05):
        self.pose = pose
        self.state = state
        self.scale = Vector3(
            x=arr_length,
            y=arr_width,
            z=arr_width
        )
    
    @property
    def marker(self) -> Marker | None:
        if self.pose is None: return None

        marker = Marker(
            action=Marker.ADD,
            type=Marker.ARROW,
            scale=self.scale,
            color=ColorRGBA(
                a=1.0,
                r=1.0 if not self.state else 0.0,
                g=1.0 if self.state else 0.0
            ),
            frame_locked=True
        )
        marker.pose.position.x = self.pose.transform.translation.x
        marker.pose.position.y = self.pose.transform.translation.y
        marker.pose.position.z = self.pose.transform.translation.z
        marker.pose.orientation = self.pose.transform.rotation
        
        return marker

class RobotMarkerNode(Node):
    def __init__(self):
        super().__init__('robot_marker')
        
        self.map_frame = self.declare_parameter('map_frame', 'map').get_parameter_value().string_value
        self.arr_length = self.declare_parameter('arr_length', 0.25).get_parameter_value().double_value
        self.arr_width = self.declare_parameter('arr_width', 0.05).get_parameter_value().double_value

        self.marker_pub = self.create_publisher(MarkerArray, 'marker', qos.qos_profile_system_default)

        self.create_subscription(String, 'robot_pass', self.pass_cb, qos.qos_profile_system_default)
        self.create_subscription(String, 'robot_stop', self.stop_cb, qos.qos_profile_system_default)
        self.create_subscription(TransformStamped, 'robot_poses', self.pose_cb, qos.qos_profile_system_default)

        self.robots: dict[str, Robot] = dict()

    def pass_cb(self, data: String):
        self.state_cb(data, True)

    def stop_cb(self, data: String):
        self.state_cb(data, False)
    
    def state_cb(self, data: String, state: bool):
        robot_name = data.data
        if robot_name not in self.robots:
            self.robots[robot_name] = Robot(
                state=state,
                arr_length=self.arr_length,
                arr_width=self.arr_width
            )
        else:
            self.robots[robot_name].state = state
        self.update_marker(robot_name)
    
    def pose_cb(self, data: TransformStamped):
        robot_name = data.header.frame_id
        if robot_name not in self.robots:
            self.robots[robot_name] = Robot(
                pose=data,
                arr_length=self.arr_length,
                arr_width=self.arr_width
            )
        else:
            self.robots[robot_name].pose = data
        self.update_marker(robot_name, data.header.stamp)

    def update_marker(self, name: str, stamp: Time | None = None):
        marker = self.robots[name].marker
        if marker is None: return # insufficient info to start drawing marker

        if stamp is None: stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp

        marker.ns = name; marker.id = 0 # ID 0 will be delete all markers

        self.marker_pub.publish(MarkerArray(
            markers=[
                # Marker(
                #     header=marker.header,
                #     ns=name, id=0,
                #     action=Marker.DELETEALL
                # ), # delete all markers with this namespace
                marker # new robot marker
            ]
        ))

def main():
    rclpy.init()
    node = RobotMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
