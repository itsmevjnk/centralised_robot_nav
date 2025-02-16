#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy import qos

from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion, Pose
from std_msgs.msg import ColorRGBA

import random

class PathMarkerNode(Node):
    def __init__(self):
        super().__init__('path_marker')

        self.frame_id = self.declare_parameter('frame_id', '').get_parameter_value().string_value
        self.thickness = self.declare_parameter('thickness', 0.01).get_parameter_value().double_value
        self.alpha = self.declare_parameter('alpha', 1.0).get_parameter_value().double_value
        self.ns_suffix = self.declare_parameter('ns_suffix', '').get_parameter_value().string_value

        self.marker_pub = self.create_publisher(MarkerArray, 'marker', qos.qos_profile_system_default)
        self.create_subscription(Path, 'path', self.path_cb, qos.qos_profile_system_default)

        self.colours: dict[str, ColorRGBA] = dict()

    def path_cb(self, data: Path):     
        frame_id = data.header.frame_id
        if self.frame_id != '': data.header.frame_id = self.frame_id # frame_id override

        ns = frame_id + self.ns_suffix

        # get colour
        colour = self.colours.get(frame_id, None)
        if colour is None: # assign colour
            colour = ColorRGBA(a=self.alpha)
            colour.r, colour.g, colour.b = [random.random() for _ in range(3)]
            self.colours[frame_id] = colour
        
        self.marker_pub.publish(MarkerArray(
            markers=[
                Marker(
                    header=data.header,
                    ns=ns, id=0,
                    action=Marker.DELETEALL
                ), # delete all markers with this namespace
                Marker(
                    header=data.header,
                    ns=ns, id=1,
                    action=Marker.ADD,
                    type=Marker.LINE_STRIP,
                    color=colour,
                    points=[
                        Point(x=pose.pose.position.x, y=pose.pose.position.y, z=pose.pose.position.z)
                        for pose in data.poses
                    ],
                    pose=Pose(
                        orientation=Quaternion(w=1.0)
                    ),
                    scale=Vector3(x=self.thickness),
                    frame_locked=True
                ) # new path marker
            ]
        ))

def main():
    rclpy.init()
    node = PathMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
