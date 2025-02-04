#!/usr/bin/env python3

import rclpy
from rclpy import qos
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped

from scipy.spatial.transform import Rotation

class PoseTelemetryNode(Node):
    def __init__(self):
        super().__init__('pose_telemetry')
        
        # in degrees
        self.max_rx = self.declare_parameter('max_rx', 2.0).get_parameter_value().double_value
        self.max_ry = self.declare_parameter('max_ry', 2.0).get_parameter_value().double_value

        self.collision: dict[str, bool] = dict()
        self.telemetry_pub = self.create_publisher(String, 'telemetry', qos.qos_profile_system_default)
        self.create_subscription(TransformStamped, 'robot_poses', self.msg_cb, qos.qos_profile_system_default)

    def msg_cb(self, data: TransformStamped):
        rx, ry, _ = Rotation.from_quat([data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]).as_euler('xyz', degrees=True)
        collision = (abs(rx) > self.max_rx) or (abs(ry) > self.max_ry)
        robot_name = data.child_frame_id
        if self.collision.get(robot_name, None) != collision:
            self.get_logger().info(f'{robot_name} collision status is {collision}')
            self.telemetry_pub.publish(String(data=f'{self.get_clock().now().nanoseconds}:pose_telemetry:{robot_name},{collision}')) # telemetry format: (nanosec):pose_telemetry:(robot name),(True if in collision, else False)
            self.collision[robot_name] = collision

def main():
    rclpy.init()
    node = PoseTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
