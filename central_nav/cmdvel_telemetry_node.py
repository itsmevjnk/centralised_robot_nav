#!/usr/bin/env python3

import rclpy
from rclpy import qos
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CmdVelTelemetryNode(Node):
    def __init__(self):
        super().__init__('cmdvel_telemetry')

        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value
        
        self.moving = None # undetermined
        self.telemetry_pub = self.create_publisher(String, 'telemetry', qos.qos_profile_system_default)
        self.create_subscription(Twist, 'cmd_vel', self.msg_cb, qos.qos_profile_system_default)

    def msg_cb(self, data: Twist):
        moving = (data.linear.x**2 + data.linear.y**2 + data.angular.z**2 > 0) # check if all values are non-zero
        if self.moving != moving:
            self.get_logger().info(f'{self.robot_name} moving status is {moving}')
            self.telemetry_pub.publish(String(data=f'{self.get_clock().now().nanoseconds}:cmdvel_telemetry:{self.robot_name},{moving}')) # telemetry format: (nanosec):cmdvel_telemetry:(robot name),(True if moving, else False)
            self.moving = moving

def main():
    rclpy.init()
    node = CmdVelTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
