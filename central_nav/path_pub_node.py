import rclpy
from rclpy.node import Node
from rclpy import qos

from nav_msgs.msg import Path

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value
        
        self.pub = self.create_publisher(Path, 'path_out', qos.qos_profile_system_default)
        self.create_subscription(Path, 'path_in', self.msg_cb, qos.qos_profile_system_default)

    def msg_cb(self, data: Path):
        data.header.frame_id = self.robot_name
        self.pub.publish(data)

def main():
    rclpy.init()
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
