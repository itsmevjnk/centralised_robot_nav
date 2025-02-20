import rclpy
from rclpy.node import Node
from rclpy import qos

from nav_msgs.msg import Path
from action_msgs.msg import GoalStatusArray, GoalStatus

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_erase')

        self.map_frame = self.declare_parameter('map_frame', 'map').get_parameter_value().string_value
        
        self.pub = self.create_publisher(Path, 'path', qos.qos_profile_system_default)

        latched_qos = qos.QoSProfile(
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            depth=1,
            durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        ) # same as bt_navigator
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status_cb, latched_qos)

    def path_cb(self, data: Path):
        # data.header.frame_id = self.robot_name
        self.pub.publish(data)
    
    def status_cb(self, data: GoalStatusArray):
        if len(data.status_list) == 0: return # nothing to do

        latest_status: GoalStatus = data.status_list[-1] # newest goal is last
        self.get_logger().info(f'latest goal status: {latest_status.status}')
        if latest_status.status == 4 or latest_status.status == 6: # 4 = succeeded, 5 = canceled, 6 = aborted
            path = Path()
            path.header.stamp = self.get_clock().now().to_msg() # not really needed
            path.header.frame_id = self.map_frame
            path.poses = [] # no poses - erase path
            self.pub.publish(path)

def main():
    rclpy.init()
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
