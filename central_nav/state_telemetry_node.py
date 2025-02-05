import rclpy
from rclpy.node import Node
from rclpy import qos

from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray, GoalStatus

class NavStateTelemetryNode(Node):
    def __init__(self):
        super().__init__('nav2_cancel_stopper')
        
        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value
        
        self.active = None # undetermined
        self.telemetry_pub = self.create_publisher(String, 'telemetry', qos.qos_profile_system_default)

        latched_qos = qos.QoSProfile(
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            depth=1,
            durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        ) # same as bt_navigator
        self.create_subscription(GoalStatusArray, 'goal_status', self.status_cb, latched_qos)
        
    def status_cb(self, data: GoalStatusArray):
        if len(data.status_list) == 0: return # nothing to do

        latest_status: GoalStatus = data.status_list[-1] # newest goal is last
        self.get_logger().info(f'latest goal status: {latest_status.status}')

        if latest_status.status == 0 or latest_status.status == 5: # 4 = succeeded, 5 = canceled, 6 = aborted
            return # we don't really care about cancelled goals (since that'll happen a lot), and also unknown statuses

        active = latest_status.status < 4 # 1 = accepted, 2 = executing, 3 = cancelling
        if self.active != active:
            self.get_logger().info(f'{self.robot_name} navigation status is {active}')
            self.telemetry_pub.publish(String(data=f'{self.get_clock().now().nanoseconds}:state_telemetry:{self.robot_name},{active},{latest_status.status}'))
            self.active = active
            
def main():
    rclpy.init()
    node = NavStateTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
