import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class GoalMonitor(Node):
    def __init__(self):
        super().__init__('goal_monitor')
        self.subscription = self.create_subscription(
            String,
            '/controller_server/goal_status',
            self.controller_callback,
            10)
        
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def controller_callback(self, msg):
        if "Reached the goal!" in msg.data:
            self.get_logger().info("Controller reached goal, sending success to navigator")
            # Trigger navigation succeeded by cancelling goal
            if self._action_client.wait_for_server(timeout_sec=2.0):
                self._action_client.cancel_all_goals()
            else:
                self.get_logger().warn("Navigator action server not available!")

def main(args=None):
    rclpy.init(args=args)
    node = GoalMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
