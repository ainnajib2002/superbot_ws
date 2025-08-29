#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
import time

class SmartGoalSender(Node):
    def __init__(self):
        super().__init__('smart_goal_sender')

        self.goal_sent = False
        self.sending_goal = False
        self.goal_pose = self.create_goal_pose()

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Action client untuk navigate_to_pose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Timer untuk cek dan kirim goal
        self.create_timer(1.0, self.try_send_goal)

    def create_goal_pose(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = 2.0
        goal.pose.position.y = 0.0
        goal.pose.orientation.w = 1.0
        return goal

    def try_send_goal(self):
        if self.goal_sent or self.sending_goal:
            return

        self.get_logger().info('Cek transform dan server...')
        try:
            now = rclpy.time.Time()
            # Minta transform, beri timeout
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', now, timeout=Duration(seconds=1.0))

            if not self._action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Action server tidak tersedia.')
                return

            self.get_logger().info('Transform dan server tersedia. Mengirim goal...')
            self.send_goal()
            self.sending_goal = True

        except Exception as e:
            self.get_logger().warn(f'Menunggu initial pose atau transform... ({str(e)})')

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal_pose

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn('Goal ditolak oleh server. Mencoba lagi...')
                self.sending_goal = False
                return

            self.get_logger().info('Goal diterima, menunggu hasil navigasi...')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.result_callback)

        except Exception as e:
            self.get_logger().error(f'Exception saat kirim goal: {e}')
            self.sending_goal = False

    def result_callback(self, future):
        try:
            result = future.result()
            if result.status == 4:  # CANCELED
                self.get_logger().warn('Navigasi dibatalkan.')
                self.goal_sent = False
                self.sending_goal = False
            elif result.status != 0:
                self.get_logger().warn(f'Navigasi gagal, status: {result.status}')
                self.goal_sent = False
                self.sending_goal = False
            else:
                self.get_logger().info('Navigasi selesai sukses!')
                self.goal_sent = True
                self.sending_goal = False

        except Exception as e:
            self.get_logger().error(f'Gagal mendapatkan hasil navigasi: {e}')
            self.goal_sent = False
            self.sending_goal = False

def main(args=None):
    rclpy.init(args=args)
    node = SmartGoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
