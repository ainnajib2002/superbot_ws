#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class RPMCalculator(Node):
    def __init__(self):
        super().__init__('rpm_calculator')
        self.rpm_pub_l = self.create_publisher(Float64, '/wheel_l_rpm', 10)
        self.rpm_pub_r = self.create_publisher(Float64, '/wheel_r_rpm', 10)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.prev_time = self.get_clock().now()

    def listener_callback(self, msg):
        try:
            left_idx = msg.name.index('left_wheel_joint')
            right_idx = msg.name.index('right_wheel_joint')
            vel_l = msg.velocity[left_idx]
            vel_r = msg.velocity[right_idx]

            # RPM = rad/s × 60 / 2π
            rpm_l = vel_l * 60.0 / (2 * 3.14159265)
            rpm_r = vel_r * 60.0 / (2 * 3.14159265)

            self.rpm_pub_l.publish(Float64(data=rpm_l))
            self.rpm_pub_r.publish(Float64(data=rpm_r))

            self.get_logger().info(f"Left: {rpm_l:.2f} RPM | Right: {rpm_r:.2f} RPM")
        except Exception as e:
            self.get_logger().warn(f"Error reading joint_states: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = RPMCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
