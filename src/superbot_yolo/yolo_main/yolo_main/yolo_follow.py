import rclpy
from rclpy.node import Node
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult  
from yolo_msg.msg import PersonInfo
from geometry_msgs.msg import Twist
import time
import math

class FollowPerson(Node):
    def __init__(self):
        super().__init__('follow_person')

        # === Parameter ===
        self.declare_parameter("following_active", False)  
        self.following_active = self.get_parameter("following_active").get_parameter_value().bool_value
        self.declare_parameter("left_zone_threshold", 300.0) # Batas kiri zona aman
        self.declare_parameter("right_zone_threshold", 380.0)   
        self.declare_parameter("forward_zone_threshold", 1.4)   
        self.declare_parameter("backward_zone_threshold", 1.85)   
        self.declare_parameter("backward_velocity", -1.0)  # m/s (mundur)
        self.declare_parameter("forward_velocity", 1.0)   # m/s (maju)
        self.declare_parameter("ramp_linear_speed", 0.5)     # percepatan/perlambatan tiap 100ms
        self.declare_parameter("ramp_angular_speed", 0.2)     # percepatan/perlambatan tiap 100ms
        self.declare_parameter("kp_angular", 0.006)
        self.declare_parameter("max_angular_speed", 1.0)

        self.following_active = self.get_parameter("following_active").get_parameter_value().bool_value
        self.forward_zone_threshold = self.get_parameter("forward_zone_threshold").get_parameter_value().double_value
        self.backward_zone_threshold = self.get_parameter("backward_zone_threshold").get_parameter_value().double_value
        self.left_zone_threshold = self.get_parameter("left_zone_threshold").get_parameter_value().double_value
        self.right_zone_threshold = self.get_parameter("right_zone_threshold").get_parameter_value().double_value
        self.kp_angular = self.get_parameter("kp_angular").get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter("max_angular_speed").get_parameter_value().double_value
        self.backward_velocity = self.get_parameter("backward_velocity").get_parameter_value().double_value
        self.forward_velocity = self.get_parameter("forward_velocity").get_parameter_value().double_value
        self.ramp_linear_speed = self.get_parameter("ramp_linear_speed").get_parameter_value().double_value
        self.ramp_angular_speed = self.get_parameter("ramp_angular_speed").get_parameter_value().double_value


        # ---- callback jika parameter berubah ----
        self.add_on_set_parameters_callback(self.param_cb)     

        # === State ===
        self.filtered_distance = None
        self.filtered_center_x_obj = None
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.last_distance = None
        self.distance_threshold = 2
        self.last_distance_time = self.get_clock().now()
        self.data_timeout = 1.0  # detik (atur sesuai kebutuhan)

        self.create_timer(0.1, self.timer_callback)

        # === Subscriber & Publisher ===
        self.subscription = self.create_subscription(
            PersonInfo,
            '/yolo/person_info',
            self.person_info_calback,
            10
        )
        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel_follow',
            10
        )

        # === Timer ===
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def param_cb(self, params):
        for p in params:
            if p.name == "following_active":
                self.following_active = p.value
                self.get_logger().info(f"following_active ⇒ {self.following_active}")
                # pastikan robot langsung berhenti saat dimatikan
                if not self.following_active:
                    self._publish_zero()
        return SetParametersResult(successful=True)


    def person_info_calback(self, msg: PersonInfo):
        raw_distance = msg.distance_cm

        if raw_distance > 0:
            if self.last_distance is None or abs(raw_distance - self.last_distance) >= self.distance_threshold:
               self.filtered_distance = raw_distance
               self.last_distance = raw_distance
               # self.get_logger().info(f"[Filtered] Distance = {raw_distance} cm")
            self.filtered_center_x_obj = msg.center_x_obj
            self.last_distance_time = self.get_clock().now()

    def timer_callback(self):
        # === OFF mode: pastikan kecepatan 0 ===
        if not self.following_active:
            self._ramp_to(0.0, 0.0)  
            self._publish_twist()    
            return      
        
        now = self.get_clock().now()
        time_since_last = (now - self.last_distance_time).nanoseconds / 1e9  # Konversi ke detik

        # linear velocity
        if self.filtered_distance is None or time_since_last > self.data_timeout:
            self.get_logger().warn("Distance data expired or not received")
            self.filtered_distance = None  # Reset
            self.last_distance = None
            target_linear_velocity = 0.0
        else:
            target_linear_velocity = 0.0
            distance_m = self.filtered_distance / 100

            if distance_m < self.forward_zone_threshold:
                target_linear_velocity = self.forward_velocity
            elif distance_m > self.backward_zone_threshold:
                target_linear_velocity = self.backward_velocity
            else: 
                target_linear_velocity = 0.0

        # angular velocity
        if self.filtered_center_x_obj is None or time_since_last > self.data_timeout:
            self.get_logger().warn("Error x data expired or not received")
            self.filtered_center_x_obj = None  # Reset
            target_angular_velocity = 0.0
        else:
            target_angular_velocity = 0.0
            center_x_objek = self.filtered_center_x_obj

            if center_x_objek > self.right_zone_threshold:
                # Hitung error dari tepi kanan zona
                error = center_x_objek - self.right_zone_threshold
                target_angular_velocity = -self.kp_angular * error # Belok kanan
            
            elif center_x_objek < self.left_zone_threshold:
                # Hitung error dari tepi kiri zona
                error = center_x_objek - self.left_zone_threshold
                target_angular_velocity = -self.kp_angular * error # Belok kiri            

        self._ramp_to(target_linear_velocity, target_angular_velocity)
        self._publish_twist()
        
        # === Publish velocity ===
        twist = Twist()
        twist.linear.x = self.current_linear_velocity
        twist.angular.z = self.current_angular_velocity
        self.publisher_.publish(twist)

        # # Log info
        # if self.filtered_distance is not None:
        #     distance_m = self.filtered_distance / 100
        #     self.get_logger().info(
        #         f"Distance: {distance_m:.2f} m | Target vel: {target_linear_velocity:.2f} | Current vel: {self.current_velocity:.2f}"
        #     )
        # else:
        #     self.get_logger().info(
        #         f"[NO DATA] Target vel: {target_linear_velocity:.2f} | Current vel: {self.current_velocity:.2f}"
            # )

# if self.last_distance is None or abs(distance - self.last_distance) >= self.distance_threshold:
#     self.last_distance = distance
#     distance_msg = Int32()
#     distance_msg.data = int(distance)
#     self.distance_pub.publish(distance_msg)

    def _ramp_to(self, target_linear, target_angular):
        # Ramping untuk linear
        if abs(target_linear - self.current_linear_velocity) < self.ramp_linear_speed:
            self.current_linear_velocity = target_linear
        elif target_linear > self.current_linear_velocity:
            self.current_linear_velocity += self.ramp_linear_speed
        else:
            self.current_linear_velocity -= self.ramp_linear_speed

        # Ramping untuk angular
        if abs(target_angular - self.current_angular_velocity) < self.ramp_angular_speed:
            self.current_angular_velocity = target_angular
        elif target_angular > self.current_angular_velocity:
            self.current_angular_velocity += self.ramp_angular_speed
        else:
            self.current_angular_velocity -= self.ramp_angular_speed
        
        # Batasi kecepatan setelah ramping
        self.current_linear_velocity = max(min(self.current_linear_velocity, self.forward_velocity), self.backward_velocity)
        self.current_angular_velocity = max(min(self.current_angular_velocity, self.max_angular_speed), -self.max_angular_speed)

    def _publish_twist(self):
        twist = Twist()
        twist.linear.x = self.current_linear_velocity
        twist.angular.z = self.current_angular_velocity # <-- PERBAIKAN KRITIS
        self.publisher_.publish(twist)

    def _publish_zero(self):
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self._publish_twist()

def main(args=None):
    rclpy.init(args=args)
    node = FollowPerson()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()