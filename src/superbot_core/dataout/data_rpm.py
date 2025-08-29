#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
from datetime import datetime
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Image, CameraInfo, PointCloud, JointState
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
import math

# Mapping topik dengan tipe pesan ROS2
TOPIC_TYPE_DICT = {
    "/scan": LaserScan,
    "/odom": Odometry,
    "/cmd_vel": Twist,
    "/parameter_events": String,
    "/particle_cloud": PointCloud,
    "/particlecloud": PointCloud,
    "/tf": String,
    "/tf_static": String,
    "/map": OccupancyGrid,
    "/map_updates": OccupancyGrid,
    "/plan": Path,
    "/waypoints": Path,
    "/camera/image_raw": Image,
    "/camera/camera_info": CameraInfo,
    "/clicked_point": PointStamped,
    "/goal_pose": PoseStamped,
    "/initialpose": PoseStamped,
    "/local_plan": Path,
    "/robot_description": String,
    "/rosout": String,
    "/marker": Marker,
    "/mobile_base/sensors/bumper_pointcloud": PointCloud2,
    "/global_costmap/costmap": OccupancyGrid,
    "/global_costmap/costmap_updates": OccupancyGrid,
    "/global_costmap/published_footprint": String,
    "/global_costmap/voxel_marked_cloud": PointCloud2,
    "/local_costmap/costmap": OccupancyGrid,
    "/local_costmap/costmap_updates": OccupancyGrid,
    "/local_costmap/published_footprint": String,
    "/local_costmap/voxel_marked_cloud": PointCloud2,
    "/waypoint_follower/transition_event": String,
    "/joint_states": JointState, # Added JointState topic
    #kurang joystick
}

def quaternion_to_yaw_deg(ori):
    x = ori.x
    y = ori.y
    z = ori.z
    w = ori.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    yaw_deg = math.degrees(yaw)
    return yaw_deg

class TopicLogger(Node):
    def __init__(self, topic_name, msg_type):
        super().__init__('topic_logger_node')
        self.topic_name = topic_name
        self.msg_type = msg_type

        # Buat folder output CSV jika belum ada
        output_dir = "/home/ainunnajibk/superbot_ws/src/superbot_core/dataout/csv"
        os.makedirs(output_dir, exist_ok=True)

        # Penamaan file CSV otomatis berdasarkan topik dan nomor urut
        base_filename = topic_name.strip('/').replace('/', '_')
        existing_files = [f for f in os.listdir(output_dir)
                          if f.startswith(base_filename) and f.endswith('.csv')]

        existing_nums = []
        for f in existing_files:
            try:
                num = int(f.replace(base_filename + '_', '').replace('.csv', ''))
                existing_nums.append(num)
            except ValueError:
                continue
        next_num = max(existing_nums, default=0) + 1
        filename = f"{base_filename}_{next_num}.csv"
        self.csv_path = os.path.join(output_dir, filename)

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.write_header = True

        self.subscription = self.create_subscription(
            msg_type,
            topic_name,
            self.listener_callback,
            10
        )

        self.get_logger().info(f'Subscribed to {topic_name}, saving to {self.csv_path}')
        self.counter = 0

    def listener_callback(self, msg):
        if self.counter % 10 != 0 and not isinstance(msg, JointState): # For JointState, log every message if needed or adjust rate
            self.counter += 1
            return
        self.counter += 1

        time = self.get_time(msg)

        if isinstance(msg, Odometry):
            # Ambil posisi dan orientasi dari pose
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation

            # Ambil kecepatan linear dan angular dari twist
            lin = msg.twist.twist.linear
            ang = msg.twist.twist.angular
            yaw_deg = quaternion_to_yaw_deg(ori)
    
            # Hitung jarak Euclidean dari titik (0,0)
            jarak_cm = round((pos.x**2 + pos.y**2)**0.5 * 100, 2)

            # Susun data sebagai baris
            row = [
                time,
                pos.x, pos.y, pos.z,
                ori.x, ori.y, ori.z, ori.w,
                lin.x, lin.y, lin.z,
                ang.x, ang.y, ang.z,
                jarak_cm,  # Tambahkan jarak ke baris
                yaw_deg  # Tambahkan heading (derajat)
            ]

            # Tulis header hanya sekali
            if self.write_header:
                self.csv_writer.writerow([
                    "time",
                    "pos_x", "pos_y", "pos_z",
                    "ori_x", "ori_y", "ori_z", "ori_w",
                    "linear_x", "linear_y", "linear_z",
                    "angular_x", "angular_y", "angular_z",
                    "jarak_cm",
                    "heading_deg"  # Header untuk heading
                ])
                self.write_header = False

            # Tulis ke CSV
            self.csv_writer.writerow(row)
            self.csv_file.flush()

            # Tampilkan ke terminal
            print(f"[Odometry] time: {time:.3f}")
            print(f"  Position       : x = {pos.x:.5f}, y = {pos.y:.5f}, z = {pos.z:.5f}")
            print(f"  Orientation    : x = {ori.x:.6f}, y = {ori.y:.6f}, z = {ori.z:.6f}, w = {ori.w:.6f}")
            print(f"  Linear Vel     : x = {lin.x:.6f}, y = {lin.y:.6f}, z = {lin.z:.6f}")
            print(f"  Angular Vel    : x = {ang.x:.6f}, y = {ang.y:.6f}, z = {ang.z:.6f}")
            print(f"  Jarak          : {jarak_cm:.2f} cm")
            print(f"  Heading (deg)  : {yaw_deg:.2f}°")
            print(f"  -> Data disimpan ke: {self.csv_path}")

        elif isinstance(msg, JointState):
            left_wheel_velocity_rad_s = 0.0
            right_wheel_velocity_rad_s = 0.0

            try:
                left_wheel_index = msg.name.index('left_wheel_joint')
                right_wheel_index = msg.name.index('right_wheel_joint')

                if left_wheel_index < len(msg.velocity):
                    left_wheel_velocity_rad_s = msg.velocity[left_wheel_index]
                if right_wheel_index < len(msg.velocity):
                    right_wheel_velocity_rad_s = msg.velocity[right_wheel_index]

            except ValueError:
                self.get_logger().warn("Could not find 'left_wheel_joint' or 'right_wheel_joint' in JointState message names.")
                return

            # Convert rad/s to RPM: (rad/s) * (1 revolution / 2*pi radians) * (60 seconds / 1 minute)
            left_wheel_rpm = (left_wheel_velocity_rad_s / (2 * math.pi)) * 60
            right_wheel_rpm = (right_wheel_velocity_rad_s / (2 * math.pi)) * 60

            row = [time, left_wheel_rpm, right_wheel_rpm]

            if self.write_header:
                self.csv_writer.writerow(["time", "left_wheel_rpm", "right_wheel_rpm"])
                self.write_header = False

            self.csv_writer.writerow(row)
            self.csv_file.flush()

            print(f"[JointState] time: {time:.3f}")
            print(f"  Left Wheel RPM: {left_wheel_rpm:.2f}")
            print(f"  Right Wheel RPM: {right_wheel_rpm:.2f}")
            print(f"  -> Data disimpan ke: {self.csv_path}")

            
        elif isinstance(msg, Twist):  # cmd_vel
            lin = msg.linear
            ang = msg.angular
            row = [time, lin.x, lin.y, lin.z, ang.x, ang.y, ang.z]
            if self.write_header:
                self.csv_writer.writerow(["time", "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"])
                self.write_header = False
            print(f"[cmd_vel]\ntime: {time:.2f}\nlinear:\n  x: {lin.x:.2f} y: {lin.y:.2f} z: {lin.z:.2f}\nangular:\n  x: {ang.x:.2f} y: {ang.y:.2f} z: {ang.z:.2f}")

        elif isinstance(msg, LaserScan):
            # Menghitung indeks berdasarkan sudut
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            num_readings = len(msg.ranges)

            def get_range_at_degree(deg):
                angle_rad = deg * 3.1415926535 / 180.0
                index = int((angle_rad - angle_min) / angle_increment)
                index = max(0, min(index, num_readings - 1))  # Batas aman
                return msg.ranges[index]

            r_0   = get_range_at_degree(0)
            r_90  = get_range_at_degree(90)
            r_180 = get_range_at_degree(-180)
            r_270 = get_range_at_degree(-90)  # karena 270° = -90°

            row = [time, r_0, r_90, r_180, r_270]

            if self.write_header:
                header = ["time", "range_0", "range_90", "range_180", "range_270"] # Removed 360 as it's redundant with 0
                self.csv_writer.writerow(header)
                self.write_header = False

            print(f"[LaserScan]\ntime: {time:.2f}")
            print(f"0°: {r_0:.2f} m  |  90°: {r_90:.2f} m  |  180°: {r_180:.2f} m  |  270°: {r_270:.2f} m")

        elif isinstance(msg, PoseStamped):
            pos = msg.pose.position
            ori = msg.pose.orientation
            row = [time, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]
            if self.write_header:
                self.csv_writer.writerow(["time", "pos_x", "pos_y", "pos_z", "ori_x", "ori_y", "ori_z", "ori_w"])
                self.write_header = False
            print(f"[PoseStamped]\ntime: {time:.2f}\nposition:\n  x: {pos.x:.2f} y: {pos.y:.2f} z: {pos.z:.2f}")

        elif isinstance(msg, PointStamped):
            pt = msg.point
            row = [time, pt.x, pt.y, pt.z]
            if self.write_header:
                self.csv_writer.writerow(["time", "x", "y", "z"])
                self.write_header = False
            print(f"[PointStamped]\ntime: {time:.2f}\nx: {pt.x:.2f} y: {pt.y:.2f} z: {pt.z:.2f}")

        else:
            row = [time, str(msg)]
            if self.write_header:
                self.csv_writer.writerow(["time", "data"])
                self.write_header = False
            print(f"[{self.topic_name}] time: {time:.2f}\ndata: {str(msg)[:100]}...")

        if not isinstance(msg, JointState): # Prevent duplicate writes for JointState if it needs different rate
            self.csv_writer.writerow(row)
            self.csv_file.flush()


    def get_time(self, msg):
        try:
            return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except AttributeError: # Some messages might not have a header
            return datetime.now().timestamp()


def main(args=None):
    rclpy.init(args=args)
    topic_name = input("Masukkan nama topik ROS2 (contoh: /scan atau /joint_states): ").strip()

    msg_type = TOPIC_TYPE_DICT.get(topic_name)
    if not msg_type:
        print(f"[ERROR] Topik **{topic_name}** belum dikenali di skrip ini. Tambahkan ke TOPIC_TYPE_DICT.")
        rclpy.shutdown()
        return

    topic_logger = TopicLogger(topic_name, msg_type)
    try:
        rclpy.spin(topic_logger)
    except KeyboardInterrupt:
        print("Keyboard Interrupt: Menyimpan dan keluar.")
    finally:
        topic_logger.csv_file.close()
        topic_logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()