#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
from datetime import datetime
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Image, CameraInfo, PointCloud
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
import math

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
import numpy as np

# --- 1. Mapping Topik dan Tipe Pesan ROS2 ---
# Tambahkan topik lain yang ingin Anda log ke CSV
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
    # "/joy": Joy, # Contoh untuk joystick, jika Anda menggunakannya
}

# --- Fungsi Helper ---
def quaternion_to_yaw_deg(ori):
    # Mengonversi quaternion ke yaw (derajat)
    x = ori.x
    y = ori.y
    z = ori.z
    w = ori.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    yaw_deg = math.degrees(yaw)
    return yaw_deg

# --- 2. Variabel Global untuk Data Grafik Real-time ---
# Menggunakan deque untuk efisiensi memori (hanya menyimpan N poin terakhir)
MAX_PLOT_POINTS = 250 # Jumlah poin data maksimum yang ditampilkan di grafik
time_plot_data = deque(maxlen=MAX_PLOT_POINTS)
linear_vel_plot_data = deque(maxlen=MAX_PLOT_POINTS)
angular_vel_plot_data = deque(maxlen=MAX_PLOT_POINTS)

# --- 3. Kelas Node ROS2: TopicLoggerAndPlotter ---
class TopicLoggerAndPlotter(Node):
    def __init__(self, topic_name, msg_type, enable_plot=False):
        super().__init__('topic_logger_and_plotter_node')
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.enable_plot = enable_plot # Flag untuk mengaktifkan/menonaktifkan plotting

        # Inisialisasi untuk CSV logging
        output_dir = "/home/ainunnajibk/superbot_ws/src/superbot_core/dataout/csv"
        os.makedirs(output_dir, exist_ok=True)
        base_filename = topic_name.strip('/').replace('/', '_')
        existing_files = [f for f in os.listdir(output_dir)
                          if f.startswith(base_filename) and f.endswith('.csv')]
        existing_nums = []
        for f in existing_files:
            try:
                num_part = f.replace(base_filename + '_', '').replace('.csv', '')
                if num_part.isdigit():
                    existing_nums.append(int(num_part))
            except ValueError:
                continue
        next_num = max(existing_nums, default=0) + 1
        self.csv_path = os.path.join(output_dir, f"{base_filename}_{next_num}.csv")
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.write_csv_header = True # Flag untuk menulis header CSV hanya sekali
        self.get_logger().info(f'Subscribed to {topic_name}, saving to {self.csv_path}')

        # Inisialisasi untuk plotting (jika diaktifkan)
        if self.enable_plot:
            self.start_time = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(f"[INFO] Real-time plotting enabled for {topic_name}.")

        # Membuat subscriber untuk topik yang dipilih
        self.subscription = self.create_subscription(
            msg_type,
            topic_name,
            self.listener_callback,
            10 # QoS history depth
        )
        self.counter = 0 # Counter untuk mengontrol frekuensi logging/plotting

    def listener_callback(self, msg):
        # Proses pesan hanya setiap 10 kali (untuk mengurangi beban CPU dan ukuran file)
        self.counter += 1
        if self.counter % 10 != 0:
            return

        current_time_stamp = self.get_time_from_msg(msg)

        # --- Logging ke CSV ---
        self._log_to_csv(msg, current_time_stamp)

        # --- Plotting Real-time (jika diaktifkan dan tipe pesan sesuai) ---
        if self.enable_plot and (isinstance(msg, Odometry) or isinstance(msg, Twist)):
            self._update_plot_data(msg, current_time_stamp)

    def _log_to_csv(self, msg, time):
        """Mencatat data pesan ke file CSV."""
        row = []
        header = []

        if isinstance(msg, Odometry):
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            lin = msg.twist.twist.linear
            ang = msg.twist.twist.angular
            yaw_deg = quaternion_to_yaw_deg(ori)
            jarak_cm = round((pos.x**2 + pos.y**2)**0.5 * 100, 2)
            
            row = [
                time, pos.x, pos.y, pos.z,
                ori.x, ori.y, ori.z, ori.w,
                lin.x, lin.y, lin.z,
                ang.x, ang.y, ang.z,
                jarak_cm, yaw_deg
            ]
            header = [
                "time", "pos_x", "pos_y", "pos_z",
                "ori_x", "ori_y", "ori_z", "ori_w",
                "linear_x", "linear_y", "linear_z",
                "angular_x", "angular_y", "angular_z",
                "jarak_cm", "heading_deg"
            ]
            self.get_logger().info(f"[CSV Log - Odometry] time: {time:.3f}, Linear X: {lin.x:.3f}, Angular Z: {ang.z:.3f}")

        elif isinstance(msg, Twist):
            lin = msg.linear
            ang = msg.angular
            row = [time, lin.x, lin.y, lin.z, ang.x, ang.y, ang.z]
            header = ["time", "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"]
            self.get_logger().info(f"[CSV Log - Twist] time: {time:.3f}, Linear X: {lin.x:.3f}, Angular Z: {ang.z:.3f}")

        elif isinstance(msg, LaserScan):
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            num_readings = len(msg.ranges)

            def get_range_at_degree(deg):
                angle_rad = deg * (math.pi / 180.0)
                index = int((angle_rad - angle_min) / angle_increment)
                index = max(0, min(index, num_readings - 1))
                return msg.ranges[index]

            r_0   = get_range_at_degree(0)
            r_90  = get_range_at_degree(90)
            r_180 = get_range_at_degree(180)
            r_270 = get_range_at_degree(270)

            row = [time, r_0, r_90, r_180, r_270]
            header = ["time", "range_0", "range_90", "range_180", "range_270"]
            self.get_logger().info(f"[CSV Log - LaserScan] time: {time:.3f}, Range 0: {r_0:.2f} m")

        elif isinstance(msg, PoseStamped):
            pos = msg.pose.position
            ori = msg.pose.orientation
            row = [time, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]
            header = ["time", "pos_x", "pos_y", "pos_z", "ori_x", "ori_y", "ori_z", "ori_w"]
            self.get_logger().info(f"[CSV Log - PoseStamped] time: {time:.3f}, Pos X: {pos.x:.2f}, Y: {pos.y:.2f}")

        elif isinstance(msg, PointStamped):
            pt = msg.point
            row = [time, pt.x, pt.y, pt.z]
            header = ["time", "x", "y", "z"]
            self.get_logger().info(f"[CSV Log - PointStamped] time: {time:.3f}, X: {pt.x:.2f}, Y: {pt.y:.2f}")
        
        else: # Untuk tipe pesan lain yang tidak spesifik di atas
            row = [time, str(msg)]
            header = ["time", "data"]
            self.get_logger().info(f"[CSV Log - {type(msg).__name__}] time: {time:.3f}")

        if self.write_csv_header:
            self.csv_writer.writerow(header)
            self.write_csv_header = False
        
        self.csv_writer.writerow(row)
        self.csv_file.flush() # Pastikan data segera ditulis ke disk

    def _update_plot_data(self, msg, time):
        """Memperbarui data untuk plotting real-time."""
        elapsed_since_start = time - self.start_time

        linear_x = 0.0
        angular_z = 0.0

        if isinstance(msg, Odometry):
            linear_x = msg.twist.twist.linear.x
            angular_z = msg.twist.twist.angular.z
        elif isinstance(msg, Twist):
            linear_x = msg.linear.x
            angular_z = msg.angular.z
        
        # Tambahkan data ke deque global
        time_plot_data.append(elapsed_since_start)
        linear_vel_plot_data.append(linear_x)
        angular_vel_plot_data.append(angular_z)

    def get_time_from_msg(self, msg):
        """Mengekstrak timestamp dari pesan atau menggunakan waktu sistem."""
        try:
            return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except AttributeError:
            return self.get_clock().now().nanoseconds * 1e-9

# --- 4. Setup Grafik Matplotlib ---
# Inisialisasi plot dengan styling yang bagus
plt.style.use('default')
fig, ax = plt.subplots(figsize=(12, 8))

# Garis untuk kecepatan linear dan angular
line_linear, = ax.plot([], [], 'b-', linewidth=2, label='Kecepatan Linear Aktual (m/s)', alpha=0.8)
line_angular, = ax.plot([], [], 'g-', linewidth=2, label='Kecepatan Angular Aktual (rad/s)', alpha=0.8)

# Garis untuk setpoint (bisa diubah secara dinamis jika setpoint diketahui)
setpoint_linear_line = ax.axhline(y=0, color='orange', linestyle='--', alpha=0.7, label='Setpoint Linear')
setpoint_angular_line = ax.axhline(y=0, color='red', linestyle='--', alpha=0.7, label='Setpoint Angular')

def init_plot():
    """Fungsi inisialisasi untuk animasi Matplotlib."""
    ax.set_title("Respon Kecepatan Robot Real-time", fontsize=16, fontweight='bold', pad=20)
    ax.set_xlabel("Waktu (s)", fontsize=12)
    ax.set_ylabel("Kecepatan", fontsize=12)
    
    ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax.set_facecolor('#f8f9fa')
    
    ax.legend(loc='upper right', frameon=True, fancybox=True, shadow=True, 
              framealpha=0.9, fontsize=10)
    
    ax.tick_params(axis='both', which='major', labelsize=10)
    
    ax.set_ylim(-0.5, 0.5) # Batas Y awal
    ax.set_xlim(0, 10) # Batas X awal

    return line_linear, line_angular, setpoint_linear_line, setpoint_angular_line

def update_plot(frame):
    """Fungsi pembaruan untuk animasi Matplotlib."""
    # Pastikan ada cukup data untuk di-plot
    if not time_plot_data or len(time_plot_data) < 2:
        return line_linear, line_angular, setpoint_linear_line, setpoint_angular_line

    # Update data pada garis plot
    line_linear.set_data(time_plot_data, linear_vel_plot_data)
    line_angular.set_data(time_plot_data, angular_vel_plot_data)

    # Auto-scale sumbu X
    time_min, time_max = min(time_plot_data), max(time_plot_data)
    ax.set_xlim(time_min - 0.1, time_max + 0.1)

    # Auto-scale sumbu Y dengan margin
    all_values = list(linear_vel_plot_data) + list(angular_vel_plot_data)
    if all_values:
        y_min = min(all_values) - 0.15 * (max(all_values) - min(all_values) + 0.1)
        y_max = max(all_values) + 0.15 * (max(all_values) - min(all_values) + 0.1)
        if abs(y_max - y_min) < 0.1: # Hindari skala terlalu kecil
            y_min = -0.5
            y_max = 0.5
        ax.set_ylim(y_min, y_max)
    
    # Update setpoint lines (sesuaikan ini jika Anda ingin menampilkan setpoint dinamis)
    # Misalnya, jika Anda sedang mengirim perintah /cmd_vel, setpoint adalah nilai cmd_vel terakhir
    # Anda perlu menyimpan setpoint_cmd_vel terakhir di node atau global variable.
    # Contoh hardcoded untuk visualisasi:
    # setpoint_linear_line.set_ydata([0.2, 0.2]) # Jika target kecepatan linear 0.2 m/s
    # setpoint_angular_line.set_ydata([0.0, 0.0]) # Jika target kecepatan angular 0.0 rad/s

    return line_linear, line_angular, setpoint_linear_line, setpoint_angular_line

def ros_spin_thread(node):
    """Fungsi untuk menjalankan rclpy.spin() di thread terpisah."""
    rclpy.spin(node)

# --- 5. Fungsi Main ---
def main(args=None):
    rclpy.init(args=args)
    
    print("=== ROS2 Logger dan Real-time Plotter ===")
    print("Pilih topik yang ingin Anda pantau:")
    # Tampilkan topik yang didukung
    for topic, msg_type in TOPIC_TYPE_DICT.items():
        print(f"  - {topic} (tipe: {msg_type.__name__})")
    
    topic_name = input("\nMasukkan nama topik ROS2: ").strip()

    msg_type = TOPIC_TYPE_DICT.get(topic_name)
    if not msg_type:
        print(f"[ERROR] Topik '{topic_name}' tidak dikenali. Mohon periksa kembali atau tambahkan ke TOPIC_TYPE_DICT.")
        rclpy.shutdown()
        return

    enable_plot = False
    if topic_name in ["/odom", "/cmd_vel"]:
        choice = input(f"Topik '{topic_name}' dapat di-plot. Apakah Anda ingin menampilkan grafik real-time? (y/n): ").strip().lower()
        if choice == 'y':
            enable_plot = True

    node = TopicLoggerAndPlotter(topic_name, msg_type, enable_plot)

    # Jalankan ROS spin di thread terpisah agar UI Matplotlib tidak blokir
    ros_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    ros_thread.start()

    print(f"[INFO] Mulai mencatat data topik: {topic_name} ke {node.csv_path}")
    if enable_plot:
        print("[INFO] Grafik kecepatan real-time akan muncul.")
    print("[INFO] Tekan Ctrl+C di terminal untuk menghentikan program.")

    try:
        if enable_plot:
            # Inisialisasi dan jalankan animasi Matplotlib
            ani = animation.FuncAnimation(fig, update_plot, init_func=init_plot, 
                                        interval=100, # Perbarui setiap 100 ms (10 Hz)
                                        blit=True, # Optimalisasi rendering
                                        cache_frame_data=False)
            plt.tight_layout()
            plt.show() # Tampilkan jendela plot
        else:
            # Jika plotting tidak diaktifkan, biarkan node ROS berjalan sampai diinterupsi
            # Ini akan menjaga thread ROS tetap hidup dan terus mencatat data
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1) # Spin sebentar agar thread ROS tetap aktif

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt: Program dihentikan oleh pengguna.")
    finally:
        # Tutup file CSV dan hancurkan node ROS
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()
        #/ Pastikan thread ROS juga berhenti
        if ros_thread.is_alive():
            ros_thread.join(timeout=1.0) # Beri waktu thread untuk selesai

if __name__ == '__main__':
    main()