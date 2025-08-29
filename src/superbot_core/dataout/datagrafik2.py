#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
from datetime import datetime
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan, Image, CameraInfo, PointCloud
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
import math
from rclpy.executors import MultiThreadedExecutor

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
import numpy as np
import time

# Set logging level for rclpy to INFO for more detailed messages during execution
rclpy.logging.set_logger_level('rclpy', rclpy.logging.LoggingSeverity.INFO)
# Suppress matplotlib verbose logs
rclpy.logging.set_logger_level('matplotlib', rclpy.logging.LoggingSeverity.ERROR) 

# --- 1. ROS2 Topic and Message Type Mapping ---
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
    "/loadcell": Float32,
}

# --- Helper Function ---
def quaternion_to_yaw_deg(ori):
    """
    Mengkonversi orientasi quaternion menjadi sudut yaw dalam derajat.
    """
    x = ori.x
    y = ori.y
    z = ori.z
    w = ori.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    yaw_deg = math.degrees(yaw)
    return yaw_deg

# --- 2. Global Variables for Real-time Plotting & Logging Data ---
MAX_PLOT_POINTS = 250 
time_plot_data = deque(maxlen=MAX_PLOT_POINTS)
linear_vel_plot_data = deque(maxlen=MAX_PLOT_POINTS)
angular_vel_plot_data = deque(maxlen=MAX_PLOT_POINTS)
loadcell_plot_data = deque(maxlen=MAX_PLOT_POINTS)

# Global lock for plot/log data access (essential for thread safety)
plot_data_lock = threading.Lock()

# Global variable for consistent plot start time
global_plot_start_time = None

# New: Global dictionary to store the latest data from each plot-enabled topic
# This now also holds ALL data for CSV logging.
# Initialize with np.nan for all fields and include specific keys for plot data
latest_data_buffer = {
    "/odom": {
        'pos_x': np.nan, 'pos_y': np.nan, 'pos_z': np.nan,
        'ori_x': np.nan, 'ori_y': np.nan, 'ori_z': np.nan, 'ori_w': np.nan,
        'linear_x': np.nan, 'linear_y': np.nan, 'linear_z': np.nan,
        'angular_x': np.nan, 'angular_y': np.nan, 'angular_z': np.nan,
        'jarak_cm': np.nan, 'heading_deg': np.nan, 'timestamp': np.nan,
        'plot_linear_x': np.nan, 'plot_angular_z': np.nan # Specific for plot
    },
    "/cmd_vel": {
        'linear_x': np.nan, 'linear_y': np.nan, 'linear_z': np.nan,
        'angular_x': np.nan, 'angular_y': np.nan, 'angular_z': np.nan, 'timestamp': np.nan,
        'plot_linear_x': np.nan, 'plot_angular_z': np.nan # Specific for plot
    },
    "/loadcell": {
        'berat_kg': np.nan, 'timestamp': np.nan,
        'plot_data': np.nan # Specific for plot
    }
}


# --- 3. ROS2 Node Class: TopicSubscriberAndBuffer ---
# This node only subscribes and updates the global latest_data_buffer
class TopicSubscriberAndBuffer(Node):
    def __init__(self, topic_name, msg_type, is_plot_enabled=False):
        super().__init__(f'topic_subscriber_buffer_{topic_name.replace("/", "_").strip("_")}')
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.is_plot_enabled = is_plot_enabled 
        
        self.get_logger().info(f'Subscribed to {topic_name}. Buffering data.')

        self.subscription = self.create_subscription(
            msg_type,
            topic_name,
            self.listener_callback,
            10 # QoS depth
        )

    def listener_callback(self, msg):
        current_ros_time = self.get_time_from_msg(msg)

        with plot_data_lock: # Protect access to the global buffer
            if self.topic_name == "/odom":
                ori = msg.pose.pose.orientation
                yaw_deg = quaternion_to_yaw_deg(ori)
                pos = msg.pose.pose.position
                lin = msg.twist.twist.linear
                ang = msg.twist.twist.angular
                jarak_cm = round((pos.x**2 + pos.y**2)**0.5 * 100, 2)

                latest_data_buffer[self.topic_name].update({
                    'pos_x': pos.x, 'pos_y': pos.y, 'pos_z': pos.z,
                    'ori_x': ori.x, 'ori_y': ori.y, 'ori_z': ori.z, 'ori_w': ori.w,
                    'linear_x': lin.x, 'linear_y': lin.y, 'linear_z': lin.z,
                    'angular_x': ang.x, 'angular_y': ang.y, 'angular_z': ang.z,
                    'jarak_cm': jarak_cm, 'heading_deg': yaw_deg,
                    'timestamp': current_ros_time,
                    'plot_linear_x': lin.x, # For plot
                    'plot_angular_z': ang.z  # For plot
                })

            elif self.topic_name == "/cmd_vel":
                latest_data_buffer[self.topic_name].update({
                    'linear_x': msg.linear.x, 'linear_y': msg.linear.y, 'linear_z': msg.linear.z,
                    'angular_x': msg.angular.x, 'angular_y': msg.angular.y, 'angular_z': msg.angular.z,
                    'timestamp': current_ros_time,
                    'plot_linear_x': msg.linear.x, # For plot
                    'plot_angular_z': msg.angular.z # For plot
                })

            elif self.topic_name == "/loadcell":
                latest_data_buffer[self.topic_name].update({
                    'berat_kg': msg.data,
                    'timestamp': current_ros_time,
                    'plot_data': msg.data # For plot
                })
            
            # rclpy.logging.get_logger(self.get_name()).info(f"[{self.topic_name}] Buffering data. Time: {current_ros_time:.2f}")

    def get_time_from_msg(self, msg):
        try:
            return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except AttributeError:
            return self.get_clock().now().nanoseconds * 1e-9

# New: Central node for CSV logging
class CentralCsvLogger(Node):
    def __init__(self, selected_topics_for_csv):
        super().__init__('central_csv_logger_node')
        self.selected_topics_for_csv = selected_topics_for_csv
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        self.header_written = False

        output_dir = "/home/ainunnajibk/superbot_ws/src/superbot_core/dataout/csv"
        os.makedirs(output_dir, exist_ok=True)
        
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(output_dir, f"merged_log_{timestamp_str}.csv")
        
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        self.get_logger().info(f'Central CSV Logger started, saving to {self.csv_path}')

        # Define the full header based on all possible topics that can be logged
        self.full_csv_header = self._define_full_header()
        self.csv_writer.writerow(self.full_csv_header) # Write header immediately
        self.header_written = True

        # Create a timer to log data periodically (e.g., 10 Hz)
        self.timer = self.create_timer(0.1, self._log_data_callback) # Log every 100ms

    def _define_full_header(self):
        header = ["time_merge_log"] # Use a unique timestamp for the merged log
        
        # Odom related fields
        if "/odom" in self.selected_topics_for_csv:
            header.extend([
                "odom_pos_x", "odom_pos_y", "odom_pos_z",
                "odom_ori_x", "odom_ori_y", "odom_ori_z", "odom_ori_w",
                "odom_linear_x", "odom_linear_y", "odom_linear_z",
                "odom_angular_x", "odom_angular_y", "odom_angular_z",
                "odom_jarak_cm", "odom_heading_deg", "odom_timestamp" # Add topic-specific timestamp
            ])
        
        # Cmd_vel related fields (only if odom not selected or if specifically want cmd_vel)
        if "/cmd_vel" in self.selected_topics_for_csv and "/odom" not in self.selected_topics_for_csv:
             header.extend([
                "cmd_linear_x", "cmd_linear_y", "cmd_linear_z",
                "cmd_angular_x", "cmd_angular_y", "cmd_angular_z", "cmd_timestamp" # Add topic-specific timestamp
            ])
        
        # Loadcell related fields, placed at the end as requested
        if "/loadcell" in self.selected_topics_for_csv:
            header.extend(["loadcell_berat_kg", "loadcell_timestamp"]) # Add topic-specific timestamp

        return header

    def _log_data_callback(self):
        with plot_data_lock: # Protect access to the global buffer
            current_log_time = self.get_clock().now().nanoseconds * 1e-9
            row = [current_log_time]

            # Populate row based on topics selected for CSV and data in buffer
            # Use .get() with default np.nan for fields that might not be available
            
            # Odometry data
            if "/odom" in self.selected_topics_for_csv:
                odom_data = latest_data_buffer.get("/odom", {})
                row.extend([
                    odom_data.get('pos_x', np.nan), odom_data.get('pos_y', np.nan), odom_data.get('pos_z', np.nan),
                    odom_data.get('ori_x', np.nan), odom_data.get('ori_y', np.nan), odom_data.get('ori_z', np.nan), odom_data.get('ori_w', np.nan),
                    odom_data.get('linear_x', np.nan), odom_data.get('linear_y', np.nan), odom_data.get('linear_z', np.nan),
                    odom_data.get('angular_x', np.nan), odom_data.get('angular_y', np.nan), odom_data.get('angular_z', np.nan),
                    odom_data.get('jarak_cm', np.nan), odom_data.get('heading_deg', np.nan),
                    odom_data.get('timestamp', np.nan) # Add odom's last timestamp
                ])
            # Cmd_vel data (only if odom not primarily chosen for data)
            if "/cmd_vel" in self.selected_topics_for_csv and "/odom" not in self.selected_topics_for_csv:
                cmd_vel_data = latest_data_buffer.get("/cmd_vel", {})
                row.extend([
                    cmd_vel_data.get('linear_x', np.nan), cmd_vel_data.get('linear_y', np.nan), cmd_vel_data.get('linear_z', np.nan),
                    cmd_vel_data.get('angular_x', np.nan), cmd_vel_data.get('angular_y', np.nan), cmd_vel_data.get('angular_z', np.nan),
                    cmd_vel_data.get('timestamp', np.nan) # Add cmd_vel's last timestamp
                ])

            # Loadcell data, placed at the end of the row
            if "/loadcell" in self.selected_topics_for_csv:
                loadcell_data = latest_data_buffer.get("/loadcell", {})
                row.extend([
                    loadcell_data.get('berat_kg', np.nan),
                    loadcell_data.get('timestamp', np.nan) # Add loadcell's last timestamp
                ])

            self.csv_writer.writerow(row)
            self.csv_file.flush()
            # self.get_logger().info(f"Logged merged data to CSV (length {len(row)}): {row}")

    def destroy_node(self):
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info(f"CSV file closed for {self.csv_path}.")
        super().destroy_node()


# --- 4. Matplotlib Plot Setup ---
plt.style.use('default')
fig, ax1 = plt.subplots(figsize=(12, 8))

ax1.set_title("Respon Robot Real-time (Kecepatan & Loadcell)", fontsize=16, fontweight='bold', pad=20)
ax1.set_xlabel("Waktu (s)", fontsize=12)
ax1.set_ylabel("Kecepatan (m/s atau rad/s)", fontsize=12, color='blue')
ax1.tick_params(axis='y', labelcolor='blue')

line_linear, = ax1.plot([], [], 'b-', linewidth=2, label='Kecepatan Linear Aktual (m/s)', alpha=0.8)
line_angular, = ax1.plot([], [], 'g-', linewidth=2, label='Kecepatan Angular Aktual (rad/s)', alpha=0.8)

setpoint_linear_line = ax1.axhline(y=0, color='orange', linestyle='--', alpha=0.7, label='Setpoint Linear')
setpoint_angular_line = ax1.axhline(y=0, color='red', linestyle='--', alpha=0.7, label='Setpoint Angular')

ax2 = ax1.twinx()
ax2.set_ylabel("Berat (kg)", fontsize=12, color='purple')
ax2.tick_params(axis='y', labelcolor='purple')

line_loadcell, = ax2.plot([], [], 'm-', linewidth=2, label='Berat Loadcell (kg)', alpha=0.8)

def init_plot():
    ax1.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    ax1.set_facecolor('#f8f9fa')
    
    lines_to_return = [line_linear, line_angular, setpoint_linear_line, setpoint_angular_line, line_loadcell]
    
    h1, l1 = ax1.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax1.legend(h1 + h2, l1 + l2, loc='upper right', frameon=True, fancybox=True, shadow=True, 
               framealpha=0.9, fontsize=10)
    
    ax1.tick_params(axis='both', which='major', labelsize=10)
    ax2.tick_params(axis='y', which='major', labelsize=10)

    ax1.set_ylim(-0.5, 0.5)
    ax2.set_ylim(-10, 10)
    ax1.set_xlim(0, 10)

    return lines_to_return

def update_plot(frame):
    """
    Updates the plot by appending new data from latest_data_buffer to deques.
    """
    global global_plot_start_time

    current_time_for_plot = time.time()
    if global_plot_start_time is None:
        global_plot_start_time = current_time_for_plot # Initialize only once

    elapsed_since_start_plot = current_time_for_plot - global_plot_start_time

    with plot_data_lock: # Protect access to the global buffer
        # Get data from the buffer. Check if data is fresh enough for plotting.
        freshness_threshold_s = 0.5 # Data older than 0.5s will be treated as NaN for plotting

        new_linear_vel = np.nan
        new_angular_vel = np.nan
        new_loadcell = np.nan

        # Process Odometry data for plot
        odom_buffer = latest_data_buffer.get("/odom", {})
        if odom_buffer and 'plot_linear_x' in odom_buffer and \
           (current_time_for_plot - odom_buffer.get('timestamp', -np.inf)) < freshness_threshold_s:
            new_linear_vel = odom_buffer['plot_linear_x']
            new_angular_vel = odom_buffer['plot_angular_z']
        else: # Fallback to cmd_vel if odom data is not fresh/available
            cmd_vel_buffer = latest_data_buffer.get("/cmd_vel", {})
            if cmd_vel_buffer and 'plot_linear_x' in cmd_vel_buffer and \
               (current_time_for_plot - cmd_vel_buffer.get('timestamp', -np.inf)) < freshness_threshold_s:
                new_linear_vel = cmd_vel_buffer['plot_linear_x']
                new_angular_vel = cmd_vel_buffer['plot_angular_z']
        
        # Process Loadcell data for plot
        loadcell_buffer = latest_data_buffer.get("/loadcell", {})
        if loadcell_buffer and 'plot_data' in loadcell_buffer and \
           (current_time_for_plot - loadcell_buffer.get('timestamp', -np.inf)) < freshness_threshold_s:
            new_loadcell = loadcell_buffer['plot_data']

        time_plot_data.append(elapsed_since_start_plot)
        linear_vel_plot_data.append(new_linear_vel)
        angular_vel_plot_data.append(new_angular_vel)
        loadcell_plot_data.append(new_loadcell)

        # Update data on plot lines
        line_linear.set_data(time_plot_data, linear_vel_plot_data)
        line_angular.set_data(time_plot_data, angular_vel_plot_data)
        line_loadcell.set_data(time_plot_data, loadcell_plot_data)

        # Auto-scale X-axis
        if time_plot_data:
            time_min = time_plot_data[0]
            time_max = time_plot_data[-1]
            ax1.set_xlim(time_min - 0.1, time_max + 0.1)
        else:
            ax1.set_xlim(0, 10)

        # Auto-scale Y1-axis (Velocity) - Centered at 0
        valid_vel_values = [v for v in list(linear_vel_plot_data) + list(angular_vel_plot_data) if not np.isnan(v)]
        if valid_vel_values:
            max_val_vel = np.nanmax(valid_vel_values)
            min_val_vel = np.nanmin(valid_vel_values)
            max_abs_vel = max(abs(v) for v in [max_val_vel, min_val_vel] if not np.isnan(v)) if valid_vel_values else 0.0
            
            y1_limit = max(0.1, max_abs_vel * 1.2)
            ax1.set_ylim(-y1_limit, y1_limit)
        else:
            ax1.set_ylim(-0.5, 0.5)
        
        # Auto-scale Y2-axis (Loadcell) - Centered at 0
        valid_loadcell_values = [v for v in loadcell_plot_data if not np.isnan(v)]
        if valid_loadcell_values:
            max_val_lc = np.nanmax(valid_loadcell_values)
            min_val_lc = np.nanmin(valid_loadcell_values)
            max_abs_loadcell = max(abs(v) for v in [max_val_lc, min_val_lc] if not np.isnan(v)) if valid_loadcell_values else 0.0
            y2_limit = max(1.0, max_abs_loadcell * 1.2)
            ax2.set_ylim(-y2_limit, y2_limit)
        else:
            ax2.set_ylim(-10, 10)

    return line_linear, line_angular, setpoint_linear_line, setpoint_angular_line, line_loadcell

def ros_spin_executor(nodes, executor):
    """
    Function target for the ROS2 executor thread.
    """
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    except rclpy.executors.ExecutorShutdownException:
        pass 
    except Exception as e:
        if nodes: 
            nodes[0].get_logger().error(f"Executor thread encountered an error: {e}")
        else:
            print(f"Error in ROS executor thread (no nodes available to log): {e}")

# --- 5. Main Function ---
def main(args=None):
    rclpy.init(args=args) 
    
    print("=== ROS2 Logger dan Real-time Plotter ===")
    print("Pilih topik yang ingin Anda pantau:")
    for topic, msg_type in TOPIC_TYPE_DICT.items():
        print(f"  - {topic} (tipe: {msg_type.__name__})")
    
    topic_input = input("\nMasukkan nama topik ROS2 (contoh: '/odom', '/loadcell', atau '/odom /loadcell' untuk multi-plot): ").strip()

    topic_input = topic_input.replace(",", "")
    selected_topics = topic_input.split()

    plot_topics = ["/odom", "/loadcell", "/cmd_vel"]
    # All selected topics will be logged to CSV in the merged file
    csv_topics = selected_topics 

    enable_plot = any(topic in plot_topics for topic in selected_topics)

    subscriber_nodes = [] 
    
    # Initialize TopicSubscriberAndBuffer for each selected topic
    for topic_name in selected_topics:
        msg_type = TOPIC_TYPE_DICT.get(topic_name)
        if not msg_type:
            print(f"[ERROR] Topik '{topic_name}' tidak dikenali. Mohon periksa kembali atau tambahkan ke TOPIC_TYPE_DICT.")
            rclpy.shutdown()
            return

        plot_this_topic = topic_name in plot_topics
        node = TopicSubscriberAndBuffer(topic_name, msg_type, is_plot_enabled=plot_this_topic)
        subscriber_nodes.append(node)
        print(f"[INFO] Mulai mem-buffer data topik: {topic_name}")
    
    if not subscriber_nodes:
        print("[ERROR] Tidak ada topik valid yang dipilih untuk diproses.")
        rclpy.shutdown()
        return

    # Create the Central CSV Logger node
    central_csv_logger_node = CentralCsvLogger(csv_topics)
    all_nodes_to_spin = subscriber_nodes + [central_csv_logger_node]


    executor = MultiThreadedExecutor()
    ros_executor_thread = threading.Thread(target=ros_spin_executor, args=(all_nodes_to_spin, executor,), daemon=True)
    ros_executor_thread.start()

    if enable_plot:
        print("[INFO] Grafik kecepatan dan berat real-time akan muncul.")
    print("[INFO] Tekan Ctrl+C di terminal untuk menghentikan program.")

    try:
        if enable_plot:
            global global_plot_start_time
            # Initialize global_plot_start_time here, before FuncAnimation starts
            global_plot_start_time = time.time() 

            ani = animation.FuncAnimation(fig, update_plot, init_func=init_plot, 
                                          interval=100, 
                                          blit=True, 
                                          cache_frame_data=False) 
            plt.tight_layout()
            plt.show()
        else:
            while rclpy.ok():
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nKeyboard Interrupt: Program dihentikan oleh pengguna.")
    finally:
        executor.shutdown()
        # Destroy all nodes
        for node in all_nodes_to_spin:
            node.destroy_node()
        rclpy.shutdown()
        
        if ros_executor_thread.is_alive():
            ros_executor_thread.join(timeout=1.0)

if __name__ == '__main__':
    main()