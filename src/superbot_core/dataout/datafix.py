#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
from datetime import datetime
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan, Image, CameraInfo, PointCloud, JointState
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
import math
from rclpy.executors import MultiThreadedExecutor

# Import TF2 related modules
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped

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
# Note: /tf is handled separately via tf2_ros.Buffer, not direct subscription
TOPIC_TYPE_DICT = {
    "/scan": LaserScan,
    "/odom": Odometry,
    "/cmd_vel": Twist,
    "/parameter_events": String,
    "/particle_cloud": PointCloud,
    "/particlecloud": PointCloud,
    # "/tf": String, # Removed: TF is handled via tf2_ros.Buffer
    "/tf_static": String,
    "/map": OccupancyGrid,
    "/map_updates": OccupancyGrid,
    "/plan": Path, # Added /plan topic
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
    "/joint_states": JointState,
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
    },
    "/joint_states": { # Added for JointState
        'left_wheel_rpm': np.nan, 'right_wheel_rpm': np.nan, 'timestamp': np.nan,
        'plot_left_wheel_rpm': np.nan, 'plot_right_wheel_rpm': np.nan # For potential plotting
    },
    "/plan": { # Added for /plan topic
        'last_pose_x': np.nan, 'last_pose_y': np.nan, 'last_pose_z': np.nan,
        'last_ori_x': np.nan, 'last_ori_y': np.nan, 'last_ori_z': np.nan, 'last_ori_w': np.nan,
        'num_poses': np.nan, 'timestamp': np.nan
    },
    # New: TF data buffer
    "tf_data": {
        'pos_x': np.nan, 'pos_y': np.nan, 'pos_z': np.nan,
        'timestamp': np.nan,
        'source_frame': '', 'target_frame': '' # To indicate which transform was found
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
            
            elif self.topic_name == "/joint_states": # Added JointState handling
                left_wheel_velocity_rad_s = np.nan
                right_wheel_velocity_rad_s = np.nan

                try:
                    left_wheel_index = msg.name.index('left_wheel_joint')
                    right_wheel_index = msg.name.index('right_wheel_joint')

                    if left_wheel_index < len(msg.velocity):
                        left_wheel_velocity_rad_s = msg.velocity[left_wheel_index]
                    if right_wheel_index < len(msg.velocity):
                        right_wheel_velocity_rad_s = msg.velocity[right_wheel_index]

                except ValueError:
                    pass # Suppress warning if not found, just use NaN
                
                left_wheel_rpm = (left_wheel_velocity_rad_s / (2 * math.pi)) * 60 if not np.isnan(left_wheel_velocity_rad_s) else np.nan
                right_wheel_rpm = (right_wheel_velocity_rad_s / (2 * math.pi)) * 60 if not np.isnan(right_wheel_velocity_rad_s) else np.nan

                latest_data_buffer[self.topic_name].update({
                    'left_wheel_rpm': left_wheel_rpm,
                    'right_wheel_rpm': right_wheel_rpm,
                    'timestamp': current_ros_time,
                    'plot_left_wheel_rpm': left_wheel_rpm, # For potential plotting
                    'plot_right_wheel_rpm': right_wheel_rpm # For potential plotting
                })
            
            elif self.topic_name == "/plan": # Added Path handling
                if msg.poses: # Check if there are any poses in the path
                    last_pose = msg.poses[-1].pose
                    latest_data_buffer[self.topic_name].update({
                        'last_pose_x': last_pose.position.x,
                        'last_pose_y': last_pose.position.y,
                        'last_pose_z': last_pose.position.z,
                        'last_ori_x': last_pose.orientation.x,
                        'last_ori_y': last_pose.orientation.y,
                        'last_ori_z': last_pose.orientation.z,
                        'last_ori_w': last_pose.orientation.w,
                        'num_poses': len(msg.poses),
                        'timestamp': current_ros_time
                    })
                else:
                    # If the path is empty, reset data
                    latest_data_buffer[self.topic_name].update({
                        'last_pose_x': np.nan, 'last_pose_y': np.nan, 'last_pose_z': np.nan,
                        'last_ori_x': np.nan, 'last_ori_y': np.nan, 'last_ori_z': np.nan, 'last_ori_w': np.nan,
                        'num_poses': 0, 'timestamp': current_ros_time
                    })

    def get_time_from_msg(self, msg):
        try:
            return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except AttributeError:
            return self.get_clock().now().nanoseconds * 1e-9

# New: Central node for CSV logging and TF lookup
class CentralCsvLogger(Node):
    def __init__(self, selected_topics_for_csv, enable_tf_logging=False):
        super().__init__('central_csv_logger_node')
        self.selected_topics_for_csv = selected_topics_for_csv
        self.enable_tf_logging = enable_tf_logging
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        self.header_written = False
        self.last_log_time = None 
        self.start_log_time = self.get_clock().now().nanoseconds * 1e-9 # Initialize start time for elapsed_time

        output_dir = "/home/orin/superbot_ws/src/superbot_core/dataout/csv"
        os.makedirs(output_dir, exist_ok=True)
        
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(output_dir, f"merged_log_{timestamp_str}.csv")
        
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        self.get_logger().info(f'Central CSV Logger started, saving to {self.csv_path}')

        # --- TF2 Buffer and Listener Setup ---
        if self.enable_tf_logging:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.target_frame = 'base_link' # Common robot base frame
            self.source_frames = ['map', 'odom'] # Try map first, then odom
            self.get_logger().info(f"TF logging enabled. Will try to get transform from {self.source_frames} to {self.target_frame}.")


        # Define the full header based on all possible topics that can be logged
        self.full_csv_header = self._define_full_header()
        self.csv_writer.writerow(self.full_csv_header) # Write header immediately
        self.header_written = True

        # Create a timer to log data periodically (e.g., 10 Hz)
        self.timer = self.create_timer(0.05, self._log_data_callback) # Changed to 0.05s

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
        
        # Cmd_vel related fields
        if "/cmd_vel" in self.selected_topics_for_csv: 
             header.extend([
                "cmd_linear_x", "cmd_linear_y", "cmd_linear_z",
                "cmd_angular_x", "cmd_angular_y", "cmd_angular_z", "cmd_timestamp" # Add topic-specific timestamp
            ])
        
        # JointState related fields
        if "/joint_states" in self.selected_topics_for_csv:
            header.extend(["joint_left_wheel_rpm", "joint_right_wheel_rpm", "joint_timestamp"])

        # Loadcell related fields
        if "/loadcell" in self.selected_topics_for_csv:
            header.extend(["loadcell_berat_kg", "loadcell_timestamp"]) # Add topic-specific timestamp

        # Plan related fields
        if "/plan" in self.selected_topics_for_csv:
            header.extend([
                "plan_last_pose_x", "plan_last_pose_y", "plan_last_pose_z",
                "plan_last_ori_x", "plan_last_ori_y", "plan_last_ori_z", "plan_last_ori_w",
                "plan_num_poses", "plan_timestamp"
            ])
        
        # New: TF related fields
        if self.enable_tf_logging:
            header.extend(["tf_pos_x", "tf_pos_y", "tf_pos_z", "tf_timestamp", "tf_source_frame", "tf_target_frame"])

        header.append("interval_s") # New column for interval
        header.append("elapsed_time_s") # New column for running time

        return header

    def _log_data_callback(self):
        with plot_data_lock: # Protect access to the global buffer
            current_log_time = self.get_clock().now().nanoseconds * 1e-9
            
            interval = np.nan
            if self.last_log_time is not None:
                interval = current_log_time - self.last_log_time
            self.last_log_time = current_log_time

            elapsed_time = current_log_time - self.start_log_time # Calculate elapsed time

            # --- TF Lookup (if enabled) ---
            if self.enable_tf_logging:
                tf_found = False
                for source_frame in self.source_frames:
                    try:
                        # Lookup the transform from source_frame to target_frame at current time
                        # Use rclpy.time.Time() for the time argument
                        transform = self.tf_buffer.lookup_transform(
                            source_frame,
                            self.target_frame,
                            rclpy.time.Time() # Use current time
                        )
                        # Update global buffer with TF data
                        latest_data_buffer["tf_data"].update({
                            'pos_x': transform.transform.translation.x,
                            'pos_y': transform.transform.translation.y,
                            'pos_z': transform.transform.translation.z,
                            'timestamp': transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9,
                            'source_frame': source_frame,
                            'target_frame': self.target_frame
                        })
                        tf_found = True
                        break # Found a transform, no need to check other source frames
                    except TransformException as ex:
                        # self.get_logger().warn(f"Could not transform {source_frame} to {self.target_frame}: {ex}", throttle_duration_sec=1.0)
                        pass # Suppress frequent warnings if transform is not yet available

                if not tf_found:
                    # If no transform was found, set TF data in buffer to NaN
                    latest_data_buffer["tf_data"].update({
                        'pos_x': np.nan, 'pos_y': np.nan, 'pos_z': np.nan,
                        'timestamp': np.nan, 'source_frame': '', 'target_frame': ''
                    })

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
            
            # Cmd_vel data
            if "/cmd_vel" in self.selected_topics_for_csv:
                cmd_vel_data = latest_data_buffer.get("/cmd_vel", {})
                row.extend([
                    cmd_vel_data.get('linear_x', np.nan), cmd_vel_data.get('linear_y', np.nan), cmd_vel_data.get('linear_z', np.nan),
                    cmd_vel_data.get('angular_x', np.nan), cmd_vel_data.get('angular_y', np.nan), cmd_vel_data.get('angular_z', np.nan),
                    cmd_vel_data.get('timestamp', np.nan) # Add cmd_vel's last timestamp
                ])

            # JointState data
            if "/joint_states" in self.selected_topics_for_csv:
                joint_state_data = latest_data_buffer.get("/joint_states", {})
                row.extend([
                    joint_state_data.get('left_wheel_rpm', np.nan),
                    joint_state_data.get('right_wheel_rpm', np.nan),
                    joint_state_data.get('timestamp', np.nan)
                ])

            # Loadcell data
            if "/loadcell" in self.selected_topics_for_csv:
                loadcell_data = latest_data_buffer.get("/loadcell", {})
                row.extend([
                    loadcell_data.get('berat_kg', np.nan),
                    loadcell_data.get('timestamp', np.nan) # Add loadcell's last timestamp
                ])
            
            # Plan data
            if "/plan" in self.selected_topics_for_csv:
                plan_data = latest_data_buffer.get("/plan", {})
                row.extend([
                    plan_data.get('last_pose_x', np.nan), plan_data.get('last_pose_y', np.nan), plan_data.get('last_pose_z', np.nan),
                    plan_data.get('last_ori_x', np.nan), plan_data.get('last_ori_y', np.nan), plan_data.get('last_ori_z', np.nan), plan_data.get('last_ori_w', np.nan),
                    plan_data.get('num_poses', np.nan), plan_data.get('timestamp', np.nan)
                ])

            # New: TF data
            if self.enable_tf_logging:
                tf_data = latest_data_buffer.get("tf_data", {})
                row.extend([
                    tf_data.get('pos_x', np.nan), tf_data.get('pos_y', np.nan), tf_data.get('pos_z', np.nan),
                    tf_data.get('timestamp', np.nan), tf_data.get('source_frame', ''), tf_data.get('target_frame', '')
                ])

            row.append(interval) # Append the calculated interval
            row.append(elapsed_time) # Append the elapsed time

            self.csv_writer.writerow(row)
            self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info(f"CSV file closed for {self.csv_path}.")
        super().destroy_node()

# New: Node for printing consolidated data to terminal
class TerminalPrinter(Node):
    def __init__(self, selected_topics_for_print, enable_tf_print=False):
        super().__init__('terminal_printer_node')
        self.selected_topics_for_print = selected_topics_for_print
        self.enable_tf_print = enable_tf_print
        self.last_print_time = self.get_clock().now().nanoseconds * 1e-9
        self.print_interval = 0.5 # seconds, print every 0.5 seconds
        self.start_print_time = self.get_clock().now().nanoseconds * 1e-9 # Initial print time

        self.get_logger().info('Terminal Printer started.')
        self.timer = self.create_timer(self.print_interval, self._print_data_callback)

    def _print_data_callback(self):
        current_time = self.get_clock().now().nanoseconds * 1e-9
        
        # Calculate elapsed time for terminal display
        elapsed_time_for_display = current_time - self.start_print_time

        with plot_data_lock:
            output_lines = [f"\n--- Data Log ({elapsed_time_for_display:.1f} s) ---"]

            if "/odom" in self.selected_topics_for_print:
                odom_data = latest_data_buffer.get("/odom", {})
                if not np.isnan(odom_data.get('timestamp', np.nan)):
                    output_lines.append(f"  [Odometry]")
                    output_lines.append(f"    Pos (x,y): ({odom_data.get('pos_x', np.nan):.2f}, {odom_data.get('pos_y', np.nan):.2f})")
                    output_lines.append(f"    Lin Vel: {odom_data.get('linear_x', np.nan):.2f} m/s, Ang Vel: {odom_data.get('angular_z', np.nan):.2f} rad/s")
                    output_lines.append(f"    Jarak: {odom_data.get('jarak_cm', np.nan):.2f} cm, Heading: {odom_data.get('heading_deg', np.nan):.2f}°")

            if self.enable_tf_print:
                tf_data = latest_data_buffer.get("tf_data", {})
                if not np.isnan(tf_data.get('timestamp', np.nan)):
                    output_lines.append(f"  [TF Pose ({tf_data.get('source_frame', '')} -> {tf_data.get('target_frame', '')})]")
                    output_lines.append(f"    Pos (x,y,z): ({tf_data.get('pos_x', np.nan):.2f}, {tf_data.get('pos_y', np.nan):.2f}, {tf_data.get('pos_z', np.nan):.2f})")

            if "/cmd_vel" in self.selected_topics_for_print:
                cmd_vel_data = latest_data_buffer.get("/cmd_vel", {})
                if not np.isnan(cmd_vel_data.get('timestamp', np.nan)):
                    output_lines.append(f"  [Cmd_Vel]")
                    output_lines.append(f"    Linear X: {cmd_vel_data.get('linear_x', np.nan):.2f} m/s, Angular Z: {cmd_vel_data.get('angular_z', np.nan):.2f} rad/s")

            if "/joint_states" in self.selected_topics_for_print:
                joint_state_data = latest_data_buffer.get("/joint_states", {})
                if not np.isnan(joint_state_data.get('timestamp', np.nan)):
                    output_lines.append(f"  [JointState]")
                    output_lines.append(f"    Left Wheel RPM: {joint_state_data.get('left_wheel_rpm', np.nan):.2f}, Right Wheel RPM: {joint_state_data.get('right_wheel_rpm', np.nan):.2f}")

            if "/loadcell" in self.selected_topics_for_print:
                loadcell_data = latest_data_buffer.get("/loadcell", {})
                if not np.isnan(loadcell_data.get('timestamp', np.nan)):
                    output_lines.append(f"  [Loadcell]")
                    output_lines.append(f"    Berat: {loadcell_data.get('berat_kg', np.nan):.4f} kg")
            
            if "/plan" in self.selected_topics_for_print: # Added /plan printout
                plan_data = latest_data_buffer.get("/plan", {})
                if not np.isnan(plan_data.get('timestamp', np.nan)):
                    output_lines.append(f"  [Plan (Path)]")
                    output_lines.append(f"    Last Pose (x,y): ({plan_data.get('last_pose_x', np.nan):.2f}, {plan_data.get('last_pose_y', np.nan):.2f})")
                    output_lines.append(f"    Num Poses: {int(plan_data.get('num_poses', np.nan)) if not np.isnan(plan_data.get('num_poses', np.nan)) else 'N/A'}")


            if len(output_lines) > 1: # Only print if there's actual data to show
                print("\n".join(output_lines))

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

        # Prioritize Odometry data for plot if available and fresh
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
    print("  - /tf (Untuk posisi robot yang dikoreksi, dari map/odom ke base_link)")
    
    topic_input = input("\nMasukkan nama topik ROS2 (contoh: '/odom', '/loadcell', '/joint_states', '/tf', '/plan' atau '/odom /loadcell /joint_states /tf /plan' untuk multi-topic): ").strip()

    topic_input = topic_input.replace(",", "")
    selected_topics = topic_input.split()

    # Determine if TF logging is requested
    enable_tf_logging = False
    if "/tf" in selected_topics:
        enable_tf_logging = True
        selected_topics.remove("/tf") # Remove /tf from direct subscription list as it's handled by CentralCsvLogger

    # Topics that can be plotted on the existing graph
    plot_topics = ["/odom", "/loadcell", "/cmd_vel"]
    
    # All selected topics will be logged to CSV in the merged file
    csv_topics = selected_topics 

    # Topics to be displayed in the terminal in a consolidated manner
    terminal_print_topics = selected_topics # All selected topics will also be printed

    enable_plot = any(topic in plot_topics for topic in selected_topics)

    subscriber_nodes = [] 
    
    # Initialize TopicSubscriberAndBuffer for each selected topic (excluding /tf)
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
    
    if not subscriber_nodes and not enable_tf_logging:
        print("[ERROR] Tidak ada topik valid yang dipilih untuk diproses.")
        rclpy.shutdown()
        return

    # Create the Central CSV Logger node (now also handles TF lookup)
    central_csv_logger_node = CentralCsvLogger(csv_topics, enable_tf_logging=enable_tf_logging)
    # Create the Terminal Printer node
    terminal_printer_node = TerminalPrinter(terminal_print_topics, enable_tf_print=enable_tf_logging)

    all_nodes_to_spin = subscriber_nodes + [central_csv_logger_node, terminal_printer_node]


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
                                          interval=100, # Plot update interval (ms)
                                          blit=True, 
                                          cache_frame_data=False) 
            plt.tight_layout()
            plt.show()
        else:
            # If no plot is enabled, just keep the script running for logging
            while rclpy.ok():
                time.sleep(0.1) # Keep main thread alive
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
