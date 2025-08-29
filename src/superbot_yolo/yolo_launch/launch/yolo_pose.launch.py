from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

def generate_launch_description():
    # # Tambahkan camera.launch.py
    # usb_cam_pkg = get_package_share_directory('usb_cam')
    # camera_launch_path = os.path.join(usb_cam_pkg, 'launch', 'camera.launch.py')

    # # Tambahkan camera.launch.py
    # camera_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(camera_launch_path)
    # )

    # QoS parameters untuk mengurangi delay
    qos_params = {
        'qos_overrides': {
            '/camera1/image_raw': {  # atau '/camera/image_raw' tergantung topic
                'subscription': {
                    'reliability': 'best_effort',
                    'history': 'keep_last',
                    'depth': 1
                }
            }
        }
    }

    return LaunchDescription([
        Node(
            package='yolo_main',
            executable='pose_estimation',
            name='pose_estimation_node',
            output='screen',
            parameters=[qos_params]
        ),
        Node(
            package='yolo_main',
            executable='yolo_follow',
            output='screen',
            arguments=['--ros-args', '-r', '__node:=follow_person'],  # override node name
            parameters=[qos_params]
        ),

    # camera_launch,

    ])
    