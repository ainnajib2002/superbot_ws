import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

def generate_launch_description():
    core_package = 'superbot_core'
    hardware_package = 'superbot_hardware'

    # Include rsp.launch.py dari superbot_core
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(core_package), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # Include joystick.launch.py dari superbot_core
    # joystick = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory(core_package), 'launch', 'joystick.launch.py')
    #     ]),
    #     launch_arguments={'use_sim_time': 'false'}.items()
    # )

    # Include rplidar.launch.py dari superbot_core
    # rplidar = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory(core_package), 'launch', 'rplidar.launch.py')
    #     ]),
    #     launch_arguments={'use_sim_time': 'false'}.items()
    # )

    # Include slam.py dari superbot_core
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(core_package), 'launch', 'slam.py')
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Gunakan twist_mux.yaml dari superbot_core
    twist_mux_params = os.path.join(get_package_share_directory(core_package), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_in', '/cmd_vel'), ('/cmd_vel_out', '/diffbot_base_controller/cmd_vel_unstamped')
        ]
    )

    # Gunakan xacro dari superbot_hardware
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(hardware_package), "urdf", "superbot.urdf.xacro"])
    ])
    robot_description = {"robot_description": robot_description_content}

    # Gunakan konfigurasi controller dari superbot_hardware
    robot_controllers = PathJoinSubstitution([
        FindPackageShare(hardware_package), "config", "superbot_controllers.yaml"
    ])

    # Gunakan konfigurasi RViz dari superbot_hardware
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(core_package), "config", "main.rviz"
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # # Node controller manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both"
    )

    # Node robot_state_publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel")]
    )

    # Spawner joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )

    # Spawner diffbot_base_controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"]
    )

    # # Delay RViz dan robot controller setelah joint_state_broadcaster
    # delay_rviz = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node]
    #     )
    # )

    # delay_robot_controller = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[robot_controller_spawner]
    #     )
    # )

    # Node controller manager dengan delay 3 detik
    # delayed_control_node = TimerAction(
    #     period=8.0,  # delay 3 detik, ubah sesuai kebutuhan
    #     actions=[
    #         Node(
    #             package="controller_manager",
    #             executable="ros2_control_node",
    #             parameters=[robot_description, robot_controllers],
    #             output="both"
    #         )
    #     ]
    # )

    # Return LaunchDescription
    return LaunchDescription([
        # rplidar,
        rsp,
        slam,
        # joystick,
        twist_mux,
        control_node,
        # delayed_control_node,  # delay agar tidak terlalu cepat
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        rviz_node,
        robot_controller_spawner
    ])
