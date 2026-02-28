"""
bringup_hardware.launch.py — Full Robot Bringup (LattePanda Alpha)

Launches ALL nodes needed for the real robot:
  1. Robot State Publisher (URDF → TF static tree)
  2. RPLidar C1 driver (laser scans → /scan)
  3. Differential Drive serial node (Arduino Leonardo bridge → /odom, /joint_states, cmd_vel)

Hardware: Daniel's robot — LattePanda Alpha + built-in Arduino Leonardo
  Motors: 130 RPM 12V DC with quadrature encoders (11 PPR, ~48:1 gear)
  Wheels: 69mm diameter, 181mm center-to-center separation
  LiDAR:  RPLidar C1 via USB

Usage (on LattePanda Alpha):
  ros2 launch Tomas_bot bringup_hardware.launch.py

Optional arguments:
  lidar_port:=/dev/rplidar      Serial port for RPLidar
  arduino_port:=/dev/arduino    Serial port for Arduino Leonardo
  ticks_per_rev:=528.0          Encoder ticks per wheel revolution (11 PPR × 48:1)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory('Tomas_bot')

    # ========================== ARGUMENTS ==========================

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/rplidar',
        description='Serial port for RPLidar C1',
    )
    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port', default_value='/dev/arduino',
        description='Serial port for Arduino Leonardo (built-in on LattePanda)',
    )
    ticks_per_rev_arg = DeclareLaunchArgument(
        'ticks_per_rev', default_value='528.0',
        description='Encoder ticks per full wheel revolution (11 PPR x 48:1 gear)',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz2 with the robot visualization (default false; SLAM/Nav2 launch their own RViz)',
    )

    # ========================== ROBOT STATE PUBLISHER ==========================

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'rsp_hardware.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    # ========================== RPLIDAR C1 ==========================

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 460800,       # RPLidar C1 default baud rate
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'inverted': False,
        }],
    )

    # ========================== DIFF DRIVE (Arduino Leonardo Bridge) ==========================

    diff_drive_node = Node(
        package='Tomas_bot',
        executable='diff_drive_node.py',
        name='diff_drive_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('arduino_port'),
            'baud_rate': 115200,
            'wheel_separation': 0.181,     # Center-to-center: 181mm
            'wheel_radius': 0.0345,        # 69mm wheels
            'ticks_per_rev': LaunchConfiguration('ticks_per_rev'),
            'max_motor_speed': 0.47,       # 130 RPM × π × 0.069m
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True,
            'publish_rate': 20.0,
        }],
    )

    # ========================== RVIZ2 ==========================

    rviz_config = os.path.join(pkg_path, 'config', 'nav2_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        lidar_port_arg,
        arduino_port_arg,
        ticks_per_rev_arg,
        use_rviz_arg,
        rsp,
        lidar_node,
        diff_drive_node,
        rviz_node,
    ])
