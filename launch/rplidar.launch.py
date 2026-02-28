from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Declare launch arguments so they can be overridden from CLI
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the RPLidar (e.g. /dev/ttyUSB0)'
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='460800',
        description='Baud rate for the RPLidar serial connection'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='TF frame id for the lidar (must match URDF link name)'
    )

    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Standard',
        description='Scan mode of the RPLidar (Standard, Express, Boost, etc.)'
    )

    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='scan',
        description='Topic name for the LaserScan output'
    )

    # RPLidar C1 node (using sllidar_ros2 for C1 support)
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'angle_compensate': True,
            'scan_mode': LaunchConfiguration('scan_mode'),
        }],
        remappings=[
            ('scan', LaunchConfiguration('topic_name')),
        ],
    )

    return LaunchDescription([
        serial_port_arg,
        serial_baudrate_arg,
        frame_id_arg,
        scan_mode_arg,
        topic_name_arg,
        lidar_node,
    ])
