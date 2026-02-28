"""
slam_hardware.launch.py — SLAM on Real Hardware

Launches SLAM Toolbox configured for real robot:
  - use_sim_time: false
  - Uses /scan from RPLidar C1
  - Uses /odom from diff_drive_node

Prerequisites:
  ros2 launch Tomas_bot bringup_hardware.launch.py

Usage (on LattePanda Alpha):
  ros2 launch Tomas_bot slam_hardware.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pkg_path = get_package_share_directory('Tomas_bot')

    slam_params_file = os.path.join(
        pkg_path, 'config', 'mapper_params_online_async_hardware.yaml'
    )

    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
        namespace='',
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    # ========================== RVIZ2 ==========================

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 with SLAM visualization',
    )

    rviz_config = os.path.join(pkg_path, 'config', 'slam_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (false for real hardware)',
        ),
        use_rviz_arg,
        slam_toolbox,
        activate_event,
        configure_event,
        rviz_node,
    ])
