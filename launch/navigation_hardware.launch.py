"""
navigation_hardware.launch.py — Nav2 Autonomous Navigation on Real Hardware

Launches the Nav2 stack for real robot navigation:
  - map_server + AMCL for localization on a saved map
  - Full Nav2: planner, controller, behaviors, bt_navigator
  - use_sim_time: false

Prerequisites:
  1. ros2 launch Tomas_bot bringup_hardware.launch.py
  2. Have a saved map (run SLAM first, then save with:
     ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map)

Usage (on LattePanda Alpha):
  ros2 launch Tomas_bot navigation_hardware.launch.py map:=/path/to/map.yaml

Then on DESKTOP:
  1. Open RViz with nav2_view.rviz
  2. Click "2D Pose Estimate" to set initial position
  3. Click "2D Goal Pose" to send navigation goal
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    pkg_path = get_package_share_directory('Tomas_bot')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock (false for real hardware)',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically start the Nav2 lifecycle nodes',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_path, 'config', 'nav2_params_hardware.yaml'),
        description='Full path to the Nav2 parameters file',
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(os.path.expanduser('~'), 'maps', 'my_map.yaml'),
        description='Full path to the map yaml file to load',
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # =====================================================================
    # Localization (map_server + AMCL)
    # =====================================================================
    localization_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),

            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    configured_params,
                    {'yaml_filename': map_yaml},
                ],
            ),

            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[configured_params],
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[
                    {'autostart': autostart},
                    {'node_names': ['map_server', 'amcl']},
                ],
            ),
        ]
    )

    # =====================================================================
    # Navigation (controller, planner, behaviors, smoother, bt_navigator)
    # Jazzy Nav2 pipeline:
    #   controller_server → /cmd_vel_nav
    #     → velocity_smoother → /cmd_vel_smoothed
    #       → collision_monitor → /cmd_vel
    # =====================================================================
    navigation_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),

            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[configured_params],
                remappings=[('cmd_vel', 'cmd_vel_nav')],
            ),

            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
            ),

            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[configured_params],
            ),

            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
            ),

            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
            ),

            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
            ),

            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[configured_params],
                remappings=[
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel_smoothed'),
                ],
            ),

            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                parameters=[configured_params],
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    {'autostart': autostart},
                    {'node_names': [
                        'controller_server',
                        'planner_server',
                        'smoother_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother',
                        'collision_monitor',
                    ]},
                ],
            ),
        ]
    )

    delayed_navigation = TimerAction(
        period=5.0,
        actions=[navigation_nodes],
    )

    # ========================== RVIZ2 ==========================

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 with navigation visualization',
    )

    rviz_config = os.path.join(pkg_path, 'config', 'nav2_view.rviz')

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
        stdout_linebuf,
        declare_use_sim_time,
        declare_autostart,
        declare_params_file,
        declare_map,
        declare_use_rviz,
        localization_nodes,
        delayed_navigation,
        rviz_node,
    ])
