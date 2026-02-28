"""
joystick_teleop.launch.py — PS3-Style Gamepad Teleop for Tomas_bot

Launches the joystick driver (joy_node) and custom teleop node to control
the robot with a generic PS3-style 2.4GHz wireless gamepad.

Hardware: Generic PS3-style gamepad with 2.4GHz USB wireless dongle
  - Dual analog sticks for proportional velocity control
  - L1 = enable (deadman), R1 = turbo, Start = E-stop

Prerequisites:
  1. Robot bringup must be running:
     ros2 launch Tomas_bot bringup_hardware.launch.py
  2. USB dongle plugged in (appears as /dev/input/js0)
  3. joy package installed:
     sudo apt install ros-jazzy-joy

Usage:
  ros2 launch Tomas_bot joystick_teleop.launch.py

Optional arguments:
  joy_dev:=/dev/input/js0   Joystick device path
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('Tomas_bot')

    # Config file with all joystick parameters
    joystick_params_file = os.path.join(pkg_path, 'config', 'joystick_params.yaml')

    # ========================== ARGUMENTS ==========================

    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev', default_value='/dev/input/js0',
        description='Joystick device path (check with: ls /dev/input/js*)',
    )

    # ========================== JOY_NODE (Joystick Driver) ==========================
    # Reads raw joystick events from Linux → publishes sensor_msgs/Joy on /joy

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            joystick_params_file,
            {'device_id': 0},  # Override if needed via joy_dev arg
        ],
        # Remap the device name if non-default
        arguments=[],
    )

    # ========================== JOYSTICK TELEOP NODE ==========================
    # Reads /joy → computes proportional velocity → publishes /cmd_vel

    joystick_teleop_node = Node(
        package='Tomas_bot',
        executable='joystick_teleop_node.py',
        name='joystick_teleop_node',
        output='screen',
        parameters=[joystick_params_file],
    )

    return LaunchDescription([
        joy_dev_arg,
        joy_node,
        joystick_teleop_node,
    ])
