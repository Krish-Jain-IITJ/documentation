#!/usr/bin/env python3
"""
RViz visualization launch for Pluto drone.

Starts:
  1. pluto_tf_broadcaster  — reads IMU + altitude, publishes TF transforms
  2. robot_state_publisher — serves URDF to RViz
  3. rviz2                 — opens with pre-configured pluto_rviz.rviz

Usage:
    ros2 launch pluto_dashboard rviz_launch.py

Prerequisites (must be running in separate terminals):
    ros2 run plutodrone plutonode          # drone WiFi connection
    ros2 launch pluto_dashboard dashboard_launch.py   # optional dashboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('pluto_dashboard')

    urdf_path = os.path.join(pkg, 'urdf', 'pluto.urdf')
    rviz_path = os.path.join(pkg, 'rviz',  'pluto_rviz.rviz')

    # Read URDF for robot_state_publisher
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_path,
            description='Path to RViz config file',
        ),

        # 1. TF broadcaster — attitude + altitude → /tf
        Node(
            package='pluto_dashboard',
            executable='tf_broadcaster',
            name='pluto_tf_broadcaster',
            output='screen',
        ),

        # 2. robot_state_publisher — URDF → /robot_description + joint TFs
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # 3. RViz2 with pre-built config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
