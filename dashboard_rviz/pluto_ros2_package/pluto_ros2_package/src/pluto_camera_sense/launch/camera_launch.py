#!/usr/bin/env python3
"""Launch file for pluto_camera_sense node.

Connects to the PlutoCam drone camera over WiFi (default IP: 192.168.0.1),
decodes H.264 video via FFmpeg, and publishes sensor_msgs/Image on
/plutocamera/image_raw at 1080p (default) or 720p (low_def mode).

Requires:
  - ffmpeg installed and on PATH
  - PlutoCam drone camera powered and connected via WiFi

Override the camera IP at launch time:
  ros2 launch pluto_camera_sense camera_launch.py cam_ip:=192.168.0.1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cam_ip_arg = DeclareLaunchArgument(
        'cam_ip',
        default_value='192.168.0.1',
        description='IP address of the PlutoCam drone camera'
    )

    cam_node = Node(
        package='pluto_camera_sense',
        executable='plutocam_publisher',
        name='pluto_camera_sense',
        output='screen',
        arguments=['--ip', LaunchConfiguration('cam_ip')],
    )

    return LaunchDescription([cam_ip_arg, cam_node])
