#!/usr/bin/env python3
"""Combined launch file – starts the Pluto Dashboard and optional nodes.

Usage (WiFi, default):
    ros2 launch pluto_dashboard dashboard_launch.py

With PlutoCam over WiFi:
    ros2 launch pluto_dashboard dashboard_launch.py use_camera:=true cam_ip:=192.168.0.1

With CrazyRadio PA (USB dongle) instead of WiFi:
    ros2 launch pluto_dashboard dashboard_launch.py use_radio:=true

With CrazyRadio + camera:
    ros2 launch pluto_dashboard dashboard_launch.py use_radio:=true use_camera:=true

CrazyRadio channel / address override:
    ros2 launch pluto_dashboard dashboard_launch.py use_radio:=true radio_channel:=80 radio_address:=E7E7E7E7E7
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Arguments ────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('use_camera',    default_value='false',
                              description='Launch PlutoCam publisher node'),
        DeclareLaunchArgument('use_radio',     default_value='false',
                              description='Launch CrazyRadio PA bridge instead of WiFi plutonode'),
        DeclareLaunchArgument('cam_ip',        default_value='192.168.0.1',
                              description='PlutoCam IP (WIFI-1080p-* network)'),
        DeclareLaunchArgument('radio_channel', default_value='80',
                              description='CrazyRadio channel (0-125)'),
        DeclareLaunchArgument('radio_address', default_value='E7E7E7E7E7',
                              description='Crazyflie radio address (hex)'),
        DeclareLaunchArgument('radio_index',   default_value='0',
                              description='CrazyRadio USB dongle index'),
        DeclareLaunchArgument('use_pid',       default_value='false',
                              description='Launch PID altitude hold node'),
                              description='CrazyRadio USB dongle index'),
    ]

    def _nodes(context, *a, **kw):
        use_camera = LaunchConfiguration('use_camera').perform(context).lower() == 'true'
        use_radio  = LaunchConfiguration('use_radio').perform(context).lower() == 'true'

        nodes = [
            # Dashboard is always launched.
            Node(
                package='pluto_dashboard',
                executable='dashboard',
                name='pluto_dashboard',
                output='screen',
            ),
        ]

        if use_camera:
            nodes.append(Node(
                package='pluto_camera_sense',
                executable='plutocam_publisher',
                name='pluto_camera_sense',
                output='screen',
                arguments=['--ip', LaunchConfiguration('cam_ip')],
            ))

        use_pid = LaunchConfiguration('use_pid').perform(context).lower() == 'true'

        if use_pid:
            nodes.append(Node(
                package='pluto_dashboard',
                executable='altitude_hold',
                name='pluto_altitude_hold',
                output='screen',
            ))

        if use_radio:
            nodes.append(Node(
                package='pluto_dashboard',
                executable='crazyflie_bridge',
                name='crazyflie_bridge',
                output='screen',
                parameters=[{
                    'radio_channel': int(LaunchConfiguration('radio_channel').perform(context)),
                    'radio_address': LaunchConfiguration('radio_address').perform(context),
                    'radio_index':   int(LaunchConfiguration('radio_index').perform(context)),
                }],
            ))

        return nodes

    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])
