import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():

    pkg_share = get_package_share_directory('drone_sim')
    urdf_file  = os.path.join(pkg_share, 'urdf', 'quadrotor.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'drone_world.sdf')

    # 1. Start Gazebo Harmonic with our world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # 2. Robot state publisher (URDF → TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )

    # 3. Spawn drone in Gazebo at z=0.1
    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'quadrotor',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.1',
        ],
        output='screen'
    )

    # 4. Takeoff node fires 4s after spawn
    takeoff_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='drone_sim',
                executable='simple_takeoff',
                name='simple_takeoff',
                output='screen'
            )
        ]
    )

    # 5. ROS-Gazebo bridge (cmd_vel → gz, camera gz → ROS)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_drone,
        bridge,
        takeoff_node,
    ])
