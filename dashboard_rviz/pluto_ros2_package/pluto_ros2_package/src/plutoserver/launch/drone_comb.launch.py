from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.launch_description import LaunchDescription


def generate_launch_description():

    ld = LaunchDescription()

    pub_node = Node(package="plutoserver",
                    executable="keyCommand",
                    name="key_command",)

    sub_node = Node(package="plutoserver",
                    executable="keyHandling",
                    name="key_handling",)

    ld.add_action(pub_node)
    ld.add_action(sub_node)

    return ld