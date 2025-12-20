from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="fusion",
            executable="fusion_node_exe",
            name="fusion",
            output="screen",
        ),
    ])
