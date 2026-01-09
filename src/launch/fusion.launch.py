import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchContext


def generate_launch_description() -> LaunchDescription:
    config_file = PathJoinSubstitution([
        FindPackageShare("fusion"),
        "config",
        "params.yaml",
    ])

    mounted_config_file = PathJoinSubstitution([
        FindPackageShare("fusion"),
        "config",
        "mnt",
        "params.yaml",
    ])

    context = LaunchContext()
    mounted_path = mounted_config_file.perform(context)
    config_to_use = mounted_config_file if os.path.exists(mounted_path) else config_file

    return LaunchDescription([
        Node(
            package="fusion",
            executable="fusion_node_exe",
            name="fusion",
            output="screen",
            parameters=[config_to_use],
            # prefix=['valgrind --tool=callgrind']
        ),
    ])
