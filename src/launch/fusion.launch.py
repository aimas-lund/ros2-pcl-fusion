from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = PathJoinSubstitution([
        FindPackageShare("fusion"),
        "config",
        "params.yaml",
    ])

    return LaunchDescription([
        Node(
            package="fusion",
            executable="fusion_node_exe",
            name="fusion",
            output="screen",
            parameters=[config_file],
            # prefix=['valgrind --tool=callgrind']
        ),
    ])
