from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="k1_low_level_relay",
                executable="k1_low_level_relay_node",
                name="k1_low_level_relay",
                output="screen",
                parameters=[{"relay_namespace": "/k1"}],
            )
        ]
    )
