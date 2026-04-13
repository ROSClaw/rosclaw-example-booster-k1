from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="k1_visionos_rtabmap_bridge",
                executable="k1_visionos_rtabmap_bridge_node",
                name="k1_visionos_rtabmap_bridge",
                output="screen",
            )
        ]
    )
