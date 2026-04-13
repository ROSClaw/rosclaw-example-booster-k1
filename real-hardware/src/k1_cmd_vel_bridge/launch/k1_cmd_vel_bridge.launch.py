from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    rpc_service_name = LaunchConfiguration("rpc_service_name")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="Twist topic to bridge into Booster locomotion RPC calls.",
            ),
            DeclareLaunchArgument(
                "rpc_service_name",
                default_value="/booster_rpc_service",
                description="Booster locomotion RPC service name.",
            ),
            Node(
                package="k1_cmd_vel_bridge",
                executable="k1_cmd_vel_bridge_node",
                name="k1_cmd_vel_bridge",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": cmd_vel_topic,
                        "rpc_service_name": rpc_service_name,
                    }
                ],
            )
        ]
    )
