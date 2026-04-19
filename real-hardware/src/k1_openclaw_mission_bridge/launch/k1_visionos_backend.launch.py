from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    openclaw_agent_id = LaunchConfiguration("openclaw_agent_id")
    openclaw_session_id = LaunchConfiguration("openclaw_session_id")
    openclaw_binary = LaunchConfiguration("openclaw_binary")
    openclaw_timeout_seconds = LaunchConfiguration("openclaw_timeout_seconds")
    enable_openclaw = LaunchConfiguration("enable_openclaw")

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8088"),
            DeclareLaunchArgument("openclaw_agent_id", default_value="main"),
            DeclareLaunchArgument("openclaw_session_id", default_value="k1-visionos"),
            DeclareLaunchArgument("openclaw_binary", default_value="openclaw"),
            DeclareLaunchArgument("openclaw_timeout_seconds", default_value="10"),
            DeclareLaunchArgument("enable_openclaw", default_value="true"),
            Node(
                package="k1_visionos_rtabmap_bridge",
                executable="k1_visionos_rtabmap_bridge_node",
                name="k1_visionos_rtabmap_bridge",
                output="screen",
            ),
            Node(
                package="k1_openclaw_mission_bridge",
                executable="k1_openclaw_mission_bridge_server",
                name="k1_openclaw_mission_bridge",
                output="screen",
                parameters=[
                    {
                        "host": host,
                        "port": port,
                        "openclaw_agent_id": openclaw_agent_id,
                        "openclaw_session_id": openclaw_session_id,
                        "openclaw_binary": openclaw_binary,
                        "openclaw_timeout_seconds": openclaw_timeout_seconds,
                        "enable_openclaw": enable_openclaw,
                    }
                ],
            ),
        ]
    )
