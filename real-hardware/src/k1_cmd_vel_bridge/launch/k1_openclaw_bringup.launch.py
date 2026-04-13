from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    run_rosclaw_packages = LaunchConfiguration("run_rosclaw_packages")
    run_cmd_vel_bridge = LaunchConfiguration("run_cmd_vel_bridge")
    run_autonomy = LaunchConfiguration("run_autonomy")
    platform = LaunchConfiguration("platform")
    rosbridge = LaunchConfiguration("rosbridge")
    perception = LaunchConfiguration("perception")
    discovery_config = LaunchConfiguration("discovery_config")
    safety_config = LaunchConfiguration("safety_config")
    perception_config = LaunchConfiguration("perception_config")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    rpc_service_name = LaunchConfiguration("rpc_service_name")
    autonomy_startup_mode = LaunchConfiguration("autonomy_startup_mode")

    rosclaw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rosclaw_bringup"), "launch", "rosclaw.launch.py"]
            )
        ),
        condition=IfCondition(run_rosclaw_packages),
        launch_arguments={
            "platform": platform,
            "rosbridge": rosbridge,
            "perception": perception,
            "discovery_config": discovery_config,
            "safety_config": safety_config,
            "perception_config": perception_config,
        }.items(),
    )

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("k1_cmd_vel_bridge"), "launch", "k1_cmd_vel_bridge.launch.py"]
            )
        ),
        condition=IfCondition(run_cmd_vel_bridge),
        launch_arguments={
            "cmd_vel_topic": cmd_vel_topic,
            "rpc_service_name": rpc_service_name,
        }.items(),
    )

    autonomy_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="rosclaw_autonomy",
                executable="autonomy_node",
                name="rosclaw_autonomy",
                output="screen",
                parameters=[
                    {
                        "startup_mode": autonomy_startup_mode,
                        "cmd_vel_topic": cmd_vel_topic,
                    }
                ],
                condition=IfCondition(run_autonomy),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "run_rosclaw_packages",
                default_value="true",
                description="Whether to launch rosclaw_bringup alongside the K1 bridge.",
            ),
            DeclareLaunchArgument(
                "run_cmd_vel_bridge",
                default_value="true",
                description="Whether to launch the local cmd_vel bridge node.",
            ),
            DeclareLaunchArgument(
                "run_autonomy",
                default_value="true",
                description="Whether to launch rosclaw_autonomy after rosclaw_bringup.",
            ),
            DeclareLaunchArgument(
                "platform",
                default_value="k1",
                description="ROSClaw platform launch argument.",
            ),
            DeclareLaunchArgument(
                "rosbridge",
                default_value="true",
                description="Whether to enable rosbridge in rosclaw_bringup.",
            ),
            DeclareLaunchArgument(
                "perception",
                default_value="false",
                description="Whether to enable ROSClaw perception nodes.",
            ),
            DeclareLaunchArgument(
                "discovery_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("k1_cmd_vel_bridge"), "config", "k1", "discovery.yaml"]
                ),
                description="Path to the K1 rosclaw discovery config.",
            ),
            DeclareLaunchArgument(
                "safety_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("k1_cmd_vel_bridge"), "config", "k1", "safety_policy.yaml"]
                ),
                description="Path to the K1 rosclaw safety policy config.",
            ),
            DeclareLaunchArgument(
                "perception_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("k1_cmd_vel_bridge"), "config", "k1", "perception.yaml"]
                ),
                description="Path to the K1 rosclaw perception config.",
            ),
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
            DeclareLaunchArgument(
                "autonomy_startup_mode",
                default_value="MANUAL",
                description="rosclaw_autonomy startup mode.",
            ),
            rosclaw_launch,
            bridge_launch,
            autonomy_node,
        ]
    )
