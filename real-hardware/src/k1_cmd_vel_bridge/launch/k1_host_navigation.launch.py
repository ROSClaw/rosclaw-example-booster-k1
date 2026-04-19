from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    odometer_topic = LaunchConfiguration("odometer_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    odometer_bridge = Node(
        package="k1_low_level_relay",
        executable="k1_odometer_bridge_node",
        name="k1_odometer_bridge",
        output="screen",
        parameters=[
            {
                "input_topic": odometer_topic,
                "output_topic": "/odom",
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "publish_tf": True,
            }
        ],
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
            )
        ),
        launch_arguments={
            "slam_params_file": slam_params_file,
        }.items(),
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={
            "params_file": nav2_params_file,
            "use_composition": "False",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("odometer_topic", default_value="/k1/odometer_state"),
            DeclareLaunchArgument("scan_topic", default_value="/k1/scan"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="trunk_link"),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("k1_cmd_vel_bridge"), "config", "k1", "slam_toolbox.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("k1_cmd_vel_bridge"), "config", "k1", "nav2_params.yaml"]
                ),
            ),
            odometer_bridge,
            slam_toolbox,
            nav2_bringup,
        ]
    )
