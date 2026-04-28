from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    mono_config = LaunchConfiguration("mono_config")
    stereo_config = LaunchConfiguration("stereo_config")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    bringup_share = FindPackageShare("camera_bringup_ros2")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mono_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "mono_stable.yaml"]),
                description="Stable mono camera config file.",
            ),
            DeclareLaunchArgument(
                "stereo_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "stereo_stable.yaml"]),
                description="Stable stereo camera config file.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Whether to start rviz2 with the default stable camera layout.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "rviz", "dual_usb_cameras_stable.rviz"]
                ),
                description="RViz config file.",
            ),
            Node(
                package="mono_camera_cpp_ros2",
                executable="mono_camera_node",
                name="mono_camera_publisher",
                output="screen",
                parameters=[{"config_path": mono_config}],
            ),
            Node(
                package="bi_camera_cpp_ros2",
                executable="stereo_camera_node",
                name="bi_camera_publisher",
                output="screen",
                parameters=[{"config_path": stereo_config}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(start_rviz),
            ),
        ]
    )
