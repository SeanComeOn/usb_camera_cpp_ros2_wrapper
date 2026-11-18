from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    mono_config = LaunchConfiguration("mono_config")
    stereo_config = LaunchConfiguration("stereo_config")
    wrist_stereo_config = LaunchConfiguration("wrist_stereo_config")
    wrist_stereo_enabled = LaunchConfiguration("wrist_stereo_enabled")
    start_rviz = LaunchConfiguration("start_rviz")
    start_watchdog = LaunchConfiguration("start_watchdog")
    wrist_usb_identity = LaunchConfiguration("wrist_usb_identity")
    stereo_usb_identity = LaunchConfiguration("stereo_usb_identity")
    wrist_usb_device = LaunchConfiguration("wrist_usb_device")
    stereo_usb_device = LaunchConfiguration("stereo_usb_device")
    wrist_stereo_usb_identity = LaunchConfiguration("wrist_stereo_usb_identity")
    wrist_stereo_usb_device = LaunchConfiguration("wrist_stereo_usb_device")
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
                "wrist_stereo_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "stereo_stable.yaml"]),
                description="Optional wrist-mounted stereo camera config file.",
            ),
            DeclareLaunchArgument(
                "wrist_stereo_enabled",
                default_value="false",
                description="Whether to launch the additional wrist-mounted stereo camera.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Whether to start rviz2 with the default stable camera layout.",
            ),
            DeclareLaunchArgument(
                "start_watchdog",
                default_value="true",
                description="Restart a camera node if its USB device exists but image flow stalls.",
            ),
            DeclareLaunchArgument(
                "wrist_usb_identity",
                default_value="1e45:0209",
                description="USB vendor:product identity used by the wrist camera watchdog.",
            ),
            DeclareLaunchArgument(
                "stereo_usb_identity",
                default_value="1bcf:0b15",
                description="USB vendor:product identity used by the stereo camera watchdog.",
            ),
            DeclareLaunchArgument("wrist_usb_device", default_value=""),
            DeclareLaunchArgument("stereo_usb_device", default_value=""),
            DeclareLaunchArgument("wrist_stereo_usb_identity", default_value=""),
            DeclareLaunchArgument("wrist_stereo_usb_device", default_value=""),
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
                parameters=[{"config_path": mono_config, "device_path": wrist_usb_device}],
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="bi_camera_cpp_ros2",
                executable="stereo_camera_node",
                name="third_person_stereo_camera_publisher",
                output="screen",
                parameters=[{"config_path": stereo_config, "device_path": stereo_usb_device}],
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="bi_camera_cpp_ros2",
                executable="stereo_camera_node",
                name="wrist_stereo_camera_publisher",
                output="screen",
                parameters=[
                    {"config_path": wrist_stereo_config, "device_path": wrist_stereo_usb_device}
                ],
                condition=IfCondition(wrist_stereo_enabled),
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="camera_bringup_ros2",
                executable="camera_stream_watchdog.py",
                name="camera_stream_watchdog",
                output="screen",
                parameters=[
                    {
                        "startup_grace_sec": 20.0,
                        "stall_timeout_sec": 5.0,
                        "minimum_fps": 27.0,
                        "low_fps_grace_sec": 10.0,
                        "restart_backoff_sec": 30.0,
                        "terminate_grace_sec": 3.0,
                        "problem_log_interval_sec": 10.0,
                        "wrist_usb_identity": wrist_usb_identity,
                        "stereo_usb_identity": stereo_usb_identity,
                        "wrist_usb_device": wrist_usb_device,
                        "stereo_usb_device": stereo_usb_device,
                        "wrist_stereo_enabled": wrist_stereo_enabled,
                        "wrist_stereo_usb_identity": wrist_stereo_usb_identity,
                        "wrist_stereo_usb_device": wrist_stereo_usb_device,
                    }
                ],
                condition=IfCondition(start_watchdog),
                respawn=True,
                respawn_delay=2.0,
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
