"""Observation-safe launch for the single flight-action executor path."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("air_ground_landing_ros2"))
    parameters = LaunchConfiguration("parameters_file")
    landing_config = LaunchConfiguration("landing_config_file")
    return LaunchDescription([
        DeclareLaunchArgument(
            "parameters_file",
            default_value=str(share / "config" / "action_executor.yaml"),
            description="Single action executor parameter profile",
        ),
        DeclareLaunchArgument(
            "landing_config_file",
            default_value=str(share / "config" / "moving_landing.prototype.json"),
            description="IBVS vision configuration",
        ),
        ExecuteProcess(
            cmd=["python3", "-m", "air_ground_landing_ros2.ekf_report_filter"],
            output="screen",
        ),
        Node(
            package="air_ground_landing_ros2",
            executable="ibvs_adapter",
            name="ibvs_adapter",
            output="screen",
            parameters=[parameters, {"config_path": landing_config}],
        ),
        Node(
            package="air_ground_landing_ros2",
            executable="landing_target_adapter",
            name="landing_target_adapter",
            output="screen",
            parameters=[parameters, {"config_path": landing_config}],
        ),
        Node(
            package="air_ground_landing_ros2",
            executable="action_executor",
            name="action_executor",
            output="screen",
            parameters=[parameters],
        ),
        Node(
            package="air_ground_landing_ros2",
            executable="flight_status_http",
            name="flight_status_http",
            output="screen",
            parameters=[parameters, {"guided_status_topic": "/landing/action/status"}],
        ),
    ])
