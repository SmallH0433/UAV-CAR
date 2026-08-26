from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("air_ground_landing_ros2"))
    default_parameters = str(share / "config" / "adapters.yaml")
    default_landing_config = str(share / "config" / "moving_landing.prototype.json")
    parameters = LaunchConfiguration("parameters_file")
    landing_config = LaunchConfiguration("landing_config_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "parameters_file",
                default_value=default_parameters,
                description="ROS 2 adapter parameter profile",
            ),
            DeclareLaunchArgument(
                "landing_config_file",
                default_value=default_landing_config,
                description="Moving landing vision/quality configuration",
            ),
            Node(
                package="air_ground_landing_ros2",
                executable="elastic_trajectory_adapter",
                name="elastic_trajectory_adapter",
                output="screen",
                parameters=[parameters],
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
                executable="simple_landing_coordinator",
                name="simple_landing_coordinator",
                output="screen",
                parameters=[parameters],
            ),
            Node(
                package="air_ground_landing_ros2",
                executable="guided_executor",
                name="guided_executor",
                output="screen",
                parameters=[parameters],
            ),
        ]
    )
