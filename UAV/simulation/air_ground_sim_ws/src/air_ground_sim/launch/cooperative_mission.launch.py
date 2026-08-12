"""One-command launch for the full air-ground cooperative mission."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _default_ardupilot_dir() -> str:
    configured = os.environ.get("ARDUPILOT_DIR")
    if configured:
        return os.path.expanduser(configured)
    candidates = [
        os.path.expanduser("~/ardupilot"),
        os.path.expanduser(
            "~/projects/air_ground_open_source/01_flight_stack/ardupilot"
        ),
    ]
    return next((candidate for candidate in candidates if os.path.isdir(candidate)), candidates[0])


def generate_launch_description():
    share = get_package_share_directory("air_ground_sim")
    deployment_launch = os.path.join(share, "launch", "deployment_sim.launch.py")
    cooperative_parameters = os.path.join(
        share, "config", "cooperative_mission.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("start_sitl", default_value="true"),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("start_web_gateway", default_value="true"),
            DeclareLaunchArgument("software_rendering", default_value="true"),
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument("ardupilot_dir", default_value=_default_ardupilot_dir()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(deployment_launch),
                launch_arguments={
                    "start_gazebo": LaunchConfiguration("start_gazebo"),
                    "headless": LaunchConfiguration("headless"),
                    "start_sitl": LaunchConfiguration("start_sitl"),
                    "start_bridge": "true",
                    "start_nav2": LaunchConfiguration("start_nav2"),
                    "start_uav_interfaces": "true",
                    "start_uav_navigation": "true",
                    "start_mission": "true",
                    "start_rviz": LaunchConfiguration("start_rviz"),
                    "start_web_gateway": LaunchConfiguration("start_web_gateway"),
                    "software_rendering": LaunchConfiguration("software_rendering"),
                    "mission_auto_start": LaunchConfiguration("auto_start"),
                    "ardupilot_dir": LaunchConfiguration("ardupilot_dir"),
                    "world": "air_ground_cooperative_mission.sdf",
                    "map": "cooperative_map.yaml",
                    "override_parameters": cooperative_parameters,
                    "initial_pose_x": "-9.0",
                    "initial_pose_y": "-6.0",
                    "initial_pose_yaw": "0.0",
                    "ugv_adapter": "ackermann",
                }.items(),
            ),
        ]
    )
