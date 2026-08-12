"""Start the common ROS 2 interfaces with either simulation or hardware endpoints."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _make_nodes(context):
    profile = LaunchConfiguration("profile").perform(context)
    if profile not in ("sim", "real"):
        raise RuntimeError("profile must be 'sim' or 'real'")
    share = get_package_share_directory("air_ground_sim")
    parameters = os.path.join(share, "config", f"{profile}_interfaces.yaml")
    override_parameters = LaunchConfiguration("override_parameters").perform(context)
    node_parameters = [parameters]
    if override_parameters:
        node_parameters.append(override_parameters)
    nodes = [
        Node(
            package="air_ground_sim",
            executable="ugv_command_gateway",
            name="ugv_command_gateway",
            output="screen",
            parameters=node_parameters,
        ),
        Node(
            package="air_ground_sim",
            executable="ugv_control_mux",
            name="ugv_control_mux",
            output="screen",
            parameters=node_parameters,
        ),
        Node(
            package="air_ground_sim",
            executable="system_supervisor",
            name="system_supervisor",
            output="screen",
            parameters=node_parameters,
        ),
    ]
    start_uav_interfaces = LaunchConfiguration("start_uav_interfaces").perform(context).lower()
    if start_uav_interfaces in ("1", "true", "yes", "on"):
        nodes.extend(
            [
                Node(
                    package="air_ground_sim",
                    executable="uav_mavlink_bridge",
                    name="uav_mavlink_bridge",
                    output="screen",
                    parameters=node_parameters,
                ),
                Node(
                    package="air_ground_sim",
                    executable="vision_input_monitor",
                    name="vision_input_monitor",
                    output="screen",
                    parameters=node_parameters,
                ),
                Node(
                    package="air_ground_sim",
                    executable="apriltag_tracker",
                    name="apriltag_tracker",
                    output="screen",
                    parameters=node_parameters,
                ),
                Node(
                    package="air_ground_sim",
                    executable="uav_command_mux",
                    name="uav_command_mux",
                    output="screen",
                    parameters=node_parameters,
                ),
                Node(
                    package="air_ground_sim",
                    executable="uav_gimbal_controller",
                    name="uav_gimbal_controller",
                    output="screen",
                    parameters=node_parameters,
                ),
                Node(
                    package="air_ground_sim",
                    executable="uav_perception",
                    name="uav_perception",
                    output="screen",
                    parameters=node_parameters,
                ),
                Node(
                    package="air_ground_sim",
                    executable="uav_docking_controller",
                    name="uav_docking_controller",
                    output="screen",
                    parameters=node_parameters,
                ),
            ]
        )
        if profile == "sim":
            nodes.append(
                Node(
                    package="air_ground_sim",
                    executable="uav_ultrasonic_adapter",
                    name="uav_ultrasonic_adapter",
                    output="screen",
                    parameters=node_parameters,
                )
            )
    start_demo_motion = LaunchConfiguration("start_demo_motion").perform(context).lower()
    if profile == "sim" and start_demo_motion in ("1", "true", "yes", "on"):
        nodes.append(
            Node(
                package="air_ground_sim",
                executable="ugv_demo_motion",
                name="ugv_demo_motion",
                output="screen",
                parameters=node_parameters,
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="sim",
                description="Interface endpoints: sim or real",
            ),
            DeclareLaunchArgument(
                "start_demo_motion",
                default_value="false",
                description="Explicitly opt in to the legacy scripted UGV motion in the simulation profile",
            ),
            DeclareLaunchArgument(
                "start_uav_interfaces",
                default_value="true",
                description="Start MAVLink, vision, AprilTag, and UAV command mux nodes",
            ),
            DeclareLaunchArgument(
                "override_parameters",
                default_value="",
                description="Optional YAML file merged after the selected profile",
            ),
            OpaqueFunction(function=_make_nodes),
        ]
    )
