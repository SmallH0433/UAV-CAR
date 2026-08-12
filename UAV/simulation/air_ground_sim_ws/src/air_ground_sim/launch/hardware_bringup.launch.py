"""Bring up the same navigation/control graph against Jetson hardware drivers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def _static_transform(parent: str, child: str, x: str, y: str, z: str) -> Node:
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{child}_static_transform",
        arguments=[
            "--x",
            x,
            "--y",
            y,
            "--z",
            z,
            "--yaw",
            "0",
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            parent,
            "--child-frame-id",
            child,
        ],
    )


def generate_launch_description():
    share = get_package_share_directory("air_ground_sim")
    nav2_share = get_package_share_directory("nav2_bringup")
    parameters = os.path.join(share, "config", "real_interfaces.yaml")
    default_mission_parameters = os.path.join(
        share, "config", "real_mission.yaml"
    )
    ekf_parameters = os.path.join(share, "config", "ugv_ekf.yaml")
    nav2_source = os.path.join(share, "config", "nav2_ackermann.yaml")
    collision_parameters = os.path.join(
        share, "config", "collision_monitor_real.yaml"
    )
    behavior_tree = os.path.join(
        share, "behavior_trees", "navigate_ackermann_safe.xml"
    )
    through_poses_behavior_tree = os.path.join(
        share, "behavior_trees", "navigate_through_poses_ackermann_safe.xml"
    )

    map_yaml = LaunchConfiguration("map")
    adapter_type = LaunchConfiguration("ugv_adapter")
    start_nav2 = LaunchConfiguration("start_nav2")
    start_hunter_driver = LaunchConfiguration("start_hunter_driver")
    start_uav_interfaces = LaunchConfiguration("start_uav_interfaces")
    start_uav_navigation = LaunchConfiguration("start_uav_navigation")
    start_web_gateway = LaunchConfiguration("start_web_gateway")
    start_mission = LaunchConfiguration("start_mission")
    initial_pose = LaunchConfiguration("set_initial_pose")
    initial_x = LaunchConfiguration("initial_pose_x")
    initial_y = LaunchConfiguration("initial_pose_y")
    initial_yaw = LaunchConfiguration("initial_pose_yaw")
    mission_parameters = LaunchConfiguration("mission_parameters")

    interfaces = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "interfaces.launch.py")),
        launch_arguments={
            "profile": "real",
            "start_demo_motion": "false",
            "start_uav_interfaces": start_uav_interfaces,
            "override_parameters": mission_parameters,
        }.items(),
    )
    chassis_adapter = Node(
        package="air_ground_sim",
        executable="ugv_chassis_adapter",
        name="ugv_chassis_adapter",
        output="screen",
        parameters=[parameters, {"adapter_type": adapter_type}],
    )
    docking_hardware_gateway = Node(
        package="air_ground_sim",
        executable="docking_hardware_gateway",
        name="docking_hardware_gateway",
        output="screen",
        parameters=[parameters, mission_parameters],
    )
    hunter_driver = Node(
        package="hunter_base",
        executable="hunter_base_node",
        name="hunter_base_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "port_name": LaunchConfiguration("can_port"),
                "odom_frame": "odom",
                "base_frame": "base_link",
                "odom_topic_name": "/ugv/wheel/odometry",
                "robot_model": "hunter2",
                "simulated_robot": False,
                "publish_tf": False,
            }
        ],
        remappings=[("/cmd_vel", "/hunter_base/cmd_vel")],
        condition=IfCondition(start_hunter_driver),
    )
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_parameters, {"use_sim_time": False}],
    )
    uav_navigation = Node(
        package="air_ground_sim",
        executable="uav_navigation",
        name="uav_navigation",
        output="screen",
        parameters=[parameters],
        condition=IfCondition(start_uav_navigation),
    )
    web_gateway = Node(
        package="air_ground_sim",
        executable="web_gateway",
        name="web_gateway",
        output="screen",
        parameters=[parameters],
        condition=IfCondition(start_web_gateway),
    )
    mission = Node(
        package="air_ground_sim",
        executable="air_ground_mission",
        name="air_ground_mission",
        output="screen",
        parameters=[
            parameters,
            mission_parameters,
            {
                "auto_start": False,
                "simulation_lifecycle": False,
                "timeout_scale": 1.0,
            },
        ],
        condition=IfCondition(start_mission),
    )
    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[collision_parameters],
        condition=IfCondition(start_nav2),
    )
    collision_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_collision",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[collision_parameters],
        condition=IfCondition(start_nav2),
    )

    configured_nav2 = ReplaceString(
        source_file=nav2_source,
        replacements={
            "<ackermann_bt_xml>": behavior_tree,
            "<ackermann_through_bt_xml>": through_poses_behavior_tree,
            "<set_initial_pose>": initial_pose,
            "<initial_pose_x>": initial_x,
            "<initial_pose_y>": initial_y,
            "<initial_pose_yaw>": initial_yaw,
        },
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
        launch_arguments={
            "map": map_yaml,
            "params_file": configured_nav2,
            "use_sim_time": "false",
            "autostart": "true",
            "use_composition": "False",
            "use_respawn": "True",
            "slam": "False",
        }.items(),
        condition=IfCondition(start_nav2),
    )

    transforms = [
        _static_transform("base_link", "laser_frame", "0.30", "0", "0.18"),
        _static_transform("base_link", "imu_link", "0", "0", "0.04"),
        _static_transform("base_link", "ugv_camera_link", "0.39", "0", "0.08"),
        _static_transform("uav_base_link", "uav_lidar_frame", "0.10", "0", "0.03"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map", default_value=os.path.join(share, "maps", "deployment_map.yaml")
            ),
            DeclareLaunchArgument("can_port", default_value="can0"),
            DeclareLaunchArgument(
                "mission_parameters",
                default_value=default_mission_parameters,
                description="Commissioned site mission YAML; the shipped template is deliberately blocked",
            ),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("start_hunter_driver", default_value="true"),
            DeclareLaunchArgument("start_uav_interfaces", default_value="true"),
            DeclareLaunchArgument("start_uav_navigation", default_value="true"),
            DeclareLaunchArgument(
                "start_web_gateway",
                default_value="true",
                description="Start the read-only-by-default browser gateway",
            ),
            DeclareLaunchArgument(
                "start_mission",
                default_value="true",
                description="Start the idle-by-default guarded mission supervisor; execution still requires readiness and an explicit start",
            ),
            DeclareLaunchArgument(
                "ugv_adapter",
                default_value="ackermann",
                description="diff_drive, ackermann, or four_wheel_steering",
            ),
            DeclareLaunchArgument(
                "set_initial_pose",
                default_value="false",
                description="Use configured initial pose instead of RViz /initialpose",
            ),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            interfaces,
            chassis_adapter,
            docking_hardware_gateway,
            hunter_driver,
            ekf,
            uav_navigation,
            web_gateway,
            mission,
            *transforms,
            TimerAction(period=3.0, actions=[nav2, collision_monitor, collision_lifecycle]),
        ]
    )
