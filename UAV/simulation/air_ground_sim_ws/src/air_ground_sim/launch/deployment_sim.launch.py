"""Launch the deployment-oriented UAV/UGV simulation and closed-loop navigation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def _first_existing_path(environment_name: str, candidates: list[str]) -> str:
    configured = os.environ.get(environment_name)
    if configured:
        return os.path.expanduser(configured)
    expanded = [os.path.expanduser(candidate) for candidate in candidates]
    return next((candidate for candidate in expanded if os.path.isdir(candidate)), expanded[0])


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
    package_models = os.path.join(share, "models")
    interface_parameters = os.path.join(share, "config", "sim_interfaces.yaml")
    ekf_parameters = os.path.join(share, "config", "ugv_ekf.yaml")
    bridge_config = os.path.join(share, "config", "deployment_gazebo_bridge.yaml")
    nav2_source = os.path.join(share, "config", "nav2_ackermann.yaml")
    collision_parameters = os.path.join(
        share, "config", "collision_monitor_sim.yaml"
    )
    rviz_config = os.path.join(share, "rviz", "cooperative_mission.rviz")
    behavior_tree = os.path.join(
        share, "behavior_trees", "navigate_ackermann_safe.xml"
    )
    through_poses_behavior_tree = os.path.join(
        share, "behavior_trees", "navigate_through_poses_ackermann_safe.xml"
    )

    default_ardupilot = _first_existing_path(
        "ARDUPILOT_DIR",
        ["~/ardupilot", "~/projects/air_ground_open_source/01_flight_stack/ardupilot"],
    )
    default_gazebo_plugin = _first_existing_path(
        "ARDUPILOT_GAZEBO_DIR",
        [
            "~/ardupilot_gazebo",
            "~/projects/air_ground_open_source/06_simulation/ardupilot_gazebo",
        ],
    )
    existing_resources = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    existing_plugins = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    resource_path = os.pathsep.join(
        filter(
            None,
            [package_models, os.path.join(default_gazebo_plugin, "models"), existing_resources],
        )
    )
    plugin_path = os.pathsep.join(
        filter(None, [os.path.join(default_gazebo_plugin, "build"), existing_plugins])
    )

    start_gazebo = LaunchConfiguration("start_gazebo")
    headless = LaunchConfiguration("headless")
    start_sitl = LaunchConfiguration("start_sitl")
    start_bridge = LaunchConfiguration("start_bridge")
    start_nav2 = LaunchConfiguration("start_nav2")
    start_uav_interfaces = LaunchConfiguration("start_uav_interfaces")
    start_uav_navigation = LaunchConfiguration("start_uav_navigation")
    start_mission = LaunchConfiguration("start_mission")
    start_rviz = LaunchConfiguration("start_rviz")
    start_web_gateway = LaunchConfiguration("start_web_gateway")
    software_rendering = LaunchConfiguration("software_rendering")
    mission_auto_start = LaunchConfiguration("mission_auto_start")
    adapter_type = LaunchConfiguration("ugv_adapter")
    ardupilot_dir = LaunchConfiguration("ardupilot_dir")
    world = LaunchConfiguration("world")
    map_file = LaunchConfiguration("map")
    override_parameters = LaunchConfiguration("override_parameters")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    home = LaunchConfiguration("home")

    gazebo_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "3", PathJoinSubstitution([share, "worlds", world])],
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    start_gazebo,
                    "'.lower() == 'true' and '",
                    headless,
                    "'.lower() != 'true'",
                ]
            )
        ),
    )
    gazebo_headless = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-r",
            "-s",
            "-v",
            "3",
            PathJoinSubstitution([share, "worlds", world]),
        ],
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    start_gazebo,
                    "'.lower() == 'true' and '",
                    headless,
                    "'.lower() == 'true'",
                ]
            )
        ),
    )
    sitl = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([ardupilot_dir, "build", "sitl", "bin", "arducopter"]),
            # A SIL acceptance run must not inherit an engineer's previous
            # EEPROM values. Reload the versioned defaults on every launch.
            "-w",
            "-S",
            "--model",
            "JSON",
            "--speedup",
            "1",
            "--slave",
            "0",
            "--defaults",
            [
                PathJoinSubstitution(
                    [ardupilot_dir, "Tools", "autotest", "default_params", "copter.parm"]
                ),
                TextSubstitution(text=","),
                PathJoinSubstitution(
                    [
                        ardupilot_dir,
                        "Tools",
                        "autotest",
                        "default_params",
                        "gazebo-iris.parm",
                    ]
                ),
                TextSubstitution(text=","),
                PathJoinSubstitution([share, "config", "ardupilot_sitl.parm"]),
            ],
            "--sim-address=127.0.0.1",
            "--serial0=udpclient:127.0.0.1:14551",
            "--home",
            home,
            "-I0",
        ],
        cwd=ardupilot_dir,
        output="screen",
        condition=IfCondition(start_sitl),
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="deployment_gazebo_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
        condition=IfCondition(start_bridge),
    )
    clock_relay = Node(
        package="air_ground_sim",
        executable="simulation_clock_relay",
        name="simulation_clock_relay",
        output="screen",
        parameters=[
            {
                "input_topic": "/clock_raw",
                "output_topic": "/clock",
                "max_rate_hz": 100.0,
                "paused_keepalive_s": 0.5,
            }
        ],
        condition=IfCondition(start_bridge),
    )
    interfaces = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "interfaces.launch.py")),
        launch_arguments={
            "profile": "sim",
            "start_demo_motion": "false",
            "start_uav_interfaces": start_uav_interfaces,
            "override_parameters": override_parameters,
        }.items(),
    )
    chassis_adapter = Node(
        package="air_ground_sim",
        executable="ugv_chassis_adapter",
        name="ugv_chassis_adapter",
        output="screen",
        parameters=[
            interface_parameters,
            override_parameters,
            {"adapter_type": adapter_type},
        ],
    )
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_parameters, {"use_sim_time": True}],
    )
    uav_navigation = Node(
        package="air_ground_sim",
        executable="uav_navigation",
        name="uav_navigation",
        output="screen",
        parameters=[interface_parameters, override_parameters],
        condition=IfCondition(start_uav_navigation),
    )
    mission = Node(
        package="air_ground_sim",
        executable="air_ground_mission",
        name="air_ground_mission",
        output="screen",
        parameters=[
            interface_parameters,
            override_parameters,
            {"auto_start": mission_auto_start},
        ],
        condition=IfCondition(start_mission),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="cooperative_mission_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(start_rviz),
    )
    web_gateway = Node(
        package="air_ground_sim",
        executable="web_gateway",
        name="web_gateway",
        output="screen",
        parameters=[interface_parameters, override_parameters],
        condition=IfCondition(start_web_gateway),
    )
    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[collision_parameters],
        condition=IfCondition(start_nav2),
    )
    collision_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_collision",
        output="screen",
        parameters=[collision_parameters],
        condition=IfCondition(start_nav2),
    )

    configured_nav2 = ReplaceString(
        source_file=nav2_source,
        replacements={
            "<ackermann_bt_xml>": behavior_tree,
            "<ackermann_through_bt_xml>": through_poses_behavior_tree,
            "<set_initial_pose>": "true",
            "<initial_pose_x>": initial_pose_x,
            "<initial_pose_y>": initial_pose_y,
            "<initial_pose_yaw>": initial_pose_yaw,
        },
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
        launch_arguments={
            "map": PathJoinSubstitution([share, "maps", map_file]),
            "params_file": configured_nav2,
            "use_sim_time": "true",
            "autostart": "true",
            "use_composition": "False",
            "use_respawn": "False",
            "slam": "False",
        }.items(),
        condition=IfCondition(start_nav2),
    )

    transforms = [
        _static_transform("base_link", "laser_frame", "0.30", "0", "0.18"),
        _static_transform("base_link", "imu_link", "0", "0", "0.04"),
        _static_transform("base_link", "ugv_camera_link", "0.39", "0", "0.08"),
        _static_transform("uav_base_link", "uav_lidar_frame", "0.10", "0", "0.03"),
        _static_transform("uav_base_link", "uav_lidar_3d_frame", "0", "0", "0.10"),
        _static_transform("uav_base_link", "uav_stereo_left_optical_frame", "0.16", "0.05", "0.02"),
        _static_transform("uav_base_link", "uav_stereo_right_optical_frame", "0.16", "-0.05", "0.02"),
        _static_transform("uav_base_link", "uav_stereo_depth_optical_frame", "0.16", "0", "0.02"),
        _static_transform("uav_base_link", "uav_ultrasonic_front_frame", "0.19", "0", "-0.01"),
        _static_transform("uav_base_link", "uav_ultrasonic_rear_frame", "-0.19", "0", "-0.01"),
        _static_transform("uav_base_link", "uav_ultrasonic_left_frame", "0", "0.19", "-0.01"),
        _static_transform("uav_base_link", "uav_ultrasonic_right_frame", "0", "-0.19", "-0.01"),
        _static_transform("uav_base_link", "uav_ultrasonic_up_frame", "0", "0", "0.11"),
        _static_transform("uav_base_link", "uav_ultrasonic_down_frame", "0", "0", "-0.11"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run the Gazebo server without its GUI",
            ),
            DeclareLaunchArgument("start_sitl", default_value="true"),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("start_uav_interfaces", default_value="true"),
            DeclareLaunchArgument("start_uav_navigation", default_value="true"),
            DeclareLaunchArgument("start_mission", default_value="false"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("start_web_gateway", default_value="false"),
            DeclareLaunchArgument(
                "software_rendering",
                default_value="true",
                description=(
                    "Use stable llvmpipe rendering. Set false on a verified "
                    "WSLg/NVIDIA or native Ubuntu GPU for higher sensor rates."
                ),
            ),
            DeclareLaunchArgument("mission_auto_start", default_value="false"),
            DeclareLaunchArgument(
                "ugv_adapter",
                default_value="ackermann",
                description="diff_drive, ackermann, or four_wheel_steering",
            ),
            DeclareLaunchArgument("ardupilot_dir", default_value=default_ardupilot),
            DeclareLaunchArgument("world", default_value="deployment_test_field.sdf"),
            DeclareLaunchArgument("map", default_value="deployment_map.yaml"),
            DeclareLaunchArgument(
                "override_parameters",
                default_value=interface_parameters,
                description="YAML merged after sim_interfaces.yaml",
            ),
            DeclareLaunchArgument("initial_pose_x", default_value="-8.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="-5.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "home",
                default_value="-35.363262,149.165237,584,353",
                description="ArduPilot SITL latitude,longitude,altitude,yaw",
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path),
            SetEnvironmentVariable(
                "LIBGL_ALWAYS_SOFTWARE", "1", condition=IfCondition(software_rendering)
            ),
            SetEnvironmentVariable(
                "GALLIUM_DRIVER", "llvmpipe", condition=IfCondition(software_rendering)
            ),
            gazebo_gui,
            gazebo_headless,
            TimerAction(
                period=2.0,
                actions=[
                    clock_relay,
                    bridge,
                    interfaces,
                    chassis_adapter,
                    ekf,
                    uav_navigation,
                    *transforms,
                ],
            ),
            TimerAction(period=4.0, actions=[sitl]),
            TimerAction(period=5.0, actions=[nav2, collision_monitor, collision_lifecycle]),
            TimerAction(period=7.0, actions=[mission]),
            TimerAction(period=8.0, actions=[rviz, web_gateway]),
        ]
    )
