"""Launch Gazebo, the ArduPilot SITL process and all ROS 2 bridges."""

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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("air_ground_sim")
    package_models = os.path.join(share, "models")
    bridge_config = os.path.join(share, "config", "gazebo_bridge.yaml")

    default_ardupilot = os.environ.get("ARDUPILOT_DIR", os.path.expanduser("~/ardupilot"))
    default_gazebo_plugin = os.environ.get(
        "ARDUPILOT_GAZEBO_DIR", os.path.expanduser("~/ardupilot_gazebo")
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
    start_sitl = LaunchConfiguration("start_sitl")
    start_bridge = LaunchConfiguration("start_bridge")
    ardupilot_dir = LaunchConfiguration("ardupilot_dir")
    world = LaunchConfiguration("world")
    home = LaunchConfiguration("home")

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "3", PathJoinSubstitution([share, "worlds", world])],
        output="screen",
        condition=IfCondition(start_gazebo),
    )

    sitl = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([ardupilot_dir, "build", "sitl", "bin", "arducopter"]),
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
                    [ardupilot_dir, "Tools", "autotest", "default_params", "gazebo-iris.parm"]
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
        name="air_ground_gazebo_bridge",
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
        launch_arguments={"profile": "sim"}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("start_sitl", default_value="true"),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("ardupilot_dir", default_value=default_ardupilot),
            DeclareLaunchArgument(
                "world",
                default_value="cmac_test_field.sdf",
                description=(
                    "World file installed by air_ground_sim. Options include "
                    "cmac_test_field.sdf, air_ground.sdf, astra_forest.sdf and "
                    "astra_dynamic_avoidance.sdf."
                ),
            ),
            DeclareLaunchArgument(
                "home",
                default_value="-35.363262,149.165237,584,353",
                description=(
                    "ArduPilot SITL start location as latitude,longitude,altitude,yaw. "
                    "The default matches the CMAC test field world."
                ),
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path),
            # WSLg's D3D12 Mesa path can crash Ogre2's render thread during
            # long camera runs.  llvmpipe is slower but much more predictable
            # for this development / AprilTag test environment.
            SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
            SetEnvironmentVariable("GALLIUM_DRIVER", "llvmpipe"),
            gazebo,
            TimerAction(period=2.0, actions=[clock_relay, bridge, interfaces]),
            # Bind the ROS-side UDP endpoint before SITL starts sending MAVLink.
            TimerAction(period=4.0, actions=[sitl]),
        ]
    )
