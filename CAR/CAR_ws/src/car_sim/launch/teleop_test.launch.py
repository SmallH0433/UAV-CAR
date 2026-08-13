"""R680 纯遥控链路：不起 perception/avoidance，只验证网页遥控通路。

gz sim → ros_gz_bridge → ugv_control_mux → ugv_command_gateway
  → chassis_controller → sim_motor_bridge → gz
web_gateway 提供 http://127.0.0.1:8765 遥控页。

注意：此链路下没有节点发布 /cmd_vel（导航输入），没有 operator heartbeat
时 mux 输出停车指令；遥控时请保持网页处于活动状态。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():
    car_sim_share = get_package_share_directory('car_sim')
    car_description_share = get_package_share_directory('car_description')
    bridge_config = os.path.join(car_sim_share, 'config', 'gz_bridge.yaml')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    world_path = PathJoinSubstitution([car_sim_share, 'worlds', world])

    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [
            os.path.join(car_description_share, 'models'),
            ':',
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
        ],
    )

    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '3', world_path],
        output='screen',
        condition=UnlessCondition(headless),
    )
    gazebo_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '-v', '3', world_path],
        output='screen',
        condition=IfCondition(headless),
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}],
    )

    sim_time = {'use_sim_time': True}

    control_mux = Node(
        package='car_sim',
        executable='ugv_control_mux',
        name='ugv_control_mux',
        output='screen',
        parameters=[sim_time, {
            'command_enabled': True,
            'require_mission_status': False,
        }],
    )
    command_gateway = Node(
        package='car_sim',
        executable='ugv_command_gateway',
        name='ugv_command_gateway',
        output='screen',
        parameters=[sim_time, {
            'command_enabled': True,
            'input_topic': '/ugv/control/cmd_vel',
            'output_topic': '/ugv/gateway/cmd_vel',
        }],
    )
    chassis_controller = Node(
        package='car_nodes',
        executable='chassis_controller_node',
        name='chassis_controller_node',
        output='screen',
        parameters=[sim_time],
        remappings=[('/cmd_vel', '/ugv/gateway/cmd_vel')],
    )
    sim_motor_bridge = Node(
        package='car_nodes',
        executable='sim_motor_bridge_node',
        name='sim_motor_bridge_node',
        output='screen',
        parameters=[sim_time],
    )
    web_gateway = Node(
        package='car_sim',
        executable='web_gateway',
        name='web_gateway',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='r680_test_field.sdf',
            description='car_sim/worlds 下的世界文件名',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='true 时 gz sim -r -s 无头运行',
        ),
        set_resource_path,
        gazebo_gui,
        gazebo_headless,
        gz_bridge,
        control_mux,
        command_gateway,
        chassis_controller,
        sim_motor_bridge,
        web_gateway,
    ])
