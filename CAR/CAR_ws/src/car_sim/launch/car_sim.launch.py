"""R680 Gazebo 仿真全链路：

gz sim（r680_test_field 世界）
  → ros_gz_bridge（传感器 + /clock + cmd_vel/odometry）
  → perception_node → avoidance_node
  → ugv_control_mux（teleop 优先，无 heartbeat 放行避障指令）
  → ugv_command_gateway（限幅 + 0.5s 超时停车）
  → chassis_controller（/cmd_vel remap 自 /ugv/gateway/cmd_vel）
  → sim_motor_bridge（/ackermann_cmd ↔ gz，/motor_feedback 闭环）
  → web_gateway（http://127.0.0.1:8765 遥控页）

headless:=true 时以 `gz sim -r -s` 无头运行。
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

    # 让 gz 能找到 model://r680_4wd
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

    perception = Node(
        package='car_nodes',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[sim_time],
    )
    avoidance = Node(
        package='car_nodes',
        executable='avoidance_node',
        name='avoidance_node',
        output='screen',
        # 实机调优参数（rpi-4.3 树莓派实测，阿克曼转弯半径 0.57m 同样适用仿真）：
        # 障碍进入 1.0m 开始绕行；蠕动提速加快贴障转向
        parameters=[sim_time, {
            'safety_distance': 1.0,
            'slow_down_distance': 1.8,
            'creep_speed': 0.25,
        }],
    )
    # teleop 优先：operator heartbeat 新鲜时 /ugv/teleop/cmd_vel 覆盖 /cmd_vel；
    # 无 heartbeat 时放行 avoidance 的 /cmd_vel（navigation_topic 默认值）。
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
        perception,
        avoidance,
        control_mux,
        command_gateway,
        chassis_controller,
        sim_motor_bridge,
        web_gateway,
    ])
