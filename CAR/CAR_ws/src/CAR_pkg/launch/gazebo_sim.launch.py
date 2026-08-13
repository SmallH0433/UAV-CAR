"""Gazebo 仿真一键启动：加载测试世界、生成 R680 小车并发布机器人状态。

用法：
    ros2 launch CAR_pkg gazebo_sim.launch.py
    ros2 launch CAR_pkg gazebo_sim.launch.py x:=1.0 y:=-0.5 yaw:=1.57
    ros2 launch CAR_pkg gazebo_sim.launch.py world:=/path/to/other.world

启动后另开终端遥控：
    ros2 run car_control teleop_keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('CAR_pkg')
    description_share = get_package_share_directory('car_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    default_world = os.path.join(bringup_share, 'worlds', 'car_test.world')
    xacro_path = os.path.join(description_share, 'urdf', 'car_gazebo.xacro')
    gazebo_launch = os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = {
        'robot_description': Command(['xacro ', xacro_path])
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Gazebo world 文件绝对路径'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='是否使用仿真时钟'),
        DeclareLaunchArgument('x', default_value='0.0', description='出生点 X（米）'),
        DeclareLaunchArgument('y', default_value='0.0', description='出生点 Y（米）'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='出生朝向（弧度）'),

        # Gazebo 服务端 + 客户端
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world, 'verbose': 'false'}.items()),

        # 机器人状态发布（固定关节 TF 与 /robot_description）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
            output='screen'),

        # 从 /robot_description 生成小车模型
        # base_footprint 位于地面投影中心，抬高 5 cm 落下避免与地面初始穿插
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'r680_4wd',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', '0.05',
                '-Y', LaunchConfiguration('yaw'),
            ],
            output='screen'),
    ])
