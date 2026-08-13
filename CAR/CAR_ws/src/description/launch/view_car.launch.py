"""在 RViz 中查看 R680 4WD 小车模型（无需 Gazebo）。

用法：
    ros2 launch car_description view_car.launch.py
    ros2 launch car_description view_car.launch.py use_joint_state_gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('car_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'CAR_description.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'view_car.rviz')

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    use_gui = LaunchConfiguration('use_joint_state_gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_joint_state_gui',
            default_value='true',
            description='是否启动 joint_state_publisher_gui 关节滑条窗口'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
            output='screen'),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'),
    ])
