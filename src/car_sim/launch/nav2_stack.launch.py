"""Nav2 栈启动（定位 + 导航），基于 nav2_bringup 官方 Humble launch 改写。

与官方 localization_launch.py / navigation_launch.py 的差异：
- lifecycle_manager 显式设置 bond_timeout: 60.0（官方 launch 未暴露该参数，
  默认 4.0s；在 Pi 4B / 高负载机器上 DDS 心跳偶发超过 4s 会导致
  lifecycle manager 误判节点死亡、整栈反复重启——实机烟测已复现）
- 合并定位（map_server + amcl）与导航（controller/planner/behavior/
  bt_navigator/waypoint_follower/velocity_smoother/smoother）于一个文件
- 去掉 use_composition / respawn / namespace 等实机用不到的开关

参数文件用 car_sim/config/nav2_params.yaml（各节点 bond_timeout: 20.0）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={'yaml_filename': map_yaml_file},
            convert_types=True),
        allow_substs=True)

    lifecycle_localization = ['map_server', 'amcl']
    lifecycle_navigation = [
        'controller_server', 'smoother_server', 'planner_server',
        'behavior_server', 'bt_navigator', 'waypoint_follower',
        'velocity_smoother',
    ]

    default_params = os.path.join(
        get_package_share_directory('car_sim'), 'config', 'nav2_params.yaml')

    nodes = [
        # ---- 定位 ----
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='screen',
             parameters=[configured_params]),
        Node(package='nav2_amcl', executable='amcl',
             name='amcl', output='screen',
             parameters=[configured_params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'autostart': autostart},
                         {'node_names': lifecycle_localization},
                         # 心跳超时容忍：默认 4s 在高负载下会误杀整栈
                         {'bond_timeout': 60.0}]),
        # ---- 导航 ----
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[configured_params],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', output='screen',
             parameters=[configured_params]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[configured_params]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[configured_params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[configured_params]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen',
             parameters=[configured_params]),
        # 速度平滑器：controller 的 cmd_vel_nav → /cmd_vel（接 mux navigation 入口）
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen',
             parameters=[configured_params],
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': autostart},
                         {'node_names': lifecycle_navigation},
                         {'bond_timeout': 60.0}]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Nav2 参数文件完整路径'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='AMCL/map_server 加载的地图 yaml 完整路径'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='自动激活生命周期节点'),
        *nodes,
    ])
