"""N10P 激光雷达（串口版）实机启动：

厂商 lslidar_driver（src/vendor/lslidar_ros2）+ 本项目配置
（car_nodes/config/lslidar_n10p_uart.yaml：frame_id=laser_frame，/scan）。

电机上电即转，无需启动命令；停转/恢复：
  ros2 topic pub --once /x10/motor_control std_msgs/msg/Int8 "{data: 0}"  # 0=停 1=转
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('car_nodes'), 'config', 'lslidar_n10p_uart.yaml')

    driver = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        namespace='x10',  # 与 yaml 顶层命名空间一致，勿改
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([driver])
