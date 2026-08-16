"""WHEELTEC G60 GPS 实机启动：

厂商 nmea_navsat_driver（src/vendor/wheeltec_gps，wheeltec 修改版，支持
$GN/$GL talker，G60 输出 GN 系语句，不能用上游旧版）+ 本项目配置
（car_nodes/config/g60_gps.yaml：/dev/wheeltec_gps，9600，frame_id=gps）。

GPS 上电即输出 NMEA，无需启动命令。发布后验证：
  ros2 topic echo /fix        # sensor_msgs/NavSatFix
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('car_nodes'), 'config', 'g60_gps.yaml')

    driver = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='nmea_serial_driver',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([driver])
