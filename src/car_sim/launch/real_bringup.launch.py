"""R680 阿克曼实机 bringup（树莓派 4B）：全链路。

lidar（镭神 N10P 串口版，厂商驱动 lslidar_driver）→ /scan → perception_node → avoidance_node
camera_driver（V4L2 摄像头）→ /camera/image_raw → perception / web
ugv_control_mux（teleop 优先，无 operator heartbeat 放行避障 /cmd_vel）
  → ugv_command_gateway（限幅 + 看门狗停车）
  → chassis_controller（自行车模型）→ /ackermann_cmd
  → motor_driver（WHEELTEC 串口协议 → STM32，实机）
web_gateway 提供 http://<树莓派IP>:8765 遥控页（前后双画面）。

雷达：厂商 ROS2 SDK 已 vendored 在 src/vendor/lslidar_ros2（lslidar_driver +
lslidar_msgs），随本工作区一起 colcon build 即可（Humble 需先装依赖）：
  sudo apt install libpcl-dev ros-humble-pcl-conversions libpcap-dev
N10P 电机上电即转，无需开工令；如需停转/恢复：
  ros2 topic pub --once /x10/motor_control std_msgs/msg/Int8 "{data: 0}"  # 0=停 1=转

启动（按实际串口设备名调整）：
  ros2 launch car_sim real_bringup.launch.py \
    motor_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0 camera_device:=/dev/video0

接了 USB 后置摄像头时（如 /dev/video1，遥控页显示后视画面）：
  ros2 launch car_sim real_bringup.launch.py rear_camera_device:=/dev/video1

不上电机/雷达做空载链路测试：
  ros2 launch car_sim real_bringup.launch.py lidar_mode:=sim \
    motor_simulate:=true camera_simulate:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    motor_port = LaunchConfiguration('motor_port')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_mode = LaunchConfiguration('lidar_mode')
    camera_device = LaunchConfiguration('camera_device')
    rear_camera_device = LaunchConfiguration('rear_camera_device')
    motor_simulate = LaunchConfiguration('motor_simulate')
    camera_simulate = LaunchConfiguration('camera_simulate')
    enable_cruise = LaunchConfiguration('enable_cruise')
    web_bind = LaunchConfiguration('web_bind')

    n10p_params = os.path.join(
        get_package_share_directory('car_nodes'), 'config', 'lslidar_n10p_uart.yaml')
    vendor_mode = IfCondition(PythonExpression(["'", lidar_mode, "' == 'vendor'"]))
    sim_mode = IfCondition(PythonExpression(["'", lidar_mode, "' == 'sim'"]))
    has_rear_camera = IfCondition(
        PythonExpression(["'", rear_camera_device, "' != ''"]))

    # 镭神 N10P 厂商驱动（lidar_mode:=vendor，默认）；命名空间 x10 与 yaml 一致，勿改。
    # 电机上电即转（驱动内 motor_running 默认 true），无需启动命令。
    lidar_vendor = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        namespace='x10',
        output='screen',
        emulate_tty=True,
        parameters=[n10p_params, {'serial_port': lidar_port}],
        condition=vendor_mode,
    )
    # 空载自检用的模拟雷达（lidar_mode:=sim）
    lidar_sim = Node(
        package='car_nodes',
        executable='lidar_driver_node',
        name='lidar_driver_node',
        output='screen',
        parameters=[{'simulate': True}],
        condition=sim_mode,
    )
    camera = Node(
        package='car_nodes',
        executable='camera_driver_node',
        name='camera_driver_node',
        output='screen',
        parameters=[{
            'device': camera_device,
            'simulate': camera_simulate,
        }],
    )
    # 后置 USB 摄像头（rear_camera_device 非空时启动第二实例）
    camera_rear = Node(
        package='car_nodes',
        executable='camera_driver_node',
        name='camera_rear_driver_node',
        output='screen',
        parameters=[{
            'device': rear_camera_device,
            'simulate': camera_simulate,
            'image_topic': '/camera/rear/image_raw',
            'info_topic': '/camera/rear/camera_info',
            'frame_id': 'rear_camera_optical_frame',
        }],
        condition=has_rear_camera,
    )
    perception = Node(
        package='car_nodes',
        executable='perception_node',
        name='perception_node',
        output='screen',
    )
    avoidance = Node(
        package='car_nodes',
        executable='avoidance_node',
        name='avoidance_node',
        output='screen',
        parameters=[{'enable_cruise': enable_cruise}],
    )
    # teleop 优先：operator heartbeat 新鲜时 /ugv/teleop/cmd_vel 覆盖 /cmd_vel；
    # 无 heartbeat 时放行 avoidance 的 /cmd_vel（navigation_topic 默认值）。
    # 巡航时 web_gateway 会联动推送 steering_assist（巡航中可手动转向）。
    control_mux = Node(
        package='car_sim',
        executable='ugv_control_mux',
        name='ugv_control_mux',
        output='screen',
        parameters=[{
            'command_enabled': True,
            'require_mission_status': False,
        }],
    )
    command_gateway = Node(
        package='car_sim',
        executable='ugv_command_gateway',
        name='ugv_command_gateway',
        output='screen',
        parameters=[{
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
        remappings=[('/cmd_vel', '/ugv/gateway/cmd_vel')],
    )
    motor_driver = Node(
        package='car_nodes',
        executable='motor_driver_node',
        name='motor_driver_node',
        output='screen',
        parameters=[{
            'port': motor_port,
            'simulate': motor_simulate,
        }],
    )
    web_gateway = Node(
        package='car_sim',
        executable='web_gateway',
        name='web_gateway',
        output='screen',
        parameters=[{'bind_address': web_bind}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'motor_port', default_value='/dev/ttyACM0',
            description='STM32 控制板串口（串口 3 的 USB 口，CH9102 一般 ttyACM*）'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/wheeltec_lidar',
            description='镭神 N10P 串口转接模块设备（udev 规则名；未配规则用实际 /dev/ttyUSB*）'),
        DeclareLaunchArgument(
            'lidar_mode', default_value='vendor',
            description="vendor=镭神 N10P 厂商驱动；sim=空载模拟雷达"),
        DeclareLaunchArgument(
            'camera_device', default_value='/dev/video0',
            description='V4L2 前视摄像头设备'),
        DeclareLaunchArgument(
            'rear_camera_device', default_value='',
            description='USB 后置摄像头设备（如 /dev/video1）；留空=不启动后摄'),
        DeclareLaunchArgument(
            'motor_simulate', default_value='false',
            description='true=电机空载仿真（不开串口，反馈=指令）'),
        DeclareLaunchArgument(
            'camera_simulate', default_value='false',
            description='true=摄像头仿真（渐变测试图，前后摄共用）'),
        DeclareLaunchArgument(
            'enable_cruise', default_value='false',
            description='true=上电即开启自主巡航避障'),
        DeclareLaunchArgument(
            'web_bind', default_value='0.0.0.0',
            description='网页控制台监听地址；0.0.0.0=允许局域网访问'),
        lidar_vendor,
        lidar_sim,
        camera,
        camera_rear,
        perception,
        avoidance,
        control_mux,
        command_gateway,
        chassis_controller,
        motor_driver,
        web_gateway,
    ])
