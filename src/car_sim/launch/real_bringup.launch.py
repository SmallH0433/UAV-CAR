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

用 K210 作前摄（固件 scripts/k210_firmware.py 烧录后 USB 直连，替代 V4L2 前摄）：
  ros2 launch car_sim real_bringup.launch.py front_camera:=k210 k210_port:=/dev/ttyUSB0

不上电机/雷达做空载链路测试：
  ros2 launch car_sim real_bringup.launch.py lidar_mode:=sim \
    motor_simulate:=true camera_simulate:=true

GPS（WHEELTEC G60）：默认随车启动（gps_port:=/dev/wheeltec_gps），
不需要时 gps_port:='' 关闭；定位输出 `ros2 topic echo /fix` 查看。

注意：Nav2 地图自主导航（AMCL + Nav2）因 Pi 4B 同时跑桌面环境性能不足
（雷达 460800 波特串口丢包、AMCL 无法收敛）已屏蔽——launch 不再提供
nav_mode 参数，相关文件（config/nav2_params.yaml、launch/nav2_stack.launch.py）
保留备用。自主巡航/避障仍由 avoidance_node 承担（原有功能不受影响）。

双向丝杆（ESP8266，对接锁定机构）：默认关闭，leadscrew_port:=/dev/ttyUSB1 启动；
状态 `ros2 topic echo /leadscrew/status`，指令示例：
  ros2 topic pub --once /leadscrew/cmd car_interfaces/msg/LeadscrewCommand \
    "{group: 0, command: 1}"   # command: 0=STOP 1=IN 2=OUT 3=RELAX 4=LOCK
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
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    camera_fps = LaunchConfiguration('camera_fps')
    rear_camera_device = LaunchConfiguration('rear_camera_device')
    motor_simulate = LaunchConfiguration('motor_simulate')
    camera_simulate = LaunchConfiguration('camera_simulate')
    enable_cruise = LaunchConfiguration('enable_cruise')
    enable_vision = LaunchConfiguration('enable_vision')
    safety_distance = LaunchConfiguration('safety_distance')
    slow_down_distance = LaunchConfiguration('slow_down_distance')
    creep_speed = LaunchConfiguration('creep_speed')
    web_bind = LaunchConfiguration('web_bind')
    gps_port = LaunchConfiguration('gps_port')
    leadscrew_port = LaunchConfiguration('leadscrew_port')
    leadscrew_simulate = LaunchConfiguration('leadscrew_simulate')
    front_camera = LaunchConfiguration('front_camera')
    k210_port = LaunchConfiguration('k210_port')
    lidar_tf_x = LaunchConfiguration('lidar_tf_x')
    lidar_tf_y = LaunchConfiguration('lidar_tf_y')
    lidar_tf_z = LaunchConfiguration('lidar_tf_z')

    n10p_params = os.path.join(
        get_package_share_directory('car_nodes'), 'config', 'lslidar_n10p_uart.yaml')
    g60_params = os.path.join(
        get_package_share_directory('car_nodes'), 'config', 'g60_gps.yaml')
    vendor_mode = IfCondition(PythonExpression(["'", lidar_mode, "' == 'vendor'"]))
    sim_mode = IfCondition(PythonExpression(["'", lidar_mode, "' == 'sim'"]))
    has_rear_camera = IfCondition(
        PythonExpression(["'", rear_camera_device, "' != ''"]))
    has_gps = IfCondition(PythonExpression(["'", gps_port, "' != ''"]))
    has_leadscrew = IfCondition(
        PythonExpression(["'", leadscrew_port, "' != ''"]))
    v4l2_front = IfCondition(
        PythonExpression(["'", front_camera, "' == 'v4l2'"]))
    k210_front = IfCondition(
        PythonExpression(["'", front_camera, "' == 'k210'"]))

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
            # 画面仅供网页显示（无视觉处理），低规格采集省 CPU：
            # 降低采集/转换/话题传输/网页编码四项开销
            'width': camera_width,
            'height': camera_height,
            'fps': camera_fps,
        }],
        condition=v4l2_front,
    )
    # K210 前摄（front_camera:=k210；固件 scripts/k210_firmware.py，USB 直连）
    camera_k210 = Node(
        package='car_nodes',
        executable='k210_camera_driver_node',
        name='k210_camera_driver_node',
        output='screen',
        parameters=[{'port': k210_port}],
        condition=k210_front,
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
        # 视觉辅助（亮度差占位判断）由独立开关控制，与是否接摄像头解耦——
        # 只作画面显示（front_camera=v4l2/k210）时保持 false 即可
        parameters=[{'enable_vision': enable_vision}],
    )
    avoidance = Node(
        package='car_nodes',
        executable='avoidance_node',
        name='avoidance_node',
        output='screen',
        parameters=[{
            'enable_cruise': enable_cruise,
            # 实机调优：0.5m 才绕行太晚（转弯半径 0.57m 绕不开），提前到 0.9m；
            # 蠕动提速让贴障转向角速度从 ~15°/s 提到 ~25°/s
            'safety_distance': safety_distance,
            'slow_down_distance': slow_down_distance,
            'creep_speed': creep_speed,
            # R680 465x385 mm + 470x470 mm 停机坪组合外廓。
            'vehicle_half_length': 0.2325,
            'vehicle_half_width': 0.235,
            'footprint_padding': 0.04,
        }],
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
    # HC-SR04 车尾超声波（GPIO14/15 直驱）：脱困倒车后方盲区急停
    ultrasonic = Node(
        package='car_nodes',
        executable='ultrasonic_driver_node',
        name='ultrasonic_driver_node',
        output='screen',
    )
    web_gateway = Node(
        package='car_sim',
        executable='web_gateway',
        name='web_gateway',
        output='screen',
        parameters=[{'bind_address': web_bind}],
    )
    # WHEELTEC G60 GPS（gps_port 非空时启动；上电即输出，无需启动命令）
    gps = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='nmea_serial_driver',
        output='screen',
        parameters=[g60_params, {'port': gps_port}],
        condition=has_gps,
    )
    # ESP8266 双向丝杆对接锁定机构（leadscrew_port 非空时启动；
    # 指令 /leadscrew/cmd，状态 /leadscrew/status，见头部 docstring）
    leadscrew = Node(
        package='car_nodes',
        executable='leadscrew_driver_node',
        name='leadscrew_driver_node',
        output='screen',
        parameters=[{
            'port': leadscrew_port,
            'simulate': leadscrew_simulate,
        }],
        condition=has_leadscrew,
    )
    # 常驻静态 TF base_footprint→laser_frame（雷达安装位置，建图依赖；
    # 数值需与 web_gateway 建图参数 mapping_lidar_x/y/z 一致）
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_static_tf',
        output='screen',
        arguments=[lidar_tf_x, lidar_tf_y, lidar_tf_z,
                   '0', '0', '0', 'base_footprint', 'laser_frame'],
    )
    # Nav2 地图自主导航已屏蔽（Pi 4B 桌面环境性能不足）：不再 include
    # nav2_stack.launch.py；文件保留备用，恢复时重新接入并传 nav_mode 参数。

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
            'camera_width', default_value='640',
            description='前摄采集宽度（仅网页显示用低规格，省 CPU）'),
        DeclareLaunchArgument(
            'camera_height', default_value='480',
            description='前摄采集高度'),
        DeclareLaunchArgument(
            'camera_fps', default_value='1',
            description='前摄采集帧率'),
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
            'enable_vision', default_value='false',
            description='true=感知节点启用视觉辅助判断；摄像头仅作画面显示时保持 false'),
        DeclareLaunchArgument(
            'safety_distance', default_value='1.0',
            description='扇区可通行阈值 m（实机调优：障碍进入该距离开始绕行）'),
        DeclareLaunchArgument(
            'slow_down_distance', default_value='1.8',
            description='开始减速的距离 m（实机调优：原 1.2）'),
        DeclareLaunchArgument(
            'creep_speed', default_value='0.25',
            description='贴障蠕动速度 m/s（实机调优：原 0.15，提速以加快贴障转向）'),
        DeclareLaunchArgument(
            'web_bind', default_value='0.0.0.0',
            description='网页控制台监听地址；0.0.0.0=允许局域网访问'),
        DeclareLaunchArgument(
            'gps_port', default_value='/dev/wheeltec_gps',
            description="WHEELTEC G60 GPS 串口（udev 规则名）；留空 ''=不启动 GPS"),
        DeclareLaunchArgument(
            'leadscrew_port', default_value='',
            description="ESP8266 双向丝杆串口（如 /dev/ttyUSB1）；留空 ''=不启动"),
        DeclareLaunchArgument(
            'leadscrew_simulate', default_value='true',
            description='true=丝杆本地仿真（不开串口，模拟状态机）'),
        DeclareLaunchArgument(
            'front_camera', default_value='v4l2',
            description="前摄类型：v4l2=camera_device 摄像头；k210=K210 串口推流摄像头；none=不启动前摄"),
        DeclareLaunchArgument(
            'k210_port', default_value='/dev/ttyUSB0',
            description='K210 USB 串口设备（front_camera:=k210 时生效）'),
        DeclareLaunchArgument(
            'lidar_tf_x', default_value='0.1',
            description='雷达相对 base_footprint 的 X 偏移 m（与建图参数一致）'),
        DeclareLaunchArgument(
            'lidar_tf_y', default_value='0.0',
            description='雷达相对 base_footprint 的 Y 偏移 m'),
        DeclareLaunchArgument(
            'lidar_tf_z', default_value='0.15',
            description='雷达相对 base_footprint 的 Z 偏移 m'),
        lidar_vendor,
        lidar_sim,
        camera,
        camera_k210,
        camera_rear,
        perception,
        avoidance,
        control_mux,
        command_gateway,
        chassis_controller,
        motor_driver,
        ultrasonic,
        web_gateway,
        gps,
        leadscrew,
        lidar_static_tf,
    ])
