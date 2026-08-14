"""R680 阿克曼实机 bringup（树莓派 4B）：全链路。

lidar_driver（RPLIDAR C1 串口）→ /scan → perception_node → avoidance_node
camera_driver（V4L2 摄像头）→ /camera/image_raw → perception / web
ugv_control_mux（teleop 优先，无 operator heartbeat 放行避障 /cmd_vel）
  → ugv_command_gateway（限幅 + 看门狗停车）
  → chassis_controller（自行车模型）→ /ackermann_cmd
  → motor_driver（WHEELTEC 串口协议 → STM32，实机）
web_gateway 提供 http://<树莓派IP>:8765 遥控页。

启动（按实际串口设备名调整）：
  ros2 launch car_sim real_bringup.launch.py \
    motor_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0 camera_device:=/dev/video0

不上电机/雷达做空载链路测试：
  ros2 launch car_sim real_bringup.launch.py motor_simulate:=true lidar_simulate:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    motor_port = LaunchConfiguration('motor_port')
    lidar_port = LaunchConfiguration('lidar_port')
    camera_device = LaunchConfiguration('camera_device')
    motor_simulate = LaunchConfiguration('motor_simulate')
    lidar_simulate = LaunchConfiguration('lidar_simulate')
    camera_simulate = LaunchConfiguration('camera_simulate')
    enable_cruise = LaunchConfiguration('enable_cruise')
    web_bind = LaunchConfiguration('web_bind')

    lidar = Node(
        package='car_nodes',
        executable='lidar_driver_node',
        name='lidar_driver_node',
        output='screen',
        parameters=[{
            'port': lidar_port,
            'simulate': lidar_simulate,
        }],
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
            'lidar_port', default_value='/dev/ttyUSB0',
            description='RPLIDAR C1 串口设备'),
        DeclareLaunchArgument(
            'camera_device', default_value='/dev/video0',
            description='V4L2 摄像头设备'),
        DeclareLaunchArgument(
            'motor_simulate', default_value='false',
            description='true=电机空载仿真（不开串口，反馈=指令）'),
        DeclareLaunchArgument(
            'lidar_simulate', default_value='false',
            description='true=雷达仿真（发布模拟场景）'),
        DeclareLaunchArgument(
            'camera_simulate', default_value='false',
            description='true=摄像头仿真（渐变测试图）'),
        DeclareLaunchArgument(
            'enable_cruise', default_value='false',
            description='true=上电即开启自主巡航避障'),
        DeclareLaunchArgument(
            'web_bind', default_value='0.0.0.0',
            description='网页控制台监听地址；0.0.0.0=允许局域网访问'),
        lidar,
        camera,
        perception,
        avoidance,
        control_mux,
        command_gateway,
        chassis_controller,
        motor_driver,
        web_gateway,
    ])
