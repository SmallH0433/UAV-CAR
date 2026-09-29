import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    lidar_params = os.path.join(
        get_package_share_directory('car_nodes'),
        'config',
        'lslidar_n10p_uart.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('simulate', default_value='false'),
        DeclareLaunchArgument('motor_simulate', default_value='false'),
        DeclareLaunchArgument('motor_port', default_value='/dev/wheeltec'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/wheeltec_lidar'),
        DeclareLaunchArgument('enable_cruise', default_value='false'),
        DeclareLaunchArgument('enable_vision', default_value='false'),
        DeclareLaunchArgument('safety_distance', default_value='1.0'),
        DeclareLaunchArgument('slow_down_distance', default_value='1.8'),
        DeclareLaunchArgument('creep_speed', default_value='0.25'),
        DeclareLaunchArgument('lidar_tf_x', default_value='0.1'),
        DeclareLaunchArgument('lidar_tf_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_z', default_value='0.15'),
        DeclareLaunchArgument('web_port', default_value='8765'),
        Node(package='car_nodes', executable='uav_bridge_node', output='screen',
             parameters=[{'uav_ip':'192.168.50.3'}]),
        Node(package='car_nodes', executable='leadscrew_driver_node', output='screen',
             parameters=[{'simulate': ParameterValue(LaunchConfiguration('simulate'), value_type=bool)}]),
        Node(
            package='lslidar_driver',
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            namespace='x10',
            output='screen',
            emulate_tty=True,
            parameters=[lidar_params, {
                'serial_port': LaunchConfiguration('lidar_port'),
            }],
        ),
        Node(
            package='car_nodes',
            executable='perception_node',
            output='screen',
            parameters=[{
                'enable_vision': ParameterValue(
                    LaunchConfiguration('enable_vision'), value_type=bool),
                # 10Hz 雷达下将细杆确认延迟由约 0.3s 降至约 0.2s。
                'thin_persist_frames': 2,
            }],
        ),
        Node(
            package='car_nodes',
            executable='avoidance_node',
            output='screen',
            parameters=[{
                'enable_cruise': ParameterValue(
                    LaunchConfiguration('enable_cruise'), value_type=bool),
                'safety_distance': ParameterValue(
                    LaunchConfiguration('safety_distance'), value_type=float),
                'slow_down_distance': ParameterValue(
                    LaunchConfiguration('slow_down_distance'), value_type=float),
                'creep_speed': ParameterValue(
                    LaunchConfiguration('creep_speed'), value_type=float),
                'vehicle_half_length': 0.2325,
                'vehicle_half_width': 0.235,
                'footprint_padding': 0.04,
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_frame_static_tf',
            output='screen',
            arguments=[
                LaunchConfiguration('lidar_tf_x'),
                LaunchConfiguration('lidar_tf_y'),
                LaunchConfiguration('lidar_tf_z'),
                '0', '0', '0', 'base_footprint', 'laser_frame',
            ],
        ),
        # Web teleop commands must pass through the same authority and safety
        # chain as the full vehicle bringup before reaching the chassis.
        Node(package='car_sim', executable='ugv_control_mux', output='screen',
             parameters=[{
                 'command_enabled': True,
                 'require_mission_status': False,
             }]),
        Node(package='car_sim', executable='ugv_command_gateway', output='screen',
             parameters=[{
                 'command_enabled': True,
                 'input_topic': '/ugv/control/cmd_vel',
                 'output_topic': '/ugv/gateway/cmd_vel',
             }]),
        Node(package='car_nodes', executable='chassis_controller_node', output='screen',
             remappings=[('/cmd_vel', '/ugv/gateway/cmd_vel')]),
        Node(package='car_nodes', executable='motor_driver_node', output='screen',
             parameters=[{
                 'port': LaunchConfiguration('motor_port'),
                 'simulate': ParameterValue(
                     LaunchConfiguration('motor_simulate'), value_type=bool),
             }]),
        Node(package='car_sim', executable='web_gateway', output='screen',
             parameters=[{'bind_address': '0.0.0.0', 'port': ParameterValue(LaunchConfiguration('web_port'), value_type=int)}]),
    ])
