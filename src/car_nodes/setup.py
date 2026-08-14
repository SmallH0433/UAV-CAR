from setuptools import setup

package_name = 'car_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoimiya',
    maintainer_email='yoimiya@todo.todo',
    description='小车 7 个功能节点：驱动、感知、避障、底盘、电机、无人机桥接',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_driver_node = car_nodes.camera_driver:main',
            'lidar_driver_node = car_nodes.lidar_driver:main',
            'perception_node = car_nodes.perception_node:main',
            'avoidance_node = car_nodes.avoidance_node:main',
            'chassis_controller_node = car_nodes.chassis_controller:main',
            'motor_driver_node = car_nodes.motor_driver:main',
            'sim_motor_bridge_node = car_nodes.sim_motor_bridge:main',
            'uav_bridge_node = car_nodes.uav_bridge:main',
        ],
    },
)
