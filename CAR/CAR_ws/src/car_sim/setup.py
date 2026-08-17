from setuptools import setup

package_name = 'car_sim'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/gz_bridge.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/car_sim.launch.py',
            'launch/teleop_test.launch.py',
            'launch/real_bringup.launch.py',
        ]),
        ('share/' + package_name + '/worlds', ['worlds/r680_test_field.sdf']),
        ('share/' + package_name + '/web', ['car_sim/web/index.html']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoimiya',
    maintainer_email='yoimiya@todo.todo',
    description='R680 仿真 + 实机部署：Gazebo 桥配置、控制权 mux、指令网关、网页遥控、实机 bringup launch',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ugv_control_mux = car_sim.ugv_control_mux:main',
            'ugv_command_gateway = car_sim.ugv_command_gateway:main',
            'web_gateway = car_sim.web_gateway:main',
        ],
    },
)
