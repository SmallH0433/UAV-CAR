from setuptools import setup

package_name = 'car_control'

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
    description='R680 4WD 无人车控制工具：键盘遥控节点',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop_keyboard = car_control.teleop_keyboard:main',
        ],
    },
)
