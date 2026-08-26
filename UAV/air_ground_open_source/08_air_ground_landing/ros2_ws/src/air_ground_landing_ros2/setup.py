from glob import glob

from setuptools import find_packages, setup


package_name = "air_ground_landing_ros2"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]) + ["air_ground_landing"],
    package_dir={"air_ground_landing": "../../../src/air_ground_landing"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml") + ["../../../config/moving_landing.prototype.json"],
        ),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Air Ground Project",
    maintainer_email="project@example.com",
    description="ROS 2 Elastic/IBVS adapters with fail-closed ArduPilot GUIDED execution.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "elastic_trajectory_adapter = air_ground_landing_ros2.elastic_trajectory_adapter:main",
            "ibvs_adapter = air_ground_landing_ros2.ibvs_adapter:main",
            "landing_target_adapter = air_ground_landing_ros2.landing_target_adapter:main",
            "simple_landing_coordinator = air_ground_landing_ros2.simple_landing_coordinator:main",
            "guided_executor = air_ground_landing_ros2.guided_executor:main",
        ]
    },
)
