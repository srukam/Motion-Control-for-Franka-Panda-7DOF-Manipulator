from glob import glob

from setuptools import find_packages, setup

package_name = "panda_arm_control_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="automatolabs",
    maintainer_email="automatolabs@gmail.com",
    description="ROS 2 port of panda_arm_control (Panda 7-DOF + UR5 6-DOF).",
    license="TODO",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "panda_arm_controller = panda_arm_control_ros2.panda_arm_controller:main",
            "ur5_arm_controller = panda_arm_control_ros2.ur5_arm_controller:main",
        ],
    },
)
