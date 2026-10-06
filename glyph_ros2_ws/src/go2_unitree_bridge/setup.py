from glob import glob
import os

from setuptools import find_packages, setup


package_name = "go2_unitree_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.py"))),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ming",
    maintainer_email="maintainer@example.com",
    description="ROS 2 bridge for Unitree Go2 using official unitree_ros2 topics.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "go2_unitree_bridge_node = go2_unitree_bridge.bridge_node:main",
            "robot_description_publisher = go2_unitree_bridge.robot_description_publisher:main",
            "rlsar_obstacle_markers = go2_unitree_bridge.rlsar_obstacle_markers:main",
            "rlsar_scan_node = go2_unitree_bridge.rlsar_scan_node:main",
            "verify_rlsar_nav2 = go2_unitree_bridge.verify_rlsar_nav2:main",
            "verify_rlsar_sim = go2_unitree_bridge.verify_rlsar_sim:main",
        ],
    },
)
