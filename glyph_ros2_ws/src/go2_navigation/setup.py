from glob import glob
import os

package_name = 'go2_navigation'

from setuptools import setup, find_packages

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Navigation utilities for Unitree Go2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'location_subscriber = go2_navigation.location_subscriber:main',
            'initial_pose_publisher = go2_navigation.initial_pose_publisher:main',
            'goal_tolerance_marker = go2_navigation.goal_tolerance_marker:main',
            'sim_fall_recovery = go2_navigation.sim_fall_recovery:main',
            'scan_restamper = go2_navigation.scan_restamper:main',
            'initial_pose_restamper = go2_navigation.initial_pose_restamper:main',
            'sim_behavior_supervisor = go2_navigation.sim_behavior_supervisor:main',
            'sim_body_motion_controller = go2_navigation.sim_body_motion_controller:main',
            'uart_dispense_bridge = go2_navigation.uart_dispense_bridge:main',
            'mapping_pose_reanchor = go2_navigation.mapping_pose_reanchor:main',
            'lidar_odom_bridge = go2_navigation.lidar_odom_bridge:main',
            'global_localization_trigger = go2_navigation.global_localization_trigger:main',
            'session_map_publisher = go2_navigation.session_map_publisher:main',
            'cmd_vel_arbiter = go2_navigation.cmd_vel_arbiter:main',
        ],
    },
)
