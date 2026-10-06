import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    foxglove = LaunchConfiguration("foxglove")
    use_slam = LaunchConfiguration("slam")
    use_nav2 = LaunchConfiguration("nav2")
    use_location_subscriber = LaunchConfiguration("location_subscriber")
    target_topic = LaunchConfiguration("target_topic")

    sim_launch = os.path.join(
        get_package_share_directory("go2_unitree_bridge"),
        "launch",
        "go2_rlsar_sim.launch.py",
    )
    slam_launch = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "online_async_launch.py",
    )
    nav2_launch = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    )
    slam_params = os.path.join(
        get_package_share_directory("go2_robot_sdk"),
        "config",
        "mapper_params_online_async.yaml",
    )
    nav2_params = os.path.join(
        get_package_share_directory("go2_robot_sdk"),
        "config",
        "nav2_params.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("slam", default_value="true"),
            DeclareLaunchArgument("nav2", default_value="true"),
            DeclareLaunchArgument("location_subscriber", default_value="false"),
            DeclareLaunchArgument("target_topic", default_value="/move_base_simple/goal"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={"foxglove": foxglove}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                condition=IfCondition(use_slam),
                launch_arguments={
                    "slam_params_file": slam_params,
                    "use_sim_time": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                condition=IfCondition(use_nav2),
                launch_arguments={
                    "params_file": nav2_params,
                    "use_sim_time": "false",
                }.items(),
            ),
            Node(
                package="go2_navigation",
                executable="location_subscriber",
                name="go2_location_subscriber",
                condition=IfCondition(use_location_subscriber),
                parameters=[
                    {
                        "target_topic": target_topic,
                        "use_sim_time": False,
                    }
                ],
                output="screen",
            ),
            Node(
                package="go2_navigation",
                executable="location_subscriber",
                name="go2_location_subscriber_target_location",
                condition=IfCondition(use_location_subscriber),
                parameters=[
                    {
                        "target_topic": "/target_location",
                        "use_sim_time": False,
                    }
                ],
                output="screen",
            ),
        ]
    )
