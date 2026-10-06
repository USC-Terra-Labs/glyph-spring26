import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    foxglove = LaunchConfiguration("foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")
    use_slam = LaunchConfiguration("slam")
    use_nav2 = LaunchConfiguration("nav2")
    use_location_subscriber = LaunchConfiguration("location_subscriber")
    target_topic = LaunchConfiguration("target_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    use_ekf = LaunchConfiguration("use_ekf")
    use_lidar_odom = LaunchConfiguration("use_lidar_odom")
    use_mapping_reanchor = LaunchConfiguration("mapping_reanchor")

    bridge_launch = os.path.join(
        get_package_share_directory("go2_unitree_bridge"),
        "launch",
        "go2_unitree_bridge.launch.py",
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
        get_package_share_directory("go2_navigation"),
        "config",
        "robot_nav2_localization_mppi.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            DeclareLaunchArgument("slam", default_value="true"),
            DeclareLaunchArgument("nav2", default_value="false"),
            DeclareLaunchArgument("location_subscriber", default_value="false"),
            DeclareLaunchArgument("target_topic", default_value="/move_base_simple/goal"),
            DeclareLaunchArgument("cloud_topic", default_value="/utlidar/cloud_base"),
            DeclareLaunchArgument("use_ekf", default_value="false"),
            DeclareLaunchArgument("use_lidar_odom", default_value="false"),
            DeclareLaunchArgument("mapping_reanchor", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge_launch),
                condition=UnlessCondition(use_lidar_odom),
                launch_arguments={
                    "foxglove": foxglove,
                    "foxglove_port": foxglove_port,
                    "use_ekf": use_ekf,
                    "zero_on_start": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge_launch),
                condition=IfCondition(use_lidar_odom),
                launch_arguments={
                    "foxglove": foxglove,
                    "foxglove_port": foxglove_port,
                    "use_ekf": "false",
                    "publish_odom": "false",
                    "publish_planar_tf": "false",
                    "publish_body_tf": "false",
                    "zero_on_start": "true",
                }.items(),
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="go2_nav_pointcloud_to_laserscan",
                remappings=[
                    ("cloud_in", cloud_topic),
                    ("scan", "/scan_raw"),
                ],
                parameters=[
                    {
                        "target_frame": "base_link",
                        "transform_tolerance": 0.2,
                        "min_height": -0.15,
                        "max_height": 0.50,
                        "angle_min": -3.14159,
                        "angle_max": 3.14159,
                        "angle_increment": 0.0087,
                        "scan_time": 0.1,
                        "range_min": 0.20,
                        "range_max": 6.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
                output="screen",
            ),
            Node(
                package="go2_navigation",
                executable="scan_restamper",
                name="go2_scan_restamper",
                parameters=[
                    {
                        "input_topic": "/scan_raw",
                        "output_topic": "/scan",
                    }
                ],
                output="screen",
            ),
            Node(
                package="go2_navigation",
                executable="lidar_odom_bridge",
                name="go2_lidar_odom_bridge",
                condition=IfCondition(use_lidar_odom),
                parameters=[
                    {
                        "input_topic": "/utlidar/robot_odom",
                        "output_topic": "/odom",
                        "odom_frame": "odom",
                        "base_frame": "base_footprint",
                        "body_frame": "base_link",
                        "publish_tf": True,
                        "use_current_time": True,
                        "zero_on_start": True,
                    }
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                condition=IfCondition(use_slam),
                launch_arguments={
                    "slam_params_file": slam_params,
                    "use_sim_time": "false",
                }.items(),
            ),
            Node(
                package="go2_navigation",
                executable="mapping_pose_reanchor",
                name="go2_mapping_pose_reanchor",
                condition=IfCondition(use_mapping_reanchor),
                parameters=[
                    {
                        "input_topic": "/initialpose",
                        "serialize_service": "/slam_toolbox/serialize_map",
                        "deserialize_service": "/slam_toolbox/deserialize_map",
                        "pose_graph_file": "/tmp/go2_mapping_reanchor.posegraph",
                        "goal_frame_id": "map",
                    }
                ],
                output="screen",
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
                parameters=[{"target_topic": target_topic}],
                output="screen",
            ),
            Node(
                package="go2_navigation",
                executable="location_subscriber",
                name="go2_location_subscriber_target_location",
                condition=IfCondition(use_location_subscriber),
                parameters=[{"target_topic": "/target_location"}],
                output="screen",
            ),
        ]
    )
