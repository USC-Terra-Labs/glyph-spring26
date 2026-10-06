import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    description_pkg = get_package_share_directory("unitree_go2_description")
    bridge_pkg = get_package_share_directory("go2_unitree_bridge")
    xacro_path = os.path.join(description_pkg, "urdf", "unitree_go2_robot.xacro")
    ekf_params = os.path.join(bridge_pkg, "config", "robot_ekf.yaml")

    sport_state_topic = LaunchConfiguration("sport_state_topic")
    sport_state_fallback_topic = LaunchConfiguration("sport_state_fallback_topic")
    low_state_topic = LaunchConfiguration("low_state_topic")
    low_state_fallback_topic = LaunchConfiguration("low_state_fallback_topic")
    foxglove = LaunchConfiguration("foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")
    foxglove_simple_visuals = LaunchConfiguration("foxglove_simple_visuals")
    foxglove_include_velodyne = LaunchConfiguration("foxglove_include_velodyne")
    foxglove_include_realsense = LaunchConfiguration("foxglove_include_realsense")
    use_ekf = LaunchConfiguration("use_ekf")
    publish_planar_tf = LaunchConfiguration("publish_planar_tf")
    publish_body_tf = LaunchConfiguration("publish_body_tf")
    publish_odom = LaunchConfiguration("publish_odom")
    odom_topic = LaunchConfiguration("odom_topic")
    zero_on_start = LaunchConfiguration("zero_on_start")
    body_motion_topic = LaunchConfiguration("body_motion_topic")
    body_motion_state_topic = LaunchConfiguration("body_motion_state_topic")
    body_motion_state_plot_topic = LaunchConfiguration("body_motion_state_plot_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    common_bridge_params = {
        "sport_state_topic": sport_state_topic,
        "sport_state_fallback_topic": sport_state_fallback_topic,
        "low_state_topic": low_state_topic,
        "low_state_fallback_topic": low_state_fallback_topic,
        "publish_planar_tf": publish_planar_tf,
        "publish_body_tf": publish_body_tf,
        "publish_odom": publish_odom,
        "odom_topic": odom_topic,
        "zero_on_start": zero_on_start,
        "body_motion_topic": body_motion_topic,
        "body_motion_state_topic": body_motion_state_topic,
        "body_motion_state_plot_topic": body_motion_state_plot_topic,
        "cmd_vel_topic": cmd_vel_topic,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("sport_state_topic", default_value="lf/sportmodestate"),
            DeclareLaunchArgument("sport_state_fallback_topic", default_value="/sportmodestate"),
            DeclareLaunchArgument("low_state_topic", default_value="lf/lowstate"),
            DeclareLaunchArgument("low_state_fallback_topic", default_value="/lowstate"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            DeclareLaunchArgument("foxglove_simple_visuals", default_value="false"),
            DeclareLaunchArgument("foxglove_include_velodyne", default_value="false"),
            DeclareLaunchArgument("foxglove_include_realsense", default_value="false"),
            DeclareLaunchArgument("use_ekf", default_value="false"),
            DeclareLaunchArgument("publish_planar_tf", default_value="true"),
            DeclareLaunchArgument("publish_body_tf", default_value="true"),
            DeclareLaunchArgument("publish_odom", default_value="true"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("zero_on_start", default_value="false"),
            DeclareLaunchArgument("body_motion_topic", default_value="/body_motion"),
            DeclareLaunchArgument("body_motion_state_topic", default_value="/body_motion_state"),
            DeclareLaunchArgument("body_motion_state_plot_topic", default_value="/body_motion_state_plot"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="go2_robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro ",
                                xacro_path,
                                " simple_visuals:=",
                                foxglove_simple_visuals,
                                " include_velodyne:=",
                                foxglove_include_velodyne,
                                " include_realsense:=",
                                foxglove_include_realsense,
                            ]
                        ),
                        "use_sim_time": False,
                    }
                ],
            ),
            Node(
                package="go2_unitree_bridge",
                executable="robot_description_publisher",
                name="go2_robot_description_publisher",
                output="screen",
                parameters=[
                    {
                        "xacro_path": xacro_path,
                        "simple_visuals": foxglove_simple_visuals,
                        "include_velodyne": foxglove_include_velodyne,
                        "include_realsense": foxglove_include_realsense,
                    }
                ],
            ),
            Node(
                package="go2_unitree_bridge",
                executable="go2_unitree_bridge_node",
                name="go2_unitree_bridge",
                output="screen",
                condition=IfCondition(use_ekf),
                parameters=[
                    {
                        **common_bridge_params,
                        "odom_topic": "/odom_raw",
                        "publish_planar_tf": False,
                        "publish_body_tf": True,
                        "publish_odom": True,
                    }
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="go2_ekf_filter",
                output="screen",
                condition=IfCondition(use_ekf),
                parameters=[ekf_params],
                remappings=[("odometry/filtered", "/odom")],
            ),
            Node(
                package="go2_unitree_bridge",
                executable="go2_unitree_bridge_node",
                name="go2_unitree_bridge",
                output="screen",
                condition=UnlessCondition(use_ekf),
                parameters=[common_bridge_params],
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                condition=IfCondition(foxglove),
                parameters=[
                    {
                        "port": foxglove_port,
                        "topic_whitelist": [
                            "^/cmd_vel$",
                            "^/cmd_vel_teleop$",
                            "^/cmd_vel_muxed$",
                            "^/cmd_vel_nav$",
                            "^/cmd_vel_dance$",
                            "^/imu/data$",
                            "^/joint_states$",
                            "^/local_costmap/published_footprint$",
                            "^\/plan$",
                            "^\/local_plan$",
                            "^\/global_plan$",
                            "^\/transformed_global_plan$",
                            "^\/follow_path\/transformed_global_plan$",
                            "^\/local_costmap\/costmap$",
                            "^\/local_costmap\/costmap_raw$",
                            "^\/global_costmap\/costmap$",
                            "^\/global_costmap\/costmap_raw$",
                            "^\/trajectory$",
                            "^\/trajectories$",
                            "^\/mppi_trajectory$",
                            "^\/mppi_trajectories$",
                            "^\/mppi_controller\/.*$",
                            "^/map$",
                            "^/map_metadata$",
                            "^/odom$",
                            "^/odom_raw$",
                            "^/scan$",
                            "^/target_location_tolerance$",
                            "^/parameter_events$",
                            "^/robot_description$",
                            "^/rosout$",
                            "^/tf$",
                            "^/tf_static$",
                        ],
                        "client_topic_whitelist": [
                            "^/clicked_point$",
                            "^/initialpose$",
                            "^/move_base_simple/goal$",
                        ],
                        "capabilities": [
                            "clientPublish",
                            "assets",
                            "connectionGraph",
                        ],
                        "ignore_unresponsive_param_nodes": True,
                    }
                ],
            ),
        ]
    )
