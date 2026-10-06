import os
import json

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_persisted_home_defaults(home_pose_path: str) -> tuple[str, str, str]:
    try:
        with open(home_pose_path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
        return (
            str(float(data["home_x"])),
            str(float(data["home_y"])),
            str(float(data["home_yaw"])),
        )
    except Exception:
        return ("0.0", "0.0", "0.0")


def generate_launch_description() -> LaunchDescription:
    foxglove = LaunchConfiguration("foxglove")
    map_yaml = LaunchConfiguration("map")
    target_topic = LaunchConfiguration("target_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_delay = LaunchConfiguration("initial_pose_delay")
    publish_initial_pose = LaunchConfiguration("publish_initial_pose")
    global_localization = LaunchConfiguration("global_localization")
    global_localization_delay = LaunchConfiguration("global_localization_delay")
    nav2_start_delay = LaunchConfiguration("nav2_start_delay")
    movement_gate_topic = LaunchConfiguration("movement_gate_topic")
    return_home_trigger_topic = LaunchConfiguration("return_home_trigger_topic")
    home_target_topic = LaunchConfiguration("home_target_topic")
    set_home_topic = LaunchConfiguration("set_home_topic")
    flavor_selection_topic = LaunchConfiguration("flavor_selection_topic")
    currently_dispensing_topic = LaunchConfiguration("currently_dispensing_topic")
    dispense_empty_topic = LaunchConfiguration("dispense_empty_topic")
    dispense_uart_port = LaunchConfiguration("dispense_uart_port")
    dispense_uart_baudrate = LaunchConfiguration("dispense_uart_baudrate")

    bridge_launch = os.path.join(
        get_package_share_directory("go2_unitree_bridge"),
        "launch",
        "go2_unitree_bridge.launch.py",
    )
    localization_launch = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch",
        "localization_launch.py",
    )
    nav2_params = os.path.join(
        get_package_share_directory("go2_navigation"),
        "config",
        "robot_nav2_localization_mppi.yaml",
    )
    home_pose_path = os.path.join(
        get_package_share_directory("go2_navigation"),
        "config",
        "robot_home_pose.json",
    )
    persisted_home_x, persisted_home_y, persisted_home_yaw = _load_persisted_home_defaults(home_pose_path)

    return LaunchDescription(
        [
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("target_topic", default_value="/move_base_simple/goal"),
            DeclareLaunchArgument("cloud_topic", default_value="/utlidar/cloud_base"),
            DeclareLaunchArgument("initial_pose_x", default_value=persisted_home_x),
            DeclareLaunchArgument("initial_pose_y", default_value=persisted_home_y),
            DeclareLaunchArgument("initial_pose_yaw", default_value=persisted_home_yaw),
            DeclareLaunchArgument("initial_pose_delay", default_value="5.0"),
            DeclareLaunchArgument("publish_initial_pose", default_value="true"),
            DeclareLaunchArgument("global_localization", default_value="false"),
            DeclareLaunchArgument("global_localization_delay", default_value="8.0"),
            DeclareLaunchArgument("nav2_start_delay", default_value="12.0"),
            DeclareLaunchArgument("movement_gate_topic", default_value="/return_home_trigger"),
            DeclareLaunchArgument("return_home_trigger_topic", default_value=""),
            DeclareLaunchArgument("home_target_topic", default_value="/return_home_target_location"),
            DeclareLaunchArgument("set_home_topic", default_value="/set_home_here"),
            DeclareLaunchArgument("flavor_selection_topic", default_value="/flavor_selection"),
            DeclareLaunchArgument("currently_dispensing_topic", default_value="/currently_dispensing"),
            DeclareLaunchArgument("dispense_empty_topic", default_value="/dispense_empty"),
            DeclareLaunchArgument("dispense_uart_port", default_value="/dev/ttyTHS1"),
            DeclareLaunchArgument("dispense_uart_baudrate", default_value="115200"),
            DeclareLaunchArgument(
                "map",
                default_value="/home/ming/ros2_ws/src/go2_navigation/maps/robot_map.yaml",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge_launch),
                launch_arguments={
                    "foxglove": foxglove,
                    "use_ekf": "false",
                    "cmd_vel_topic": "/cmd_vel_muxed",
                    "body_motion_topic": "/body_motion_sdk_disabled",
                    "body_motion_state_topic": "/body_motion_state_sdk",
                    "body_motion_state_plot_topic": "/body_motion_state_plot_sdk",
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
                        "min_height": -0.10,
                        "max_height": 0.40,
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch),
                launch_arguments={
                    "map": map_yaml,
                    "use_sim_time": "False",
                    "autostart": "True",
                    "params_file": nav2_params,
                    "use_composition": "False",
                }.items(),
            ),
            TimerAction(
                period=nav2_start_delay,
                actions=[
                    Node(
                        package="go2_navigation",
                        executable="cmd_vel_arbiter",
                        name="go2_cmd_vel_arbiter",
                        parameters=[
                            {
                                "teleop_input_topic": "/cmd_vel_teleop",
                                "nav_input_topic": "/cmd_vel_nav",
                                "dance_input_topic": "/cmd_vel_dance",
                                "output_topic": "/cmd_vel_muxed",
                                "nav_timeout_sec": 1.5,
                                "dance_timeout_sec": 1.5,
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="nav2_controller",
                        executable="controller_server",
                        name="controller_server",
                        output="screen",
                        parameters=[nav2_params],
                        arguments=["--ros-args", "--log-level", "info"],
                        remappings=[
                            ("/tf", "tf"),
                            ("/tf_static", "tf_static"),
                            ("/cmd_vel", "/cmd_vel_nav"),
                        ],
                    ),
                    Node(
                        package="nav2_smoother",
                        executable="smoother_server",
                        name="smoother_server",
                        output="screen",
                        parameters=[nav2_params],
                        arguments=["--ros-args", "--log-level", "info"],
                        remappings=[
                            ("/tf", "tf"),
                            ("/tf_static", "tf_static"),
                            ("/cmd_vel", "/cmd_vel_nav"),
                        ],
                    ),
                    Node(
                        package="nav2_planner",
                        executable="planner_server",
                        name="planner_server",
                        output="screen",
                        parameters=[nav2_params],
                        arguments=["--ros-args", "--log-level", "info"],
                        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    ),
                    Node(
                        package="nav2_behaviors",
                        executable="behavior_server",
                        name="behavior_server",
                        output="screen",
                        parameters=[nav2_params],
                        arguments=["--ros-args", "--log-level", "info"],
                        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    ),
                    Node(
                        package="nav2_bt_navigator",
                        executable="bt_navigator",
                        name="bt_navigator",
                        output="screen",
                        parameters=[nav2_params],
                        arguments=["--ros-args", "--log-level", "info"],
                        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    ),
                    Node(
                        package="nav2_waypoint_follower",
                        executable="waypoint_follower",
                        name="waypoint_follower",
                        output="screen",
                        parameters=[nav2_params],
                        arguments=["--ros-args", "--log-level", "info"],
                        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",
                        output="screen",
                        arguments=["--ros-args", "--log-level", "info"],
                        parameters=[
                            {"use_sim_time": False},
                            {"autostart": True},
                            {
                                "node_names": [
                                    "controller_server",
                                    "smoother_server",
                                    "planner_server",
                                    "behavior_server",
                                    "bt_navigator",
                                    "waypoint_follower",
                                ]
                            },
                        ],
                    ),
                    Node(
                        package="go2_navigation",
                        executable="sim_behavior_supervisor",
                        name="go2_behavior_supervisor",
                        parameters=[
                            {
                                "target_topic": target_topic,
                                "target_location_topic": "/target_location",
                                "dispatch_target_topic": "/behavior_supervisor_dispatch_goal",
                                "movement_gate_topic": movement_gate_topic,
                                "return_home_trigger_topic": return_home_trigger_topic,
                                "dispense_empty_topic": dispense_empty_topic,
                                "home_target_topic": home_target_topic,
                                "body_motion_topic": "/body_motion",
                                "home_align_cmd_vel_topic": "/cmd_vel_dance",
                                "clear_local_costmap_service": "/local_costmap/clear_entirely_local_costmap",
                                "set_home_topic": set_home_topic,
                                "persist_home": True,
                                "home_persistence_path": home_pose_path,
                                "home_x": initial_pose_x,
                                "home_y": initial_pose_y,
                                "home_yaw": initial_pose_yaw,
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="go2_navigation",
                        executable="sim_body_motion_controller",
                        name="go2_robot_body_motion_controller",
                        parameters=[
                            {
                                "motion_topic": "/body_motion",
                                "cmd_vel_topic": "/cmd_vel_dance",
                                "state_topic": "/body_motion_state",
                                "state_plot_topic": "/body_motion_state_plot",
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="go2_navigation",
                        executable="uart_dispense_bridge",
                        name="go2_uart_dispense_bridge",
                        parameters=[
                            {
                                "port": dispense_uart_port,
                                "baudrate": dispense_uart_baudrate,
                                "flavor_selection_topic": flavor_selection_topic,
                                "currently_dispensing_topic": currently_dispensing_topic,
                                "dispense_empty_topic": dispense_empty_topic,
                                "movement_gate_topic": movement_gate_topic,
                                "return_home_trigger_topic": return_home_trigger_topic,
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="go2_navigation",
                        executable="location_subscriber",
                        name="go2_location_subscriber_dispatch",
                        parameters=[
                            {
                                "target_topic": "/behavior_supervisor_dispatch_goal",
                                "use_sim_time": False,
                                "goal_cleared_topic": "/behavior_supervisor_dispatch_cleared",
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="go2_navigation",
                        executable="location_subscriber",
                        name="go2_location_subscriber_return_home",
                        parameters=[
                            {
                                "target_topic": home_target_topic,
                                "use_sim_time": False,
                                "orient_toward_goal_center": False,
                                "goal_cleared_topic": "/behavior_supervisor_home_cleared",
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="go2_navigation",
                        executable="goal_tolerance_marker",
                        name="go2_goal_tolerance_marker_dispatch",
                        parameters=[
                            {
                                "target_topic": "/behavior_supervisor_dispatch_goal",
                                "marker_topic": "/target_location_tolerance",
                                "goal_cleared_topic": "/behavior_supervisor_dispatch_cleared",
                                "radius": 0.5,
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="go2_navigation",
                        executable="goal_tolerance_marker",
                        name="go2_goal_tolerance_marker_return_home",
                        parameters=[
                            {
                                "target_topic": home_target_topic,
                                "marker_topic": "/target_location_tolerance",
                                "goal_cleared_topic": "/behavior_supervisor_home_cleared",
                                "radius": 0.5,
                            }
                        ],
                        output="screen",
                    ),
                ],
            ),
            Node(
                package="go2_navigation",
                executable="initial_pose_publisher",
                name="go2_initial_pose_publisher",
                condition=IfCondition(publish_initial_pose),
                parameters=[
                    {
                        "topic": "/initialpose",
                        "frame_id": "map",
                        "x": initial_pose_x,
                        "y": initial_pose_y,
                        "yaw": initial_pose_yaw,
                        "use_sim_time": False,
                        "delay_sec": initial_pose_delay,
                    }
                ],
                output="screen",
            ),
            Node(
                package="go2_navigation",
                executable="global_localization_trigger",
                name="go2_global_localization_trigger",
                condition=IfCondition(global_localization),
                parameters=[
                    {
                        "service_name": "/reinitialize_global_localization",
                        "delay_sec": global_localization_delay,
                    }
                ],
                output="screen",
            ),
            Node(
                package="go2_navigation",
                executable="initial_pose_restamper",
                name="go2_initial_pose_restamper",
                parameters=[
                    {
                        "input_topic": "/initialpose_raw",
                        "output_topic": "/initialpose",
                    }
                ],
                output="screen",
            ),
            Node(
                package="go2_navigation",
                executable="session_map_publisher",
                name="go2_session_map_publisher",
                parameters=[
                    {
                        "base_map_topic": "/map",
                        "overlay_topic": "/global_costmap/costmap",
                        "output_topic": "/session_map",
                    }
                ],
                output="screen",
            ),
        ]
    )
