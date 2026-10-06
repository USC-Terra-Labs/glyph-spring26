import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    foxglove = LaunchConfiguration("foxglove")
    map_yaml = LaunchConfiguration("map")
    use_nav2 = LaunchConfiguration("nav2")
    use_perfect_localization = LaunchConfiguration("perfect_localization")
    publish_initial_pose = LaunchConfiguration("publish_initial_pose")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_delay = LaunchConfiguration("initial_pose_delay")
    use_behavior_supervisor = LaunchConfiguration("behavior_supervisor")
    use_location_subscriber = LaunchConfiguration("location_subscriber")
    target_topic = LaunchConfiguration("target_topic")
    nav2_start_delay = LaunchConfiguration("nav2_start_delay")
    auto_recovery = LaunchConfiguration("auto_recovery")
    movement_gate_topic = LaunchConfiguration("movement_gate_topic")
    return_home_trigger_topic = LaunchConfiguration("return_home_trigger_topic")
    home_target_topic = LaunchConfiguration("home_target_topic")
    set_home_topic = LaunchConfiguration("set_home_topic")
    flavor_selection_topic = LaunchConfiguration("flavor_selection_topic")
    currently_dispensing_topic = LaunchConfiguration("currently_dispensing_topic")
    dispense_empty_topic = LaunchConfiguration("dispense_empty_topic")
    dispense_uart_port = LaunchConfiguration("dispense_uart_port")
    dispense_uart_baudrate = LaunchConfiguration("dispense_uart_baudrate")

    sim_launch = os.path.join(
        get_package_share_directory("go2_unitree_bridge"),
        "launch",
        "go2_rlsar_sim.launch.py",
    )
    localization_launch = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch",
        "localization_launch.py",
    )
    navigation_launch = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    )
    nav2_params = os.path.join(
        get_package_share_directory("go2_navigation"),
        "config",
        "rlsar_nav2_localization_mppi.yaml",
    )
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("nav2", default_value="true"),
            DeclareLaunchArgument("nav2_params_file", default_value=nav2_params),
            DeclareLaunchArgument("perfect_localization", default_value="true"),
            DeclareLaunchArgument("publish_initial_pose", default_value="true"),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_delay", default_value="3.0"),
            DeclareLaunchArgument("behavior_supervisor", default_value="true"),
            DeclareLaunchArgument("location_subscriber", default_value="false"),
            DeclareLaunchArgument("auto_recovery", default_value="true"),
            DeclareLaunchArgument("target_topic", default_value="/move_base_simple/goal"),
            DeclareLaunchArgument("nav2_start_delay", default_value="8.0"),
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
                default_value="/home/ming/ros2_ws/src/go2_navigation/maps/rlsar_scene.yaml",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={"foxglove": foxglove}.items(),
            ),
            TimerAction(
                period=nav2_start_delay,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(localization_launch),
                        condition=IfCondition(
                            PythonExpression(
                                ["'", use_nav2, "' == 'true' and '", use_perfect_localization, "' != 'true'"]
                            )
                        ),
                        launch_arguments={
                            "map": map_yaml,
                            "use_sim_time": "False",
                            "autostart": "True",
                            "params_file": nav2_params_file,
                            "use_composition": "False",
                        }.items(),
                    ),
                    Node(
                        package="nav2_map_server",
                        executable="map_server",
                        name="map_server",
                        condition=IfCondition(
                            PythonExpression(
                                ["'", use_nav2, "' == 'true' and '", use_perfect_localization, "' == 'true'"]
                            )
                        ),
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": False,
                                "yaml_filename": map_yaml,
                            }
                        ],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_localization",
                        condition=IfCondition(
                            PythonExpression(
                                ["'", use_nav2, "' == 'true' and '", use_perfect_localization, "' == 'true'"]
                            )
                        ),
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": False,
                                "autostart": True,
                                "node_names": ["map_server"],
                            }
                        ],
                    ),
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="map_to_odom_tf",
                        condition=IfCondition(
                            PythonExpression(
                                ["'", use_nav2, "' == 'true' and '", use_perfect_localization, "' == 'true'"]
                            )
                        ),
                        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                        output="screen",
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(navigation_launch),
                        condition=IfCondition(use_nav2),
                        launch_arguments={
                            "use_sim_time": "False",
                            "autostart": "True",
                            "params_file": nav2_params_file,
                            "use_composition": "False",
                        }.items(),
                    ),
                    Node(
                        package="go2_navigation",
                        executable="cmd_vel_arbiter",
                        name="go2_cmd_vel_arbiter",
                        condition=IfCondition(use_nav2),
                        parameters=[
                            {
                                "teleop_input_topic": "/cmd_vel_teleop",
                                "nav_input_topic": "/cmd_vel_nav",
                                "dance_input_topic": "/cmd_vel_dance",
                                "output_topic": "/cmd_vel",
                                "nav_timeout_sec": 1.5,
                                "dance_timeout_sec": 1.5,
                            }
                        ],
                        output="screen",
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
                                "delay_sec": initial_pose_delay,
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="go2_navigation",
                        executable="location_subscriber",
                        name="go2_location_subscriber",
                        condition=IfCondition(
                            PythonExpression(
                                ["'", use_location_subscriber, "' == 'true' and '", use_behavior_supervisor, "' != 'true'"]
                            )
                        ),
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
                        condition=IfCondition(
                            PythonExpression(
                                ["'", use_location_subscriber, "' == 'true' and '", use_behavior_supervisor, "' != 'true'"]
                            )
                        ),
                        parameters=[
                            {
                                "target_topic": "/target_location",
                                "use_sim_time": False,
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="go2_navigation",
                        executable="location_subscriber",
                        name="go2_location_subscriber_dispatch",
                        condition=IfCondition(use_behavior_supervisor),
                        parameters=[
                            {
                                "target_topic": "/behavior_supervisor_dispatch_goal",
                                "use_sim_time": False,
                                "goal_cleared_topic": "/behavior_supervisor_dispatch_cleared",
                                "goal_failed_topic": "/behavior_supervisor_dispatch_failed",
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="go2_navigation",
                        executable="location_subscriber",
                        name="go2_location_subscriber_return_home",
                        condition=IfCondition(
                            PythonExpression(
                                ["'", use_location_subscriber, "' == 'true' or '", use_behavior_supervisor, "' == 'true'"]
                            )
                        ),
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
                        name="go2_goal_tolerance_marker",
                        condition=IfCondition(
                            PythonExpression(
                                ["'", use_location_subscriber, "' == 'true' and '", use_behavior_supervisor, "' != 'true'"]
                            )
                        ),
                        parameters=[
                            {
                                "target_topic": target_topic,
                                "marker_topic": "/target_location_tolerance",
                                "goal_cleared_topic": "/target_location_cleared",
                                "radius": 0.5,
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="go2_navigation",
                        executable="goal_tolerance_marker",
                        name="go2_goal_tolerance_marker_target_location",
                        condition=IfCondition(
                            PythonExpression(
                                ["'", use_location_subscriber, "' == 'true' and '", use_behavior_supervisor, "' != 'true'"]
                            )
                        ),
                        parameters=[
                            {
                                "target_topic": "/target_location",
                                "marker_topic": "/target_location_tolerance",
                                "goal_cleared_topic": "/target_location_cleared",
                                "radius": 0.5,
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="go2_navigation",
                        executable="goal_tolerance_marker",
                        name="go2_goal_tolerance_marker_dispatch",
                        condition=IfCondition(use_behavior_supervisor),
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
                        condition=IfCondition(
                            PythonExpression(
                                ["'", use_location_subscriber, "' == 'true' or '", use_behavior_supervisor, "' == 'true'"]
                            )
                        ),
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
                    Node(
                        package="go2_navigation",
                        executable="sim_body_motion_controller",
                        name="go2_sim_body_motion_controller",
                        condition=IfCondition(use_behavior_supervisor),
                        parameters=[
                            {
                                "motion_topic": "/sim_body_motion",
                                "cmd_vel_topic": "/cmd_vel_dance",
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="go2_navigation",
                        executable="sim_behavior_supervisor",
                        name="go2_sim_behavior_supervisor",
                        condition=IfCondition(use_behavior_supervisor),
                        parameters=[
                            {
                                "target_topic": target_topic,
                                "target_location_topic": "/target_location",
                                "dispatch_target_topic": "/behavior_supervisor_dispatch_goal",
                                "movement_gate_topic": movement_gate_topic,
                                "return_home_trigger_topic": return_home_trigger_topic,
                                "dispense_empty_topic": dispense_empty_topic,
                                "home_target_topic": home_target_topic,
                                "body_motion_topic": "/sim_body_motion",
                                "home_align_cmd_vel_topic": "/cmd_vel_dance",
                                "set_home_topic": set_home_topic,
                                "home_x": initial_pose_x,
                                "home_y": initial_pose_y,
                                "home_yaw": initial_pose_yaw,
                                "home_reached_radius": 0.18,
                                "home_reached_yaw_tol": 0.12,
                                "home_resume_radius": 0.28,
                                "home_resume_yaw_tol": 0.22,
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
                        executable="sim_fall_recovery",
                        name="go2_sim_fall_recovery",
                        condition=IfCondition(auto_recovery),
                        parameters=[
                            {
                                "imu_topic": "/imu/data",
                                "target_topic": target_topic,
                                "reset_service": "/go2_rlsar_reset",
                            }
                        ],
                        output="screen",
                    ),
                ],
            ),
        ]
    )
