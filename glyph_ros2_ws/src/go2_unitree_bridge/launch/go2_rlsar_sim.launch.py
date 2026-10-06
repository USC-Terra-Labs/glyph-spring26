import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    description_pkg = get_package_share_directory("unitree_go2_description")
    xacro_path = os.path.join(description_pkg, "urdf", "unitree_go2_robot.xacro")

    foxglove = LaunchConfiguration("foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")
    foxglove_simple_visuals = LaunchConfiguration("foxglove_simple_visuals")
    foxglove_include_velodyne = LaunchConfiguration("foxglove_include_velodyne")
    foxglove_include_realsense = LaunchConfiguration("foxglove_include_realsense")
    robot_name = LaunchConfiguration("robot_name")
    scene_name = LaunchConfiguration("scene_name")
    rlsar_root = LaunchConfiguration("rlsar_root")

    sim_cmd = [
        "bash",
        "-lc",
        [
            "cd ",
            rlsar_root,
            " && source /opt/ros/humble/setup.bash && xvfb-run -a ./cmake_build/bin/rl_sim_mujoco ",
            robot_name,
            " ",
            scene_name,
        ],
    ]

    return LaunchDescription(
        [
            SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            DeclareLaunchArgument("foxglove_simple_visuals", default_value="false"),
            DeclareLaunchArgument("foxglove_include_velodyne", default_value="false"),
            DeclareLaunchArgument("foxglove_include_realsense", default_value="false"),
            DeclareLaunchArgument("robot_name", default_value="go2"),
            DeclareLaunchArgument("scene_name", default_value="scene"),
            DeclareLaunchArgument(
                "rlsar_root",
                default_value=EnvironmentVariable(
                    "RLSAR_ROOT", default_value="/home/ming/rl_sar"
                ),
            ),
            ExecuteProcess(
                cmd=sim_cmd,
                name="rlsar_mujoco_sim",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="go2_rlsar_robot_state_publisher",
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
                name="go2_rlsar_robot_description_publisher",
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
                executable="rlsar_scan_node",
                name="go2_rlsar_scan",
                output="screen",
            ),
            Node(
                package="go2_unitree_bridge",
                executable="rlsar_obstacle_markers",
                name="go2_rlsar_obstacle_markers",
                output="screen",
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
                            "^/obstacle_markers$",
                            "^/odom$",
                            "^/scan$",
                            "^/target_location_tolerance$",
                            "^/parameter_events$",
                            "^/robot_description$",
                            "^/rosout$",
                            "^/tf$",
                            "^/tf_static$",
                        ],
                        "capabilities": [
                            "clientPublish",
                            "assets",
                            "connectionGraph",
                        ],
                        "client_topic_whitelist": [
                            "^/clicked_point$",
                            "^/initialpose$",
                            "^/move_base_simple/goal$",
                        ],
                        "ignore_unresponsive_param_nodes": True,
                    }
                ],
            ),
        ]
    )
