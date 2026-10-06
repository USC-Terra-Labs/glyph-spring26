#!/usr/bin/env python3

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def resolve_xacro_path(param_xacro_path: str) -> str:
    if param_xacro_path and os.path.exists(param_xacro_path):
        return param_xacro_path

    try:
        description_pkg = get_package_share_directory("unitree_go2_description")
        candidate = os.path.join(description_pkg, "urdf", "unitree_go2_robot.xacro")
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass

    return "/opt/ros/humble/share/unitree_go2_description/urdf/unitree_go2_robot.xacro"


class RobotDescriptionPublisher(Node):
    def __init__(self) -> None:
        super().__init__("robot_description_publisher")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(String, "/robot_description", qos)

        self.declare_parameter("xacro_path", "")
        self.declare_parameter("ros_control_file", "")
        self.declare_parameter("simple_visuals", False)
        self.declare_parameter("include_velodyne", True)
        self.declare_parameter("include_realsense", True)

        self._xacro_path = resolve_xacro_path(str(self.get_parameter("xacro_path").value))
        self._ros_control_file = str(self.get_parameter("ros_control_file").value)
        self._simple_visuals = bool(self.get_parameter("simple_visuals").value)
        self._include_velodyne = bool(self.get_parameter("include_velodyne").value)
        self._include_realsense = bool(self.get_parameter("include_realsense").value)
        self._publish_count = 0

        self.get_logger().info(f"Using xacro path: {self._xacro_path}")
        self.timer = self.create_timer(1.0, self.publish_description)

    def publish_description(self) -> None:
        if not os.path.exists(self._xacro_path):
            self.get_logger().error(f"xacro file not found: {self._xacro_path}")
            return

        cmd = ["xacro", self._xacro_path]
        if self._ros_control_file:
            cmd.append(f"robot_controllers:={self._ros_control_file}")
        cmd.extend(
            [
                f"simple_visuals:={'true' if self._simple_visuals else 'false'}",
                f"include_velodyne:={'true' if self._include_velodyne else 'false'}",
                f"include_realsense:={'true' if self._include_realsense else 'false'}",
            ]
        )

        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().error(f"xacro failed: {result.stderr}")
            return

        self._publisher.publish(String(data=result.stdout))
        self._publish_count += 1
        self.get_logger().info("Published robot description")
        if self._publish_count >= 2:
            self.timer.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
