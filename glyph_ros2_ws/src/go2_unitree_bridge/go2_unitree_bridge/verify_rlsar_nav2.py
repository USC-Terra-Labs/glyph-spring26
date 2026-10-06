from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan


@dataclass
class TopicStatus:
    received: bool = False
    count: int = 0


class VerifyRLSarNav2(Node):
    def __init__(self) -> None:
        super().__init__("verify_rlsar_nav2")
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.topic_status: Dict[str, TopicStatus] = {
            "/odom": TopicStatus(),
            "/joint_states": TopicStatus(),
            "/imu/data": TopicStatus(),
            "/scan": TopicStatus(),
            "/map": TopicStatus(),
        }
        self.map_width = 0
        self.map_height = 0
        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.create_subscription(Odometry, "/odom", self._mark("/odom"), sensor_qos)
        self.create_subscription(JointState, "/joint_states", self._mark("/joint_states"), sensor_qos)
        self.create_subscription(Imu, "/imu/data", self._mark("/imu/data"), sensor_qos)
        self.create_subscription(LaserScan, "/scan", self._mark("/scan"), sensor_qos)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)

    def _mark(self, topic: str):
        def callback(_msg) -> None:
            status = self.topic_status[topic]
            status.received = True
            status.count += 1

        return callback

    def _on_map(self, msg: OccupancyGrid) -> None:
        status = self.topic_status["/map"]
        status.received = True
        status.count += 1
        self.map_width = int(msg.info.width)
        self.map_height = int(msg.info.height)

    def wait_for_topics(self, timeout_sec: float) -> bool:
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(status.received for status in self.topic_status.values()):
                return True
        return False

    def wait_for_action(self, timeout_sec: float) -> bool:
        return self.action_client.wait_for_server(timeout_sec=timeout_sec)


def main() -> int:
    rclpy.init()
    node = VerifyRLSarNav2()
    try:
        node.get_logger().info("Waiting for RL-sar Nav2 topics: /odom /joint_states /imu/data /scan /map")
        if not node.wait_for_topics(timeout_sec=20.0):
            missing = [topic for topic, status in node.topic_status.items() if not status.received]
            node.get_logger().error(f"FAIL: missing topics/messages: {', '.join(missing)}")
            return 1

        node.get_logger().info(
            "Topic check passed: "
            + ", ".join(
                f"{topic}={node.topic_status[topic].count}"
                for topic in ["/odom", "/joint_states", "/imu/data", "/scan", "/map"]
            )
        )

        node.get_logger().info("Waiting for /navigate_to_pose action server")
        if not node.wait_for_action(timeout_sec=15.0):
            node.get_logger().error("FAIL: /navigate_to_pose action server did not appear")
            return 1

        node.get_logger().info(
            f"PASS: map={node.map_width}x{node.map_height}, navigate_to_pose action is available"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
