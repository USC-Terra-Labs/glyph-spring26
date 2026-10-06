import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRestamper(Node):
    def __init__(self) -> None:
        super().__init__("go2_scan_restamper")
        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("min_cluster_size", 1)
        self.declare_parameter("cluster_range_jump", 0.18)
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.min_cluster_size = int(self.get_parameter("min_cluster_size").value)
        self.cluster_range_jump = float(self.get_parameter("cluster_range_jump").value)
        self.pub = self.create_publisher(LaserScan, output_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            LaserScan,
            input_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )

    @staticmethod
    def _is_valid_range(value: float, range_min: float, range_max: float) -> bool:
        return math.isfinite(value) and range_min <= value <= range_max

    def _filter_small_clusters(self, msg: LaserScan) -> None:
        if self.min_cluster_size <= 1:
            return

        ranges = list(msg.ranges)
        clusters = []
        start = None
        prev = None

        for idx, value in enumerate(ranges):
            if not self._is_valid_range(value, msg.range_min, msg.range_max):
                if start is not None:
                    clusters.append((start, idx - 1))
                    start = None
                    prev = None
                continue

            if start is None:
                start = idx
                prev = value
                continue

            if abs(value - prev) > self.cluster_range_jump:
                clusters.append((start, idx - 1))
                start = idx
            prev = value

        if start is not None:
            clusters.append((start, len(ranges) - 1))

        for start_idx, end_idx in clusters:
            if (end_idx - start_idx + 1) < self.min_cluster_size:
                for idx in range(start_idx, end_idx + 1):
                    ranges[idx] = float("inf")

        msg.ranges = ranges

    def _on_scan(self, msg: LaserScan) -> None:
        msg.header.stamp = self.get_clock().now().to_msg()
        self._filter_small_clusters(msg)
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRestamper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
