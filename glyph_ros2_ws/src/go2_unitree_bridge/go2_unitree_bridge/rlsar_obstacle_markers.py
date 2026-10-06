from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


Rectangle = Tuple[float, float, float, float]


class RLSarObstacleMarkers(Node):
    def __init__(self) -> None:
        super().__init__("rlsar_obstacle_markers")

        self.obstacles: List[Rectangle] = [
            (-1.0, -0.35, 0.64, 1.79),
            (1.4, 2.05, 0.54, 1.79),
            (1.3, 2.05, -2.96, -1.81),
        ]

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, "/obstacle_markers", latched_qos)
        self.create_timer(1.0, self._publish_markers)
        self._publish_markers()

    def _publish_markers(self) -> None:
        msg = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for idx, (xmin, xmax, ymin, ymax) in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "rlsar_obstacles"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = 0.5 * (xmin + xmax)
            marker.pose.position.y = 0.5 * (ymin + ymax)
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            marker.scale.x = xmax - xmin
            marker.scale.y = ymax - ymin
            marker.scale.z = 0.4
            marker.color.r = 0.12
            marker.color.g = 0.12
            marker.color.b = 0.12
            marker.color.a = 0.85
            marker.lifetime.sec = 0
            marker.frame_locked = False
            msg.markers.append(marker)

        self.marker_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = RLSarObstacleMarkers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
