import math
from typing import List, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


Rectangle = Tuple[float, float, float, float]


class RLSarScanNode(Node):
    def __init__(self) -> None:
        super().__init__("rlsar_scan_node")
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(1.0)
        self.range_min = 0.05
        self.range_max = 6.0
        self.scan_rate_hz = 10.0

        # Match the synthetic scan geometry to the saved rlsar_scene occupancy map.
        self.obstacles: List[Rectangle] = [
            (-1.0, -0.35, 0.64, 1.79),
            (1.4, 2.05, 0.54, 1.79),
            (1.3, 2.05, -2.96, -1.81),
        ]

        self.latest_odom = None
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, "/odom", self._odom_cb, sensor_qos)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", sensor_qos)
        self.create_timer(1.0 / self.scan_rate_hz, self._publish_scan)

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg

    @staticmethod
    def _yaw_from_quaternion(w: float, x: float, y: float, z: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _ray_box_distance(px: float, py: float, dx: float, dy: float, rect: Rectangle) -> float:
        xmin, xmax, ymin, ymax = rect
        tmin = -math.inf
        tmax = math.inf

        if abs(dx) < 1e-9:
            if px < xmin or px > xmax:
                return math.inf
        else:
            tx1 = (xmin - px) / dx
            tx2 = (xmax - px) / dx
            tmin = max(tmin, min(tx1, tx2))
            tmax = min(tmax, max(tx1, tx2))

        if abs(dy) < 1e-9:
            if py < ymin or py > ymax:
                return math.inf
        else:
            ty1 = (ymin - py) / dy
            ty2 = (ymax - py) / dy
            tmin = max(tmin, min(ty1, ty2))
            tmax = min(tmax, max(ty1, ty2))

        if tmax < max(tmin, 0.0):
            return math.inf
        return tmin if tmin >= 0.0 else tmax

    def _publish_scan(self) -> None:
        if self.latest_odom is None:
            return

        pose = self.latest_odom.pose.pose
        px = pose.position.x
        py = pose.position.y
        yaw = self._yaw_from_quaternion(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        )

        beam_count = int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1
        ranges = []
        for i in range(beam_count):
            angle = yaw + self.angle_min + i * self.angle_increment
            dx = math.cos(angle)
            dy = math.sin(angle)
            distance = self.range_max
            for rect in self.obstacles:
                hit = self._ray_box_distance(px, py, dx, dy, rect)
                if self.range_min <= hit < distance:
                    distance = hit
            ranges.append(distance)

        scan = LaserScan()
        scan.header.stamp = self.latest_odom.header.stamp
        scan.header.frame_id = "base_footprint"
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.scan_rate_hz
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        self.scan_pub.publish(scan)


def main() -> None:
    rclpy.init()
    node = RLSarScanNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
