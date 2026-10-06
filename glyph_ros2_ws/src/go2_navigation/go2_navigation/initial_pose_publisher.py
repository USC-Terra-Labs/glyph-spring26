import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("go2_initial_pose_publisher")
        self.declare_parameter("topic", "/initialpose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("delay_sec", 3.0)

        topic = str(self.get_parameter("topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.yaw = float(self.get_parameter("yaw").value)
        delay_sec = float(self.get_parameter("delay_sec").value)

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, topic, 10)
        self.timer = self.create_timer(delay_sec, self._publish_once)
        self.published = False

    def _publish_once(self) -> None:
        if self.published:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        half_yaw = self.yaw * 0.5
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        covariance = [0.0] * 36
        covariance[0] = 0.25
        covariance[7] = 0.25
        covariance[35] = 0.0685389
        msg.pose.covariance = covariance

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published initial pose in frame '{self.frame_id}': x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}"
        )
        self.published = True
        self.timer.cancel()


def main() -> None:
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
