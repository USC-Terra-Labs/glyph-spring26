import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPoseRestamper(Node):
    def __init__(self) -> None:
        super().__init__("go2_initial_pose_restamper")
        self.declare_parameter("input_topic", "/initialpose")
        self.declare_parameter("output_topic", "/initialpose_amcl")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            input_topic,
            self._handle_pose,
            10,
        )

    def _handle_pose(self, msg: PoseWithCovarianceStamped) -> None:
        forwarded = PoseWithCovarianceStamped()
        forwarded.header.frame_id = msg.header.frame_id or "map"
        forwarded.header.stamp.sec = 0
        forwarded.header.stamp.nanosec = 0
        forwarded.pose = msg.pose
        self.publisher.publish(forwarded)


def main() -> None:
    rclpy.init()
    node = InitialPoseRestamper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
