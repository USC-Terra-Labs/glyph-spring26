import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState


class VerifyRLSarSim(Node):
    def __init__(self) -> None:
        super().__init__("verify_rlsar_sim")
        self.odom_msg = None
        self.joint_state_msg = None
        self.imu_msg = None

        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom_msg = msg

    def _joint_cb(self, msg: JointState) -> None:
        self.joint_state_msg = msg

    def _imu_cb(self, msg: Imu) -> None:
        self.imu_msg = msg

    def wait_for_topics(self, timeout_sec: float = 20.0) -> None:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom_msg and self.joint_state_msg and self.imu_msg:
                return
        raise RuntimeError("Timed out waiting for /odom, /joint_states, and /imu/data")

    def publish_cmd_vel(self, linear_x: float, duration_sec: float = 4.0, rate_hz: float = 10.0) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        interval = 1.0 / rate_hz
        deadline = time.time() + duration_sec
        while time.time() < deadline:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(interval)


def main() -> int:
    rclpy.init()
    node = VerifyRLSarSim()
    try:
        node.get_logger().info("Waiting for rl_sar MuJoCo topics")
        node.wait_for_topics()

        start = node.odom_msg.pose.pose.position
        node.get_logger().info(
            f"Initial pose: x={start.x:.3f}, y={start.y:.3f}, z={start.z:.3f}"
        )

        node.get_logger().info("Publishing /cmd_vel forward command")
        node.publish_cmd_vel(0.2)

        settle_deadline = time.time() + 1.0
        while time.time() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        end = node.odom_msg.pose.pose.position
        dx = end.x - start.x
        dy = end.y - start.y
        dz = end.z - start.z
        distance = math.hypot(dx, dy)

        node.get_logger().info(
            f"Final pose: x={end.x:.3f}, y={end.y:.3f}, z={end.z:.3f}; "
            f"delta=({dx:.3f}, {dy:.3f}, {dz:.3f}), planar={distance:.3f} m"
        )

        if distance < 0.02:
            node.get_logger().error("FAIL: robot did not translate enough under /cmd_vel")
            return 1

        node.get_logger().info("PASS: rl_sar MuJoCo sim is publishing state and moving under /cmd_vel")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
