import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty


class SimFallRecovery(Node):
    def __init__(self) -> None:
        super().__init__("go2_sim_fall_recovery")

        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("target_topic", "/target_location")
        self.declare_parameter("reset_service", "/go2_rlsar_reset")
        self.declare_parameter("tip_angle_rad", 1.0)
        self.declare_parameter("tip_duration_sec", 0.35)
        self.declare_parameter("recovery_delay_sec", 4.0)

        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        target_topic = self.get_parameter("target_topic").get_parameter_value().string_value
        reset_service = self.get_parameter("reset_service").get_parameter_value().string_value
        self._tip_angle_rad = self.get_parameter("tip_angle_rad").get_parameter_value().double_value
        self._tip_duration_sec = self.get_parameter("tip_duration_sec").get_parameter_value().double_value
        self._recovery_delay_sec = self.get_parameter("recovery_delay_sec").get_parameter_value().double_value

        target_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._reset_client = self.create_client(Empty, reset_service)
        self._target_pub = self.create_publisher(PoseStamped, target_topic, target_qos)
        self.create_subscription(PoseStamped, target_topic, self._target_callback, target_qos)
        self.create_subscription(Imu, imu_topic, self._imu_callback, imu_qos)

        self._last_target = None
        self._tip_start = None
        self._recovering = False

        self.get_logger().info("Sim fall recovery active")

    def _target_callback(self, msg: PoseStamped) -> None:
        self._last_target = msg

    def _imu_callback(self, msg: Imu) -> None:
        if self._recovering:
            return

        roll, pitch = self._quaternion_to_rpy(
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        )
        tipped = abs(roll) >= self._tip_angle_rad or abs(pitch) >= self._tip_angle_rad
        now = time.monotonic()

        if tipped:
            if self._tip_start is None:
                self._tip_start = now
            elif now - self._tip_start >= self._tip_duration_sec:
                self._recovering = True
                self._tip_start = None
                self.get_logger().warn("Robot appears tipped; triggering sim recovery")
                threading.Thread(target=self._recover_sequence, daemon=True).start()
        else:
            self._tip_start = None

    def _recover_sequence(self) -> None:
        try:
            if not self._reset_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("Reset service unavailable")
                return

            future = self._reset_client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() is None:
                self.get_logger().error("Reset service call failed")
                return

            time.sleep(self._recovery_delay_sec)
            if self._last_target is not None:
                republished = PoseStamped()
                republished.header = self._last_target.header
                republished.header.stamp = self.get_clock().now().to_msg()
                republished.pose = self._last_target.pose
                self._target_pub.publish(republished)
                self.get_logger().info("Republished last target after recovery")
        finally:
            self._recovering = False

    @staticmethod
    def _quaternion_to_rpy(w: float, x: float, y: float, z: float):
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        return roll, pitch


def main() -> None:
    rclpy.init()
    node = SimFallRecovery()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
