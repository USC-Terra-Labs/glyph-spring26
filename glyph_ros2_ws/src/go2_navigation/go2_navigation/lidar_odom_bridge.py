"""
    lidar_odom_bridge adapts LiDAR odometry into the Nav2-style planar odom model.

    It takes full 6-DoF odometry (including roll, pitch, z) and splits it into:
    - A planar /odom output (x, y, yaw only) suitable for Nav2
    - A secondary TF (base_footprint -> base_link) that preserves body tilt and height
    
     Key behaviors:
    - Optionally zeroes the initial pose to redefine origin
    - Smooths XY position and yaw independently to reduce noise
    - Removes roll/pitch from the main odometry (Nav2 assumes planar motion)
    - Publishes consistent TF transforms for both planar motion and body pose

    Output structure:
        odom -> base_footprint   (planar motion: x, y, yaw)
        base_footprint -> base_link  (roll, pitch, z height)

    It is only used in mapping mode when use_lidar_odom:=true in ros2_ws/src/go2_navigation/launch/go2_nav_robot.launch.py 
    (false by default, but recommended because performance is slightly better than go2's raw odom)
"""

import math  # Used for trigonometry, smoothing, and angle wrapping

import rclpy  # ROS2 Python client library
from geometry_msgs.msg import TransformStamped  # TF transform message
from nav_msgs.msg import Odometry  # Odometry message
from rclpy.node import Node  # Base class for ROS2 nodes
from tf2_ros import TransformBroadcaster  # For publishing TF transforms


class LidarOdomBridge(Node):
    def __init__(self) -> None:
        # Initialize node
        super().__init__("go2_lidar_odom_bridge")

        # Declare configurable parameters
        self.declare_parameter("input_topic", "/utlidar/robot_odom")
        self.declare_parameter("output_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("use_current_time", True)
        self.declare_parameter("smooth_xy_alpha", 0.25)
        self.declare_parameter("smooth_yaw_alpha", 0.2)
        self.declare_parameter("zero_on_start", False)

        # Retrieve parameter values
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.body_frame = str(self.get_parameter("body_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.use_current_time = bool(self.get_parameter("use_current_time").value)
        self.smooth_xy_alpha = float(self.get_parameter("smooth_xy_alpha").value)
        self.smooth_yaw_alpha = float(self.get_parameter("smooth_yaw_alpha").value)
        self.zero_on_start = bool(self.get_parameter("zero_on_start").value)

        # Publisher for planar odometry
        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)

        # TF broadcaster for odom->base and base->body transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to incoming LiDAR odometry
        self.sub = self.create_subscription(Odometry, input_topic, self._on_odom, 10)

        # Internal smoothing state
        self._smoothed_x = None
        self._smoothed_y = None
        self._smoothed_yaw = None

        # Optional origin reset storage
        self._origin_x = None
        self._origin_y = None
        self._origin_yaw = None

    @staticmethod
    def _rpy_from_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
        """
        Convert quaternion to roll, pitch, yaw (Euler angles).
        """
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        """
        Convert roll, pitch, yaw back into quaternion form.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return x, y, z, w

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """
        Normalize angle to [-pi, pi].
        """
        return math.atan2(math.sin(angle), math.cos(angle))

    def _smooth_value(self, previous: float | None, current: float, alpha: float) -> float:
        """
        Exponential smoothing for scalar values.
        """
        if previous is None:
            return current
        return previous + alpha * (current - previous)

    def _smooth_yaw(self, current_yaw: float) -> float:
        """
        Smooth yaw while handling wrap-around correctly.
        """
        if self._smoothed_yaw is None:
            self._smoothed_yaw = current_yaw
            return current_yaw
        delta = self._wrap_angle(current_yaw - self._smoothed_yaw)
        self._smoothed_yaw = self._wrap_angle(self._smoothed_yaw + self.smooth_yaw_alpha * delta)
        return self._smoothed_yaw

    def _on_odom(self, msg: Odometry) -> None:
        """
        Main callback: converts incoming 3D odometry into planar odom + TF.
        """
        # Extract orientation
        q = msg.pose.pose.orientation
        roll, pitch, yaw = self._rpy_from_quaternion(q.x, q.y, q.z, q.w)

        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Optionally set first reading as origin
        if self.zero_on_start and self._origin_x is None:
            self._origin_x = x
            self._origin_y = y
            self._origin_yaw = yaw
            self.get_logger().info(
                f"Zeroed lidar odom origin at x={self._origin_x:.3f}, y={self._origin_y:.3f}, yaw={self._origin_yaw:.3f}"
            )

        # Apply origin shift if enabled
        if self._origin_x is not None and self._origin_y is not None and self._origin_yaw is not None:
            dx = x - self._origin_x
            dy = y - self._origin_y

            # Rotate into new frame
            cos_yaw = math.cos(-self._origin_yaw)
            sin_yaw = math.sin(-self._origin_yaw)
            x = cos_yaw * dx - sin_yaw * dy
            y = sin_yaw * dx + cos_yaw * dy
            yaw = self._wrap_angle(yaw - self._origin_yaw)

        # Smooth position and yaw
        smoothed_x = self._smooth_value(self._smoothed_x, x, self.smooth_xy_alpha)
        smoothed_y = self._smooth_value(self._smoothed_y, y, self.smooth_xy_alpha)
        smoothed_yaw = self._smooth_yaw(yaw)

        # Store smoothed values
        self._smoothed_x = smoothed_x
        self._smoothed_y = smoothed_y

        # Build planar quaternion (yaw only)
        planar_qx, planar_qy, planar_qz, planar_qw = self._quaternion_from_rpy(0.0, 0.0, smoothed_yaw)

        # Build body quaternion (roll + pitch only)
        body_qx, body_qy, body_qz, body_qw = self._quaternion_from_rpy(roll, pitch, 0.0)

        # Choose timestamp source
        stamp = self.get_clock().now().to_msg() if self.use_current_time else msg.header.stamp

        # Construct planar odometry message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = smoothed_x
        odom.pose.pose.position.y = smoothed_y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = planar_qx
        odom.pose.pose.orientation.y = planar_qy
        odom.pose.pose.orientation.z = planar_qz
        odom.pose.pose.orientation.w = planar_qw

        # Pass through velocity unchanged
        odom.twist.twist = msg.twist.twist

        self.odom_pub.publish(odom)

        # Optionally publish TF transforms
        if not self.publish_tf:
            return

        # odom -> base_footprint (planar)
        planar_tf = TransformStamped()
        planar_tf.header.stamp = stamp
        planar_tf.header.frame_id = self.odom_frame
        planar_tf.child_frame_id = self.base_frame
        planar_tf.transform.translation.x = smoothed_x
        planar_tf.transform.translation.y = smoothed_y
        planar_tf.transform.translation.z = 0.0
        planar_tf.transform.rotation.x = planar_qx
        planar_tf.transform.rotation.y = planar_qy
        planar_tf.transform.rotation.z = planar_qz
        planar_tf.transform.rotation.w = planar_qw

        # base_footprint -> base_link (body tilt + height)
        body_tf = TransformStamped()
        body_tf.header.stamp = stamp
        body_tf.header.frame_id = self.base_frame
        body_tf.child_frame_id = self.body_frame
        body_tf.transform.translation.x = 0.0
        body_tf.transform.translation.y = 0.0
        body_tf.transform.translation.z = msg.pose.pose.position.z
        body_tf.transform.rotation.x = body_qx
        body_tf.transform.rotation.y = body_qy
        body_tf.transform.rotation.z = body_qz
        body_tf.transform.rotation.w = body_qw

        # Broadcast both transforms together
        self.tf_broadcaster.sendTransform([planar_tf, body_tf])


def main(args=None) -> None:
    # Initialize ROS2
    rclpy.init(args=args)

    # Create node
    node = LidarOdomBridge()

    try:
        # Spin to process odometry
        rclpy.spin(node)
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
