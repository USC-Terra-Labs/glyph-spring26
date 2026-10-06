import json
import math
from typing import Final

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32, String, UInt8
from tf2_ros import TransformBroadcaster
from unitree_api.msg import Request
from unitree_go.msg import LowState, SportModeState


JOINT_NAMES: Final[list[str]] = [
    "lf_hip_joint",
    "lf_upper_leg_joint",
    "lf_lower_leg_joint",
    "rf_hip_joint",
    "rf_upper_leg_joint",
    "rf_lower_leg_joint",
    "lh_hip_joint",
    "lh_upper_leg_joint",
    "lh_lower_leg_joint",
    "rh_hip_joint",
    "rh_upper_leg_joint",
    "rh_lower_leg_joint",
]

MOTOR_INDEX_ORDER: Final[list[int]] = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]

API_ID_BALANCE_STAND: Final[int] = 1002
API_ID_STOP_MOVE: Final[int] = 1003
API_ID_EULER: Final[int] = 1007
API_ID_MOVE: Final[int] = 1008
API_ID_BODY_HEIGHT: Final[int] = 1013


class Go2UnitreeBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("go2_unitree_bridge_node")

        self.declare_parameter("sport_state_topic", "lf/sportmodestate")
        self.declare_parameter("sport_state_fallback_topic", "/sportmodestate")
        self.declare_parameter("low_state_topic", "lf/lowstate")
        self.declare_parameter("low_state_fallback_topic", "/lowstate")
        self.declare_parameter("cmd_topic", "/api/sport/request")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_planar_tf", True)
        self.declare_parameter("publish_body_tf", True)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("zero_on_start", False)
        self.declare_parameter("body_motion_topic", "/body_motion")
        self.declare_parameter("body_motion_state_topic", "/body_motion_state")
        self.declare_parameter("body_motion_state_plot_topic", "/body_motion_state_plot")
        self.declare_parameter("body_motion_hz", 30.0)
        self.declare_parameter("sport_state_republish_hz", 20.0)
        self.declare_parameter("sport_state_republish_delay_sec", 0.15)
        self.declare_parameter("sport_state_hold_sec", 6.0)
        self.declare_parameter("sport_state_warn_sec", 0.5)
        self.declare_parameter("dance1_yaw_amplitude", 0.05)
        self.declare_parameter("dance1_frequency_hz", 0.20)
        self.declare_parameter("dance1_roll_amplitude", 0.01)
        self.declare_parameter("dance2_pitch_amplitude", 0.10)
        self.declare_parameter("dance2_height_amplitude", 0.040)
        self.declare_parameter("dance2_frequency_hz", 0.40)
        self.declare_parameter("dance2_height_rate_limit", 0.04)

        sport_state_topic = self.get_parameter("sport_state_topic").value
        sport_state_fallback_topic = self.get_parameter("sport_state_fallback_topic").value
        low_state_topic = self.get_parameter("low_state_topic").value
        low_state_fallback_topic = self.get_parameter("low_state_fallback_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.body_frame = self.get_parameter("body_frame").value
        self.imu_frame = self.get_parameter("imu_frame").value
        publish_tf_enabled = bool(self.get_parameter("publish_tf").value)
        self.publish_planar_tf_enabled = publish_tf_enabled and bool(self.get_parameter("publish_planar_tf").value)
        self.publish_body_tf_enabled = publish_tf_enabled and bool(self.get_parameter("publish_body_tf").value)
        self.publish_odom_enabled = bool(self.get_parameter("publish_odom").value)
        odom_topic = self.get_parameter("odom_topic").value
        self.zero_on_start = bool(self.get_parameter("zero_on_start").value)
        body_motion_topic = self.get_parameter("body_motion_topic").value
        body_motion_state_topic = self.get_parameter("body_motion_state_topic").value
        body_motion_state_plot_topic = self.get_parameter("body_motion_state_plot_topic").value
        body_motion_hz = max(2.0, float(self.get_parameter("body_motion_hz").value))
        sport_state_republish_hz = max(1.0, float(self.get_parameter("sport_state_republish_hz").value))
        self.sport_state_republish_delay_sec = max(
            0.0,
            float(self.get_parameter("sport_state_republish_delay_sec").value),
        )
        self.sport_state_hold_sec = max(0.0, float(self.get_parameter("sport_state_hold_sec").value))
        self.sport_state_warn_sec = max(0.0, float(self.get_parameter("sport_state_warn_sec").value))
        self.dance1_yaw_amplitude = float(self.get_parameter("dance1_yaw_amplitude").value)
        self.dance1_frequency_hz = float(self.get_parameter("dance1_frequency_hz").value)
        self.dance1_roll_amplitude = float(self.get_parameter("dance1_roll_amplitude").value)
        self.dance2_pitch_amplitude = float(self.get_parameter("dance2_pitch_amplitude").value)
        self.dance2_height_amplitude = float(self.get_parameter("dance2_height_amplitude").value)
        self.dance2_frequency_hz = float(self.get_parameter("dance2_frequency_hz").value)
        self.dance2_height_rate_limit = float(self.get_parameter("dance2_height_rate_limit").value)
        self._origin_x = None
        self._origin_y = None
        self._origin_yaw = None
        self._last_body_motion_mode = "stop"
        self._body_motion_baseline_sent = False
        self._last_body_motion_tick_ns: int | None = None
        self._smoothed_body_height = 0.0
        self._last_sport_state_rx_ns: int | None = None
        self._last_sport_state_snapshot: dict[str, float] | None = None
        self._sport_state_stale_warned = False

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, odom_topic, 10)
        self.imu_publisher = self.create_publisher(Imu, "/imu/data", 10)
        self.joint_state_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.sport_request_publisher = self.create_publisher(Request, cmd_topic, 10)
        self.body_motion_state_publisher = self.create_publisher(UInt8, body_motion_state_topic, 10)
        self.body_motion_state_plot_publisher = self.create_publisher(Float32, body_motion_state_plot_topic, 10)

        self.create_subscription(
            SportModeState,
            sport_state_topic,
            self._on_sport_state,
            qos_profile_sensor_data,
        )
        if sport_state_fallback_topic != sport_state_topic:
            self.create_subscription(
                SportModeState,
                sport_state_fallback_topic,
                self._on_sport_state,
                qos_profile_sensor_data,
            )
        self.create_subscription(
            LowState,
            low_state_topic,
            self._on_low_state,
            qos_profile_sensor_data,
        )
        if low_state_fallback_topic != low_state_topic:
            self.create_subscription(
                LowState,
                low_state_fallback_topic,
                self._on_low_state,
                qos_profile_sensor_data,
            )
        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(String, body_motion_topic, self._on_body_motion, 10)
        self.body_motion_timer = self.create_timer(1.0 / body_motion_hz, self._tick_body_motion)
        self.sport_state_timer = self.create_timer(
            1.0 / sport_state_republish_hz,
            self._republish_sport_state_if_stale,
        )

        self._request_id = 1
        self.get_logger().info(
            "Bridging "
            f"{sport_state_topic} ({sport_state_fallback_topic}) -> /odom,/tf and "
            f"{low_state_topic} ({low_state_fallback_topic}) -> /joint_states,/imu/data"
        )
        self.get_logger().info(f"Listening for velocity commands on {cmd_vel_topic}")
        self.get_logger().info(f"Listening for body motion commands on {body_motion_topic}")

    def _next_request_id(self) -> int:
        self._request_id += 1
        return self._request_id

    @staticmethod
    def _rpy_from_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
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

    def _publish_sport_state_snapshot(self, snapshot: dict[str, float], stamp) -> None:
        planar_qx, planar_qy, planar_qz, planar_qw = self._quaternion_from_rpy(0.0, 0.0, snapshot["yaw"])
        body_qx, body_qy, body_qz, body_qw = self._quaternion_from_rpy(
            snapshot["roll"],
            snapshot["pitch"],
            0.0,
        )

        if self.publish_odom_enabled:
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            odom.pose.pose.position.x = snapshot["x"]
            odom.pose.pose.position.y = snapshot["y"]
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = planar_qx
            odom.pose.pose.orientation.y = planar_qy
            odom.pose.pose.orientation.z = planar_qz
            odom.pose.pose.orientation.w = planar_qw
            vx_world = snapshot["vx_world"]
            vy_world = snapshot["vy_world"]
            yaw = snapshot["yaw"]
            odom.twist.twist.linear.x = math.cos(yaw) * vx_world + math.sin(yaw) * vy_world
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.z = snapshot["yaw_speed"]
            self.odom_publisher.publish(odom)

        transforms = []
        if self.publish_planar_tf_enabled:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = snapshot["x"]
            transform.transform.translation.y = snapshot["y"]
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = planar_qx
            transform.transform.rotation.y = planar_qy
            transform.transform.rotation.z = planar_qz
            transform.transform.rotation.w = planar_qw
            transforms.append(transform)

        if self.publish_body_tf_enabled:
            body_transform = TransformStamped()
            body_transform.header.stamp = stamp
            body_transform.header.frame_id = self.base_frame
            body_transform.child_frame_id = self.body_frame
            body_transform.transform.translation.x = 0.0
            body_transform.transform.translation.y = 0.0
            body_transform.transform.translation.z = snapshot["body_z"]
            body_transform.transform.rotation.x = body_qx
            body_transform.transform.rotation.y = body_qy
            body_transform.transform.rotation.z = body_qz
            body_transform.transform.rotation.w = body_qw
            transforms.append(body_transform)

        if transforms:
            self.tf_broadcaster.sendTransform(transforms)

    def _republish_sport_state_if_stale(self) -> None:
        if self._last_sport_state_snapshot is None or self._last_sport_state_rx_ns is None:
            return

        now = self.get_clock().now()
        age_sec = (now.nanoseconds - self._last_sport_state_rx_ns) / 1e9
        if age_sec < self.sport_state_republish_delay_sec:
            return

        if self.sport_state_hold_sec > 0.0 and age_sec > self.sport_state_hold_sec:
            if not self._sport_state_stale_warned:
                self.get_logger().warn(
                    f"SportModeState stalled for {age_sec:.2f}s; holding odom/TF stopped after {self.sport_state_hold_sec:.2f}s"
                )
                self._sport_state_stale_warned = True
            return

        if age_sec >= self.sport_state_warn_sec and not self._sport_state_stale_warned:
            self.get_logger().warn(
                f"SportModeState stalled for {age_sec:.2f}s; republishing last odom/TF sample to bridge a transient dropout"
            )
            self._sport_state_stale_warned = True

        self._publish_sport_state_snapshot(self._last_sport_state_snapshot, now.to_msg())

    def _on_sport_state(self, msg: SportModeState) -> None:
        now = self.get_clock().now()
        stamp = now.to_msg()
        # Unitree IMU quaternions are published as w,x,y,z; convert to ROS xyzw.
        full_qw = float(msg.imu_state.quaternion[0])
        full_qx = float(msg.imu_state.quaternion[1])
        full_qy = float(msg.imu_state.quaternion[2])
        full_qz = float(msg.imu_state.quaternion[3])
        roll, pitch, yaw = self._rpy_from_quaternion(full_qx, full_qy, full_qz, full_qw)
        if self.zero_on_start and self._origin_x is None:
            self._origin_x = float(msg.position[0])
            self._origin_y = float(msg.position[1])
            self._origin_yaw = yaw
            self.get_logger().info(
                f"Zeroed odom origin at x={self._origin_x:.3f}, y={self._origin_y:.3f}, yaw={self._origin_yaw:.3f}"
            )

        x = float(msg.position[0])
        y = float(msg.position[1])
        if self._origin_x is not None and self._origin_y is not None and self._origin_yaw is not None:
            dx = x - self._origin_x
            dy = y - self._origin_y
            cos_yaw = math.cos(-self._origin_yaw)
            sin_yaw = math.sin(-self._origin_yaw)
            x = cos_yaw * dx - sin_yaw * dy
            y = sin_yaw * dx + cos_yaw * dy
            yaw = math.atan2(math.sin(yaw - self._origin_yaw), math.cos(yaw - self._origin_yaw))

        self._last_sport_state_snapshot = {
            "x": x,
            "y": y,
            "yaw": yaw,
            "roll": roll,
            "pitch": pitch,
            "body_z": float(msg.position[2]),
            "vx_world": float(msg.velocity[0]),
            "vy_world": float(msg.velocity[1]),
            "yaw_speed": float(msg.yaw_speed),
        }
        self._last_sport_state_rx_ns = now.nanoseconds
        if self._sport_state_stale_warned:
            self.get_logger().info("SportModeState updates resumed")
            self._sport_state_stale_warned = False

        self._publish_sport_state_snapshot(self._last_sport_state_snapshot, stamp)

    def _on_low_state(self, msg: LowState) -> None:
        stamp = self.get_clock().now().to_msg()

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = JOINT_NAMES
        joint_state.position = [float(msg.motor_state[index].q) for index in MOTOR_INDEX_ORDER]
        joint_state.velocity = [float(msg.motor_state[index].dq) for index in MOTOR_INDEX_ORDER]
        self.joint_state_publisher.publish(joint_state)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame
        imu.orientation.w = float(msg.imu_state.quaternion[0])
        imu.orientation.x = float(msg.imu_state.quaternion[1])
        imu.orientation.y = float(msg.imu_state.quaternion[2])
        imu.orientation.z = float(msg.imu_state.quaternion[3])
        imu.angular_velocity.x = float(msg.imu_state.gyroscope[0])
        imu.angular_velocity.y = float(msg.imu_state.gyroscope[1])
        imu.angular_velocity.z = float(msg.imu_state.gyroscope[2])
        imu.linear_acceleration.x = float(msg.imu_state.accelerometer[0])
        imu.linear_acceleration.y = float(msg.imu_state.accelerometer[1])
        imu.linear_acceleration.z = float(msg.imu_state.accelerometer[2])
        imu.orientation_covariance[0] = 0.0025
        imu.orientation_covariance[4] = 0.0025
        imu.orientation_covariance[8] = 0.0025
        imu.angular_velocity_covariance[0] = 0.000001
        imu.angular_velocity_covariance[4] = 0.000001
        imu.angular_velocity_covariance[8] = 0.000001
        imu.linear_acceleration_covariance[0] = 0.0001
        imu.linear_acceleration_covariance[4] = 0.0001
        imu.linear_acceleration_covariance[8] = 0.0001
        self.imu_publisher.publish(imu)

    def _on_cmd_vel(self, msg: Twist) -> None:
        request = Request()
        request.header.identity.id = self._next_request_id()

        moving = any(
            abs(value) > 1e-4
            for value in (msg.linear.x, msg.linear.y, msg.angular.z)
        )

        if moving:
            request.header.identity.api_id = API_ID_MOVE
            request.parameter = json.dumps(
                {
                    "x": float(msg.linear.x),
                    "y": float(msg.linear.y),
                    "z": float(msg.angular.z),
                }
            )
        else:
            request.header.identity.api_id = API_ID_STOP_MOVE

        self.sport_request_publisher.publish(request)

    def _on_body_motion(self, msg: String) -> None:
        mode = msg.data.strip().lower()
        if mode == self._last_body_motion_mode:
            return

        if mode == "stop":
            self._last_body_motion_mode = mode
            self._body_motion_baseline_sent = False
            return

        if mode not in {"dance1", "dance2"}:
            self.get_logger().warn(f"Ignoring unsupported body motion mode '{msg.data}'")
            return

        self._last_body_motion_mode = mode
        self._body_motion_baseline_sent = False

    def _publish_balance_stand(self) -> None:
        request = Request()
        request.header.identity.id = self._next_request_id()
        request.header.identity.api_id = API_ID_BALANCE_STAND
        self.sport_request_publisher.publish(request)

    def _publish_euler(self, roll: float, pitch: float, yaw: float) -> None:
        request = Request()
        request.header.identity.id = self._next_request_id()
        request.header.identity.api_id = API_ID_EULER
        request.parameter = json.dumps(
            {
                "x": float(roll),
                "y": float(pitch),
                "z": float(yaw),
            }
        )
        self.sport_request_publisher.publish(request)

    def _publish_body_height(self, height: float) -> None:
        request = Request()
        request.header.identity.id = self._next_request_id()
        request.header.identity.api_id = API_ID_BODY_HEIGHT
        request.parameter = json.dumps({"data": float(height)})
        self.sport_request_publisher.publish(request)

    def _send_body_motion_baseline(self) -> None:
        self._smoothed_body_height = 0.0
        self._publish_euler(0.0, 0.0, 0.0)
        self._publish_body_height(0.0)
        self._publish_balance_stand()
        self._body_motion_baseline_sent = True

    def _tick_body_motion(self) -> None:
        mode = self._last_body_motion_mode
        state = UInt8()
        state_plot = Float32()
        now_ns = self.get_clock().now().nanoseconds
        dt_sec = 1.0 / 30.0
        if self._last_body_motion_tick_ns is not None:
            dt_sec = max(1e-3, (now_ns - self._last_body_motion_tick_ns) / 1e9)
        self._last_body_motion_tick_ns = now_ns
        if mode == "stop":
            if not self._body_motion_baseline_sent:
                self._send_body_motion_baseline()
            state.data = 0
            state_plot.data = 0.0
            self.body_motion_state_publisher.publish(state)
            self.body_motion_state_plot_publisher.publish(state_plot)
            return

        now_sec = now_ns / 1e9
        if mode == "dance1":
            phase = 2.0 * math.pi * self.dance1_frequency_hz * now_sec
            yaw = self.dance1_yaw_amplitude * math.sin(phase)
            roll = self.dance1_roll_amplitude * math.sin(phase + math.pi / 2.0)
            self._smoothed_body_height = 0.0
            self._publish_euler(roll, 0.0, yaw)
            self._publish_body_height(0.0)
            state.data = 1
            state_plot.data = 1.0
        elif mode == "dance2":
            phase = 2.0 * math.pi * self.dance2_frequency_hz * now_sec
            # Continuous smooth lower-and-rise motion, staying below nominal stand height.
            target_height = -0.5 * self.dance2_height_amplitude + 0.5 * self.dance2_height_amplitude * math.sin(phase)
            max_step = self.dance2_height_rate_limit * dt_sec
            delta = max(-max_step, min(max_step, target_height - self._smoothed_body_height))
            self._smoothed_body_height += delta
            self._publish_euler(0.0, 0.0, 0.0)
            self._publish_body_height(self._smoothed_body_height)
            state.data = 2
            state_plot.data = 2.0
        self._body_motion_baseline_sent = False
        self.body_motion_state_publisher.publish(state)
        self.body_motion_state_plot_publisher.publish(state_plot)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Go2UnitreeBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
