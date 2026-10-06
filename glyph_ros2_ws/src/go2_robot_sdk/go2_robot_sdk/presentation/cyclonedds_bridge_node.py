import os
import subprocess
import sys
import tempfile
from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import Imu as SensorImu  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402

SDK_ROOT = Path(os.getenv("GO2_PYTHON_SDK_DIR", "/home/ming/go2_python_sdk"))
SDK_SITE_PACKAGES = SDK_ROOT / ".venv" / "lib" / "python3.10" / "site-packages"
SDK_IDL_ROOT = SDK_ROOT / "communicator" / "idl"

for path in (SDK_SITE_PACKAGES, SDK_IDL_ROOT):
    path_str = str(path)
    if path.exists() and path_str not in sys.path:
        sys.path.insert(0, path_str)

from cyclonedds.core import Listener  # type: ignore  # noqa: E402
from cyclonedds.domain import DomainParticipant  # type: ignore  # noqa: E402
from cyclonedds.sub import DataReader  # type: ignore  # noqa: E402
from cyclonedds.topic import Topic  # type: ignore  # noqa: E402
from unitree_go.msg.dds_ import LowState_ as DDSLowState  # type: ignore  # noqa: E402
from unitree_go.msg.dds_ import SportModeState_ as DDSSportModeState  # type: ignore  # noqa: E402

from geometry_msgs.msg import PoseStamped  # noqa: E402
from go2_interfaces.msg import BmsState, IMU, LowState, MotorState  # noqa: E402


class CycloneDDSBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("cyclonedds_bridge_node")

        self.declare_parameter("robot_ip", os.getenv("ROBOT_IP", "192.168.123.161"))
        self.declare_parameter("network_interface", os.getenv("GO2_DDS_INTERFACE", "enP8p1s0"))

        robot_ip = self.get_parameter("robot_ip").value
        configured_interface = self.get_parameter("network_interface").value
        network_interface = self._resolve_network_interface(robot_ip, configured_interface)
        self._configure_cyclonedds(robot_ip, network_interface)

        self.lowstate_publisher = self.create_publisher(LowState, "lowstate", 10)
        self.robot_pose_publisher = self.create_publisher(PoseStamped, "/utlidar/robot_pose", 10)
        self.joint_states_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.imu_data_publisher = self.create_publisher(SensorImu, "/imu/data", 10)
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.participant = DomainParticipant(domain_id=0)
        self._readers = [
            DataReader(
                self.participant,
                Topic(self.participant, "rt/lowstate", DDSLowState),
                listener=self._LowStateListener(self),
            ),
            DataReader(
                self.participant,
                Topic(self.participant, "rt/sportmodestate", DDSSportModeState),
                listener=self._SportStateListener(self),
            ),
        ]

        self.get_logger().info(
            f"Bridge subscribed to DDS topics via {network_interface} peer {robot_ip}"
        )

    def _resolve_network_interface(self, robot_ip: str, configured_interface: str) -> str:
        if configured_interface and self._interface_is_up(configured_interface):
            return configured_interface

        routed_interface = self._lookup_route_interface(robot_ip)
        if routed_interface:
            self.get_logger().warn(
                f"Configured DDS interface '{configured_interface}' is unavailable, using routed interface '{routed_interface}' instead."
            )
            return routed_interface

        active_interface = self._first_active_interface()
        if active_interface:
            self.get_logger().warn(
                f"Configured DDS interface '{configured_interface}' is unavailable, using active interface '{active_interface}' instead."
            )
            return active_interface

        return configured_interface

    def _interface_is_up(self, interface_name: str) -> bool:
        try:
            output = subprocess.check_output(
                ["ip", "-br", "addr", "show", interface_name],
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
        except (subprocess.CalledProcessError, FileNotFoundError):
            return False

        if not output:
            return False

        fields = output.split()
        return len(fields) > 1 and fields[1] not in {"DOWN", "DORMANT"}

    def _lookup_route_interface(self, robot_ip: str) -> str | None:
        if not robot_ip:
            return None

        try:
            output = subprocess.check_output(
                ["ip", "route", "get", robot_ip],
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
        except (subprocess.CalledProcessError, FileNotFoundError):
            return None

        parts = output.split()
        if "dev" not in parts:
            return None

        dev_index = parts.index("dev") + 1
        return parts[dev_index] if dev_index < len(parts) else None

    def _first_active_interface(self) -> str | None:
        try:
            output = subprocess.check_output(
                ["ip", "-br", "addr"],
                text=True,
                stderr=subprocess.DEVNULL,
            )
        except (subprocess.CalledProcessError, FileNotFoundError):
            return None

        for line in output.splitlines():
            fields = line.split()
            if len(fields) < 2:
                continue

            interface_name, state = fields[0], fields[1]
            if interface_name == "lo" or state in {"DOWN", "DORMANT"}:
                continue
            return interface_name

        return None

    def _configure_cyclonedds(self, robot_ip: str, network_interface: str) -> None:
        xml = f"""<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain Id="0">
    <General>
      <Interfaces>
        <NetworkInterface name="{network_interface}" priority="default" />
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer address="{robot_ip}" />
      </Peers>
      <EnableTopicDiscoveryEndpoints>true</EnableTopicDiscoveryEndpoints>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
    <SharedMemory>
      <Enable>false</Enable>
    </SharedMemory>
  </Domain>
</CycloneDDS>
"""
        with tempfile.NamedTemporaryFile("w", delete=False, suffix=".xml") as config_file:
            config_file.write(xml)
            self._cyclonedds_config_path = config_file.name

        os.environ["CYCLONEDDS_URI"] = f"file://{self._cyclonedds_config_path}"

    class _LowStateListener(Listener):
        def __init__(self, node: "CycloneDDSBridgeNode") -> None:
            super().__init__()
            self.node = node

        def on_data_available(self, reader) -> None:
            for sample in reader.take(N=10):
                if sample.sample_info.valid_data:
                    self.node.lowstate_publisher.publish(self.node._convert_lowstate(sample))
                    self.node.joint_states_publisher.publish(self.node._convert_joint_state(sample))
                    self.node.imu_data_publisher.publish(self.node._convert_sensor_imu(sample))

    class _SportStateListener(Listener):
        def __init__(self, node: "CycloneDDSBridgeNode") -> None:
            super().__init__()
            self.node = node

        def on_data_available(self, reader) -> None:
            for sample in reader.take(N=10):
                if sample.sample_info.valid_data:
                    pose_msg = self.node._convert_sport_state_pose(sample)
                    self.node.robot_pose_publisher.publish(pose_msg)
                    self.node.odom_publisher.publish(self.node._convert_odometry(pose_msg))
                    self.node.tf_broadcaster.sendTransform(self.node._convert_odom_tf(pose_msg))

    def _convert_lowstate(self, sample: DDSLowState) -> LowState:
        msg = LowState()
        msg.head = list(sample.head)
        msg.level_flag = sample.level_flag
        msg.frame_reserve = sample.frame_reserve
        msg.sn = list(sample.sn)
        msg.version = list(sample.version)
        msg.bandwidth = sample.bandwidth
        msg.imu_state = self._convert_imu(sample.imu_state)
        msg.motor_state = [self._convert_motor_state(motor) for motor in sample.motor_state]
        msg.bms_state = self._convert_bms_state(sample.bms_state)
        msg.foot_force = list(sample.foot_force)
        msg.foot_force_est = list(sample.foot_force_est)
        msg.tick = sample.tick
        msg.wireless_remote = list(sample.wireless_remote)
        msg.bit_flag = sample.bit_flag
        msg.adc_reel = sample.adc_reel
        msg.temperature_ntc1 = sample.temperature_ntc1
        msg.temperature_ntc2 = sample.temperature_ntc2
        msg.power_v = sample.power_v
        msg.power_a = sample.power_a
        msg.fan_frequency = list(sample.fan_frequency)
        msg.reserve = sample.reserve
        msg.crc = sample.crc
        return msg

    def _convert_imu(self, sample) -> IMU:
        msg = IMU()
        msg.quaternion = list(sample.quaternion)
        msg.gyroscope = list(sample.gyroscope)
        msg.accelerometer = list(sample.accelerometer)
        msg.rpy = list(sample.rpy)
        msg.temperature = sample.temperature
        return msg

    def _convert_motor_state(self, sample) -> MotorState:
        msg = MotorState()
        msg.mode = sample.mode
        msg.q = sample.q
        msg.dq = sample.dq
        msg.ddq = sample.ddq
        msg.tau_est = sample.tau_est
        msg.q_raw = sample.q_raw
        msg.dq_raw = sample.dq_raw
        msg.ddq_raw = sample.ddq_raw
        msg.temperature = sample.temperature
        msg.lost = sample.lost
        msg.reserve = list(sample.reserve)
        return msg

    def _convert_bms_state(self, sample) -> BmsState:
        msg = BmsState()
        msg.version_high = sample.version_high
        msg.version_low = sample.version_low
        msg.status = sample.status
        msg.soc = sample.soc
        msg.current = sample.current
        msg.cycle = sample.cycle
        msg.bq_ntc = list(sample.bq_ntc)
        msg.mcu_ntc = list(sample.mcu_ntc)
        msg.cell_vol = list(sample.cell_vol)
        return msg

    def _convert_sport_state_pose(self, sample: DDSSportModeState) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = float(sample.position[0])
        msg.pose.position.y = float(sample.position[1])
        msg.pose.position.z = float(sample.position[2])
        msg.pose.orientation.x = float(sample.imu_state.quaternion[0])
        msg.pose.orientation.y = float(sample.imu_state.quaternion[1])
        msg.pose.orientation.z = float(sample.imu_state.quaternion[2])
        msg.pose.orientation.w = float(sample.imu_state.quaternion[3])
        return msg

    def _convert_joint_state(self, sample: DDSLowState) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint',
            'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint',
            'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint',
            'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint',
        ]
        msg.position = [
            float(sample.motor_state[3].q), float(sample.motor_state[4].q), float(sample.motor_state[5].q),
            float(sample.motor_state[0].q), float(sample.motor_state[1].q), float(sample.motor_state[2].q),
            float(sample.motor_state[9].q), float(sample.motor_state[10].q), float(sample.motor_state[11].q),
            float(sample.motor_state[6].q), float(sample.motor_state[7].q), float(sample.motor_state[8].q),
        ]
        return msg

    def _convert_sensor_imu(self, sample: DDSLowState) -> SensorImu:
        msg = SensorImu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        msg.orientation.x = float(sample.imu_state.quaternion[0])
        msg.orientation.y = float(sample.imu_state.quaternion[1])
        msg.orientation.z = float(sample.imu_state.quaternion[2])
        msg.orientation.w = float(sample.imu_state.quaternion[3])
        msg.angular_velocity.x = float(sample.imu_state.gyroscope[0])
        msg.angular_velocity.y = float(sample.imu_state.gyroscope[1])
        msg.angular_velocity.z = float(sample.imu_state.gyroscope[2])
        msg.linear_acceleration.x = float(sample.imu_state.accelerometer[0])
        msg.linear_acceleration.y = float(sample.imu_state.accelerometer[1])
        msg.linear_acceleration.z = float(sample.imu_state.accelerometer[2])
        msg.orientation_covariance[0] = 0.0025
        msg.orientation_covariance[4] = 0.0025
        msg.orientation_covariance[8] = 0.0025
        msg.angular_velocity_covariance[0] = 0.000001
        msg.angular_velocity_covariance[4] = 0.000001
        msg.angular_velocity_covariance[8] = 0.000001
        msg.linear_acceleration_covariance[0] = 0.0001
        msg.linear_acceleration_covariance[4] = 0.0001
        msg.linear_acceleration_covariance[8] = 0.0001
        return msg

    def _convert_odometry(self, pose_msg: PoseStamped) -> Odometry:
        msg = Odometry()
        msg.header = pose_msg.header
        msg.child_frame_id = "base_link"
        msg.pose.pose = pose_msg.pose
        return msg

    def _convert_odom_tf(self, pose_msg: PoseStamped) -> TransformStamped:
        msg = TransformStamped()
        msg.header = pose_msg.header
        msg.child_frame_id = "base_link"
        msg.transform.translation.x = pose_msg.pose.position.x
        msg.transform.translation.y = pose_msg.pose.position.y
        msg.transform.translation.z = pose_msg.pose.position.z
        msg.transform.rotation = pose_msg.pose.orientation
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CycloneDDSBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
