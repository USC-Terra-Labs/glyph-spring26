"""
    uart_dispense_bridge connects ROS2 with the ESP over UART.

    It acts as a bidirectional bridge:
    - ROS → UART: sends flavor selections or raw commands to the ESP
    - UART → ROS: parses status messages and publishes state updates

    Core responsibilities:
    - Convert flavor selections (1/2/3) into protocol commands (A/B/C)
    - Read newline-delimited UART messages and interpret status
    - Publish dispensing state and empty state as ROS topics
    - Open the movement gate when dispensing is complete ("DONE" event)
    - Provide raw UART event stream for debugging/UI

    Key signals:
        "CUP"   → cup present → dispensing active
        "DONE"  → dispensing finished, allow robot to move
        "EMPTY" / "FLAVOR_EMPTY" → dispenser out of stock
"""

from __future__ import annotations

import re
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt8

try:
    import serial
    from serial import SerialException
except Exception:
    serial = None
    SerialException = Exception


class UartDispenseBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2_uart_dispense_bridge")

        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("flavor_selection_topic", "/flavor_selection")
        self.declare_parameter("currently_dispensing_topic", "/currently_dispensing")
        self.declare_parameter("dispense_empty_topic", "/dispense_empty")
        self.declare_parameter("uart_event_topic", "/dispense_uart_event")
        self.declare_parameter("uart_command_topic", "/dispense_uart_command")
        self.declare_parameter("movement_gate_topic", "/return_home_trigger")
        self.declare_parameter("return_home_trigger_topic", "")
        self.declare_parameter("poll_hz", 50.0)
        self.declare_parameter("write_timeout_sec", 0.2)
        self.declare_parameter("read_timeout_sec", 0.0)
        self.declare_parameter("initial_currently_dispensing", False)
        self.declare_parameter("initial_dispense_empty", False)

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.flavor_selection_topic = str(self.get_parameter("flavor_selection_topic").value)
        self.currently_dispensing_topic = str(
            self.get_parameter("currently_dispensing_topic").value
        )
        self.dispense_empty_topic = str(self.get_parameter("dispense_empty_topic").value)
        self.uart_event_topic = str(self.get_parameter("uart_event_topic").value)
        self.uart_command_topic = str(self.get_parameter("uart_command_topic").value)
        self.movement_gate_topic = str(
            self.get_parameter("movement_gate_topic").value
            or self.get_parameter("return_home_trigger_topic").value
            or "/return_home_trigger"
        )
        poll_hz = max(1.0, float(self.get_parameter("poll_hz").value))
        self.write_timeout_sec = float(self.get_parameter("write_timeout_sec").value)
        self.read_timeout_sec = float(self.get_parameter("read_timeout_sec").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.currently_dispensing_pub = self.create_publisher(
            Bool,
            self.currently_dispensing_topic,
            qos,
        )
        self.dispense_empty_pub = self.create_publisher(
            Bool,
            self.dispense_empty_topic,
            qos,
        )
        self.movement_gate_pub = self.create_publisher(
            Bool,
            self.movement_gate_topic,
            qos,
        )
        self.uart_event_pub = self.create_publisher(String, self.uart_event_topic, 10)
        self.create_subscription(
            UInt8,
            self.flavor_selection_topic,
            self._on_flavor_selection,
            qos,
        )
        self.create_subscription(
            String,
            self.uart_command_topic,
            self._on_uart_command,
            10,
        )

        self._serial: Optional[serial.Serial] = None if serial is not None else None
        self._read_buffer = bytearray()
        self._currently_dispensing = bool(
            self.get_parameter("initial_currently_dispensing").value
        )
        self._dispense_empty = bool(self.get_parameter("initial_dispense_empty").value)
        self._last_flavor_selection: Optional[int] = None

        self._connect_serial()
        self._publish_currently_dispensing(self._currently_dispensing, force=True)
        self._publish_dispense_empty(self._dispense_empty, force=True)
        self.create_timer(1.0 / poll_hz, self._poll_serial)

    _LEVELS_WATER_RE = re.compile(r"W=([0-9]+(?:\.[0-9]+)?)/([0-9]+(?:\.[0-9]+)?)", re.IGNORECASE)

    def _connect_serial(self) -> None:
        if serial is None:
            return

        try:
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.read_timeout_sec,
                write_timeout=self.write_timeout_sec,
            )
        except Exception:
            self._serial = None

    def _publish_currently_dispensing(self, state: bool, *, force: bool = False) -> None:
        if not force and self._currently_dispensing == state:
            return
        self._currently_dispensing = state
        msg = Bool()
        msg.data = state
        self.currently_dispensing_pub.publish(msg)

    def _publish_dispense_empty(self, state: bool, *, force: bool = False) -> None:
        if not force and self._dispense_empty == state:
            return
        self._dispense_empty = state
        msg = Bool()
        msg.data = state
        self.dispense_empty_pub.publish(msg)

    def _publish_uart_event(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.uart_event_pub.publish(msg)

    def _publish_movement_gate_open(self) -> None:
        msg = Bool()
        msg.data = True
        self.movement_gate_pub.publish(msg)

    def _on_flavor_selection(self, msg: UInt8) -> None:
        selection = int(msg.data)
        flavor_code = {1: "A", 2: "B", 3: "C"}.get(selection)
        if flavor_code is None:
            return
        self._write_text(flavor_code)

    def _on_uart_command(self, msg: String) -> None:
        command = str(msg.data)
        if command:
            self._write_text(command)

    def _write_text(self, payload: str) -> bool:
        if self._serial is None:
            return False
        try:
            self._serial.write(payload.encode("utf-8"))
            self._serial.flush()
            return True
        except Exception:
            self._serial = None
            return False

    def _poll_serial(self) -> None:
        if self._serial is None:
            return
        try:
            waiting = self._serial.in_waiting
            if waiting <= 0:
                return
            chunk = self._serial.read(waiting)
        except Exception:
            self._serial = None
            return
        if not chunk:
            return
        self._read_buffer.extend(chunk)
        while True:
            newline_index = self._read_buffer.find(b"\n")
            if newline_index < 0:
                return
            raw_line = bytes(self._read_buffer[:newline_index]).strip()
            del self._read_buffer[: newline_index + 1]
            if raw_line:
                self._handle_line(raw_line)

    def _handle_line(self, raw_line: bytes) -> None:
        try:
            text = raw_line.decode("utf-8", errors="replace").strip()
        except Exception:
            return
        if not text:
            return
        self._publish_uart_event(text)
        upper_text = text.upper()
        if "CUP" in upper_text:
            self._publish_currently_dispensing(True)
            return
        if "DONE" in upper_text:
            self._publish_currently_dispensing(False)
            self._publish_movement_gate_open()
            return
        if "EMPTY" in upper_text or "FLAVOR_EMPTY" in upper_text:
            self._publish_currently_dispensing(False)
            self._publish_dispense_empty(True)
            return
        if upper_text.startswith("LEVELS:"):
            self._handle_levels_line(text)
            return
        if "READY" in upper_text or "NO_SELECTION" in upper_text:
            self._publish_currently_dispensing(False)

    def _handle_levels_line(self, text: str) -> None:
        match = self._LEVELS_WATER_RE.search(text)
        if match is None:
            return
        try:
            remaining_ml = float(match.group(1))
        except ValueError:
            return
        self._publish_dispense_empty(remaining_ml <= 1.0)

    def destroy_node(self) -> bool:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UartDispenseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
