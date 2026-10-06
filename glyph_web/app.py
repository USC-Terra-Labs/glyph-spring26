import asyncio
import importlib.util
import json
import os
from pathlib import Path
import re
import threading
import time
from math import atan2, cos, sin
from typing import Any, Dict, Optional

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
import uvicorn

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Empty, String, UInt8
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

# Environment-configurable ROS topics and runtime behavior
TARGET_TOPIC = os.environ.get("TARGET_TOPIC", "/target_location")
HOME_TARGET_TOPIC = os.environ.get("HOME_TARGET_TOPIC", "/return_home_target_location")
# Legacy + current movement gate topics (both supported)
LEGACY_RETURN_HOME_TOPIC = os.environ.get("RETURN_HOME_TOPIC", "/return_home_trigger")
MOVEMENT_GATE_TOPIC = os.environ.get("MOVEMENT_GATE_TOPIC", LEGACY_RETURN_HOME_TOPIC)

SET_HOME_TOPIC = os.environ.get("SET_HOME_TOPIC", "/set_home_here")

# Dispenser + UART topics
FLAVOR_SELECTION_TOPIC = os.environ.get("FLAVOR_SELECTION_TOPIC", "/flavor_selection")
CURRENTLY_DISPENSING_TOPIC = os.environ.get("CURRENTLY_DISPENSING_TOPIC", "/currently_dispensing")
DISPENSE_EMPTY_TOPIC = os.environ.get("DISPENSE_EMPTY_TOPIC", "/dispense_empty")
DISPENSE_UART_EVENT_TOPIC = os.environ.get("DISPENSE_UART_EVENT_TOPIC", "/dispense_uart_event")
DISPENSE_UART_COMMAND_TOPIC = os.environ.get("DISPENSE_UART_COMMAND_TOPIC", "/dispense_uart_command")

# Queue / navigation control
QUEUE_CANCEL_TOPIC = os.environ.get("QUEUE_CANCEL_TOPIC", "/behavior_supervisor_cancel_goal")
QUEUE_TOPIC = os.environ.get("QUEUE_TOPIC", "/behavior_supervisor_queue")

FRAME_ID = os.environ.get("FRAME_ID", "odom")

# Map + odometry sources
MAP_TOPIC = os.environ.get("MAP_TOPIC", "/map")
ODOM_TOPIC = os.environ.get("ODOM_TOPIC", "/odom")

# HTTP server config
HTTP_HOST = os.environ.get("HOST", "0.0.0.0")
HTTP_PORT = int(os.environ.get("PORT", "8000"))

# Landmark persistence
LANDMARKS_PATH = Path(
    os.environ.get("LANDMARKS_PATH", str(Path(__file__).with_name("landmarks.json")))
)

# Distance thresholds (meters)
LANDMARK_MATCH_RADIUS_METERS = float(os.environ.get("LANDMARK_MATCH_RADIUS_METERS", "0.6"))
QUEUE_CANCEL_RADIUS_METERS = float(os.environ.get("QUEUE_CANCEL_RADIUS_METERS", "0.45"))

HAS_WEBSOCKET_RUNTIME = bool(
    importlib.util.find_spec("websockets") or importlib.util.find_spec("wsproto")
)

ros_node_holder = {"node": None, "executor": None, "running": False}
state_lock = threading.Lock()
async_loop_holder: dict[str, Optional[asyncio.AbstractEventLoop]] = {"loop": None}

# Global state shared between ROS thread + FastAPI thread
# Protected by `state_lock`
latest_state: Dict[str, Any] = {
    "robot_pose": None, # transformed into map frame
    "raw_robot_pose": None, # raw odometry frame
    "goal_pose": None,
    "return_home_goal_pose": None,
    "queue_goals": [], # raw queue markers
    "queue_status": None, # text status from marker
    "movement_gate_open": True, # whether robot allowed to move
    "return_home_signal": True, # mirrors movement gate (legacy)
    "map": None, # occupancy grid
    "last_publish_result": None,
    "config": {
        "target_topic": TARGET_TOPIC,
        "home_target_topic": HOME_TARGET_TOPIC,
        "movement_gate_topic": MOVEMENT_GATE_TOPIC,
        "return_home_topic": LEGACY_RETURN_HOME_TOPIC,
        "set_home_topic": SET_HOME_TOPIC,
        "flavor_selection_topic": FLAVOR_SELECTION_TOPIC,
        "currently_dispensing_topic": CURRENTLY_DISPENSING_TOPIC,
        "dispense_empty_topic": DISPENSE_EMPTY_TOPIC,
        "dispense_uart_event_topic": DISPENSE_UART_EVENT_TOPIC,
        "dispense_uart_command_topic": DISPENSE_UART_COMMAND_TOPIC,
        "queue_cancel_topic": QUEUE_CANCEL_TOPIC,
        "frame_id": FRAME_ID,
        "map_topic": MAP_TOPIC,
        "odom_topic": ODOM_TOPIC,
        "queue_topic": QUEUE_TOPIC,
        "landmark_match_radius_m": LANDMARK_MATCH_RADIUS_METERS,
        "landmarks_path": str(LANDMARKS_PATH),
    },
    "flavor_selection": 0,
    "currently_dispensing": False,
    "dispense_empty": False,
    "last_uart_event": None,
    "dispense_levels": None,
    # Landmark system
    "landmarks": [],
}

CLEAR_GOAL_STATUSES = {4, 6}
ACTIVE_QUEUE_LABEL = "ACTIVE"
TERMINAL_ORDER_STATUSES = {"completed", "failed", "canceled"}
LEVELS_PATTERN = re.compile(
    r"^LEVELS:W=(?P<w_current>[0-9.]+)/(?P<w_total>[0-9.]+),"
    r"A=(?P<a_current>[0-9.]+)/(?P<a_total>[0-9.]+),"
    r"B=(?P<b_current>[0-9.]+)/(?P<b_total>[0-9.]+),"
    r"C=(?P<c_current>[0-9.]+)/(?P<c_total>[0-9.]+)$"
)
TRACKED_QUEUE_ORDER_STATUSES = {"queued", "executing"}


class LandmarkRequest(BaseModel):
    id: Optional[int] = Field(default=None)
    label: Optional[str] = Field(default=None)
    x: float
    y: float
    yaw: float = 0.0
    enabled: bool = True


class OrderRequest(BaseModel):
    landmark_id: int
    flavor: int
    note: Optional[str] = None


orders_state: Dict[str, Any] = {
    "next_order_id": 1,
    "orders": {},
}


class ConnectionManager:
    """
    Tracks active websocket clients and allows broadcasting updates.

    Used by:
      - ROS callbacks to push state to UI
      - API events to notify UI
    """
    def __init__(self) -> None:
        self._connections: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        async with self._lock:
            self._connections.add(ws)

    async def disconnect(self, ws: WebSocket) -> None:
        async with self._lock:
            self._connections.discard(ws)

    async def send_json(self, ws: WebSocket, payload: Dict[str, Any]) -> None:
        await ws.send_text(json.dumps(payload))

    async def broadcast(self, payload: Dict[str, Any]) -> None:
        async with self._lock:
            connections = list(self._connections)
        if not connections:
            return
        message = json.dumps(payload)
        stale: list[WebSocket] = []
        for ws in connections:
            try:
                await ws.send_text(message)
            except Exception:
                stale.append(ws)
        if stale:
            async with self._lock:
                for ws in stale:
                    self._connections.discard(ws)


manager = ConnectionManager()


def quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    return sin(0.5 * yaw), cos(0.5 * yaw)


def yaw_from_quaternion(z: float, w: float) -> float:
    return atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def normalize_angle(angle: float) -> float:
    while angle > 3.141592653589793:
        angle -= 2.0 * 3.141592653589793
    while angle < -3.141592653589793:
        angle += 2.0 * 3.141592653589793
    return angle


def compose_2d_pose(base: Dict[str, Any], local: Dict[str, Any]) -> Dict[str, Any]:
    cos_yaw = cos(base["yaw"])
    sin_yaw = sin(base["yaw"])
    return {
        "x": base["x"] + cos_yaw * local["x"] - sin_yaw * local["y"],
        "y": base["y"] + sin_yaw * local["x"] + cos_yaw * local["y"],
        "yaw": normalize_angle(base["yaw"] + local["yaw"]),
    }


def invert_2d_pose(pose: Dict[str, Any]) -> Dict[str, Any]:
    cos_yaw = cos(pose["yaw"])
    sin_yaw = sin(pose["yaw"])
    return {
        "x": -cos_yaw * pose["x"] - sin_yaw * pose["y"],
        "y": sin_yaw * pose["x"] - cos_yaw * pose["y"],
        "yaw": normalize_angle(-pose["yaw"]),
    }


def make_pose_stamped(
    x: float,
    y: float,
    yaw: float = 0.0,
    frame_id: Optional[str] = None,
) -> PoseStamped:
    node: Node = ros_node_holder.get("node")
    if node is None:
        raise RuntimeError("ROS node not ready")

    qz, qw = quaternion_from_yaw(float(yaw))
    ps = PoseStamped()
    ps.header.stamp = node.get_clock().now().to_msg()
    ps.header.frame_id = frame_id or FRAME_ID
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = 0.0
    ps.pose.orientation.x = 0.0
    ps.pose.orientation.y = 0.0
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps


def pose_to_dict(msg: PoseStamped) -> Dict[str, Any]:
    return {
        "frame_id": msg.header.frame_id,
        "x": msg.pose.position.x,
        "y": msg.pose.position.y,
        "z": msg.pose.position.z,
        "yaw": yaw_from_quaternion(
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ),
    }


def odom_to_pose_dict(msg: Odometry) -> Dict[str, Any]:
    return {
        "frame_id": msg.header.frame_id,
        "child_frame_id": msg.child_frame_id,
        "x": msg.pose.pose.position.x,
        "y": msg.pose.pose.position.y,
        "z": msg.pose.pose.position.z,
        "yaw": yaw_from_quaternion(
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ),
        "linear_x": msg.twist.twist.linear.x,
        "linear_y": msg.twist.twist.linear.y,
        "angular_z": msg.twist.twist.angular.z,
    }


def occupancy_grid_to_dict(msg: OccupancyGrid) -> Dict[str, Any]:
    return {
        "frame_id": msg.header.frame_id,
        "width": msg.info.width,
        "height": msg.info.height,
        "resolution": msg.info.resolution,
        "origin": {
            "x": msg.info.origin.position.x,
            "y": msg.info.origin.position.y,
            "yaw": yaw_from_quaternion(
                msg.info.origin.orientation.z,
                msg.info.origin.orientation.w,
            ),
        },
        "data": list(msg.data),
    }


def pose_dict_from_xy(
    x: float,
    y: float,
    yaw: float = 0.0,
    frame_id: Optional[str] = None,
) -> Dict[str, Any]:
    return {
        "frame_id": frame_id or FRAME_ID,
        "x": float(x),
        "y": float(y),
        "z": 0.0,
        "yaw": float(yaw),
    }


def sanitize_landmark(raw: Dict[str, Any]) -> Dict[str, Any]:
    landmark_id = int(raw["id"])
    return {
        "id": landmark_id,
        "label": str(raw.get("label") or f"Landmark {landmark_id}"),
        "x": float(raw["x"]),
        "y": float(raw["y"]),
        "yaw": float(raw.get("yaw", 0.0)),
        "enabled": bool(raw.get("enabled", True)),
    }


def sorted_landmarks(landmarks: list[Dict[str, Any]]) -> list[Dict[str, Any]]:
    return sorted((sanitize_landmark(item) for item in landmarks), key=lambda item: item["id"])


def load_landmarks() -> list[Dict[str, Any]]:
    try:
        payload = json.loads(LANDMARKS_PATH.read_text(encoding="utf-8"))
        if not isinstance(payload, list):
            return []
        landmarks, _ = compact_landmark_ids(payload)
        try:
            save_landmarks(landmarks)
        except Exception:
            pass
        return landmarks
    except FileNotFoundError:
        return []
    except Exception:
        return []


def save_landmarks(landmarks: list[Dict[str, Any]]) -> None:
    LANDMARKS_PATH.parent.mkdir(parents=True, exist_ok=True)
    LANDMARKS_PATH.write_text(
        json.dumps(sorted_landmarks(landmarks), indent=2) + "\n",
        encoding="utf-8",
    )


def compact_landmark_ids(landmarks: list[Dict[str, Any]]) -> tuple[list[Dict[str, Any]], Dict[int, int]]:
    remap: Dict[int, int] = {}
    compacted: list[Dict[str, Any]] = []
    for new_id, landmark in enumerate(sorted_landmarks(landmarks), start=1):
        old_id = int(landmark["id"])
        remap[old_id] = new_id
        compacted.append(
            sanitize_landmark(
                {
                    **landmark,
                    "id": new_id,
                    "label": f"Landmark {new_id}",
                }
            )
        )
    return compacted, remap


def ensure_landmarks_compacted_locked() -> Dict[int, int]:
    compacted, remap = compact_landmark_ids(latest_state["landmarks"])
    if compacted != latest_state["landmarks"]:
        latest_state["landmarks"] = compacted
        save_landmarks(compacted)
    return remap


def next_landmark_id(landmarks: list[Dict[str, Any]]) -> int:
    taken = {int(item["id"]) for item in landmarks}
    candidate = 1
    while candidate in taken:
        candidate += 1
    return candidate


def find_landmark_by_id(landmark_id: int) -> Optional[Dict[str, Any]]:
    with state_lock:
        for landmark in latest_state["landmarks"]:
            if int(landmark["id"]) == landmark_id:
                return dict(landmark)
    return None


def match_landmark_for_pose(
    x: float,
    y: float,
    *,
    max_radius: float = LANDMARK_MATCH_RADIUS_METERS,
    landmarks: Optional[list[Dict[str, Any]]] = None,
) -> Optional[Dict[str, Any]]:
    """
    Finds the nearest landmark within a radius.
    Used to:
      - snap goals to landmarks
      - associate queue items with landmarks
    """
    candidates = landmarks
    if candidates is None:
        with state_lock:
            candidates = list(latest_state["landmarks"])

    best: Optional[Dict[str, Any]] = None
    best_distance_sq = max_radius * max_radius
    for landmark in candidates:
        dx = float(landmark["x"]) - float(x)
        dy = float(landmark["y"]) - float(y)
        distance_sq = dx * dx + dy * dy
        if distance_sq <= best_distance_sq:
            best = landmark
            best_distance_sq = distance_sq
    return dict(best) if best is not None else None


def build_queue_landmark_summary_locked() -> list[Dict[str, Any]]:
    landmarks = list(latest_state["landmarks"])
    summary: list[Dict[str, Any]] = []
    for goal in latest_state["queue_goals"]:
        matched = match_landmark_for_pose(
            goal["x"],
            goal["y"],
            landmarks=landmarks,
        )
        item = {
            "label": goal["label"],
            "x": float(goal["x"]),
            "y": float(goal["y"]),
        }
        if matched is not None:
            item["landmark_id"] = matched["id"]
            item["landmark_label"] = matched["label"]
        summary.append(item)
    return summary


def queue_label_position(label: Optional[str]) -> int:
    if label == ACTIVE_QUEUE_LABEL:
        return 0
    if isinstance(label, str) and label.startswith("Q"):
        try:
            return max(1, int(label[1:]))
        except ValueError:
            return 10_000
    return 10_000


def attach_order_ids_to_queue_summary_locked(
    queue_landmarks: list[Dict[str, Any]],
) -> list[Dict[str, Any]]:
    summary = [dict(item) for item in queue_landmarks]
    grouped_items: Dict[int, list[Dict[str, Any]]] = {}
    for item in summary:
        item["queue_position"] = queue_label_position(item.get("label"))
        landmark_id = item.get("landmark_id")
        if landmark_id is None:
            continue
        grouped_items.setdefault(int(landmark_id), []).append(item)

    for items in grouped_items.values():
        items.sort(key=lambda item: queue_label_position(item.get("label")))

    grouped_orders: Dict[int, list[Dict[str, Any]]] = {}
    for order in orders_state["orders"].values():
        if order["status"] in TERMINAL_ORDER_STATUSES:
            continue
        grouped_orders.setdefault(int(order["landmark_id"]), []).append(order)

    for orders in grouped_orders.values():
        orders.sort(key=lambda order: int(order["order_id"]))

    for landmark_id, items in grouped_items.items():
        orders = grouped_orders.get(landmark_id, [])
        for item, order in zip(items, orders):
            item["order_id"] = int(order["order_id"])
            item["flavor"] = int(order["flavor"])

    return summary


def reconcile_orders_locked() -> None:
    """
    Synchronizes queue markers from ROS with internal order state

    Ensures:
      - orders become "executing" when ACTIVE
      - extra orders become "completed"
      - queue positions stay aligned

    This is the connection between UI orders and nav2 queue.
    """
    queue_landmarks = build_queue_landmark_summary_locked()
    orders = orders_state["orders"]
    queue_counts_by_landmark: Dict[int, int] = {}
    for item in queue_landmarks:
        landmark_id = item.get("landmark_id")
        if landmark_id is None:
            continue
        landmark_id = int(landmark_id)
        queue_counts_by_landmark[landmark_id] = queue_counts_by_landmark.get(landmark_id, 0) + 1

    tracked_by_landmark: Dict[int, list[Dict[str, Any]]] = {}
    for order in orders.values():
        if order["status"] not in TRACKED_QUEUE_ORDER_STATUSES:
            continue
        tracked_by_landmark.setdefault(int(order["landmark_id"]), []).append(order)

    for tracked_orders in tracked_by_landmark.values():
        tracked_orders.sort(key=lambda order: int(order["order_id"]))

    for landmark_id, tracked_orders in tracked_by_landmark.items():
        current_count = queue_counts_by_landmark.get(landmark_id, 0)
        while len(tracked_orders) > current_count:
            completed_order = tracked_orders.pop(0)
            completed_order["status"] = "completed"
            completed_order["queue_label"] = None
            completed_order["queue_position"] = None

    active_order_ids: set[int] = set()
    grouped_queue_items: Dict[int, list[Dict[str, Any]]] = {}
    for item in queue_landmarks:
        landmark_id = item.get("landmark_id")
        if landmark_id is None:
            continue
        grouped_queue_items.setdefault(int(landmark_id), []).append(item)

    for queue_items in grouped_queue_items.values():
        queue_items.sort(key=lambda item: queue_label_position(item.get("label")))

    grouped_orders: Dict[int, list[Dict[str, Any]]] = {}
    for order in orders.values():
        if order["status"] in TERMINAL_ORDER_STATUSES:
            continue
        grouped_orders.setdefault(int(order["landmark_id"]), []).append(order)

    for candidate_orders in grouped_orders.values():
        candidate_orders.sort(key=lambda order: int(order["order_id"]))

    for landmark_id, queue_items in grouped_queue_items.items():
        candidate_orders = grouped_orders.get(landmark_id, [])
        for item, order in zip(queue_items, candidate_orders):
            queue_label = item["label"]
            order["queue_label"] = queue_label
            order["queue_position"] = queue_label_position(queue_label)
            order["status"] = "executing" if queue_label == ACTIVE_QUEUE_LABEL else "queued"
            active_order_ids.add(int(order["order_id"]))

    for order_id, order in orders.items():
        if order["status"] in TERMINAL_ORDER_STATUSES:
            continue
        if order_id in active_order_ids:
            continue
        order["queue_label"] = None
        order["queue_position"] = None


def cancel_order_for_pose_locked(x: float, y: float) -> Optional[Dict[str, Any]]:
    queue_landmarks = attach_order_ids_to_queue_summary_locked(build_queue_landmark_summary_locked())
    best_item: Optional[Dict[str, Any]] = None
    best_distance_sq = QUEUE_CANCEL_RADIUS_METERS * QUEUE_CANCEL_RADIUS_METERS
    for item in queue_landmarks:
        dx = float(item["x"]) - float(x)
        dy = float(item["y"]) - float(y)
        distance_sq = dx * dx + dy * dy
        if distance_sq <= best_distance_sq:
            best_item = item
            best_distance_sq = distance_sq

    if best_item is None:
        return None

    order_id = best_item.get("order_id")
    if order_id is None:
        return None

    order = orders_state["orders"].get(int(order_id))
    if order is None or order["status"] in TERMINAL_ORDER_STATUSES:
        return None

    order["status"] = "canceled"
    order["queue_label"] = None
    order["queue_position"] = None
    return order_to_public_dict(order)


def order_to_public_dict(order: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "order_id": int(order["order_id"]),
        "landmark_id": int(order["landmark_id"]),
        "landmark_label": str(order["landmark_label"]),
        "flavor": int(order["flavor"]),
        "status": str(order["status"]),
        "queue_label": order.get("queue_label"),
        "queue_position": order.get("queue_position"),
        "created_at": float(order["created_at"]),
        "note": order.get("note"),
    }


def parse_dispense_levels_line(text: str) -> Optional[Dict[str, Any]]:
    match = LEVELS_PATTERN.match(text.strip())
    if match is None:
        return None

    def bucket(prefix: str, label: str) -> Dict[str, Any]:
        current = float(match.group(f"{prefix}_current"))
        total = float(match.group(f"{prefix}_total"))
        percent = 0.0 if total <= 0.0 else max(0.0, min(100.0, (current / total) * 100.0))
        return {
            "label": label,
            "current_ml": current,
            "total_ml": total,
            "percent": percent,
        }

    return {
        "water": bucket("w", "Water"),
        "flavor_a": bucket("a", "Flavor A"),
        "flavor_b": bucket("b", "Flavor B"),
        "flavor_c": bucket("c", "Flavor C"),
        "raw": text.strip(),
    }


def snapshot_state() -> Dict[str, Any]:
    with state_lock:
        queue_landmarks = attach_order_ids_to_queue_summary_locked(
            build_queue_landmark_summary_locked()
        )
        return {
            "robot_pose": latest_state["robot_pose"],
            "goal_pose": latest_state["goal_pose"],
            "return_home_goal_pose": latest_state["return_home_goal_pose"],
            "queue_goals": latest_state["queue_goals"],
            "queue_landmarks": queue_landmarks,
            "queue_status": latest_state["queue_status"],
            "movement_gate_open": latest_state["movement_gate_open"],
            "return_home_signal": latest_state["return_home_signal"],
            "map": latest_state["map"],
            "last_publish_result": latest_state["last_publish_result"],
            "flavor_selection": latest_state["flavor_selection"],
            "currently_dispensing": latest_state["currently_dispensing"],
            "dispense_empty": latest_state["dispense_empty"],
            "last_uart_event": latest_state["last_uart_event"],
            "dispense_levels": latest_state["dispense_levels"],
            "landmarks": latest_state["landmarks"],
            "config": latest_state["config"],
        }


def schedule_broadcast(payload: Dict[str, Any]) -> None:
    loop = async_loop_holder.get("loop")
    if loop is None:
        return
    loop.call_soon_threadsafe(asyncio.create_task, manager.broadcast(payload))


def log_second_ui_get(message: str) -> None:
    schedule_broadcast(
        {
            "type": "activity_log",
            "message": message,
        }
    )


def log_activity(message: str) -> None:
    schedule_broadcast(
        {
            "type": "activity_log",
            "message": message,
        }
    )


def flavor_selection_to_uart_code(selection: int) -> str:
    return {1: "A", 2: "B", 3: "C"}.get(int(selection), "?")


def publish_uart_for_executing_orders() -> None:
    with state_lock:
        reconcile_orders_locked()
        pending = [
            (int(order["order_id"]), int(order["flavor"]))
            for order in orders_state["orders"].values()
            if order.get("status") == "executing" and not order.get("uart_published", False)
        ]

    if not pending:
        return

    node = ros_node_holder.get("node")
    if node is None:
        return

    for order_id, flavor in pending:
        try:
            node.publish_flavor_selection(flavor)
        except Exception:
            continue

        with state_lock:
            order = orders_state["orders"].get(order_id)
            if order is None:
                continue
            if order.get("status") != "executing" or order.get("uart_published", False):
                continue
            order["uart_published"] = True

        log_activity(f"Published UART {flavor_selection_to_uart_code(flavor)}")


with state_lock:
    latest_state["landmarks"] = load_landmarks()


class RosBridgeNode(Node):
    """
    ROS2 node that:
      - subscribes to robot + map + queue + UART topics
      - publishes navigation goals, cancel commands, dispenser commands
      - transforms poses into map frame using TF
      - pushes updates to the web UI via websocket
    """
    def __init__(self) -> None:
        super().__init__("glyph_web_bridge")
        self.transforms: Dict[tuple[str, str], Dict[str, float]] = {}
        self._movement_gate_repeat_timers: list[threading.Timer] = []
        self._gate_publish_topics = [MOVEMENT_GATE_TOPIC]
        if LEGACY_RETURN_HOME_TOPIC not in self._gate_publish_topics:
            self._gate_publish_topics.append(LEGACY_RETURN_HOME_TOPIC)
        target_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        live_command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(PoseStamped, TARGET_TOPIC, target_qos)
        self.cancel_pub = self.create_publisher(PoseStamped, QUEUE_CANCEL_TOPIC, live_command_qos)
        self.movement_gate_pubs = [
            self.create_publisher(Bool, topic, target_qos) for topic in self._gate_publish_topics
        ]
        self.currently_dispensing_pub = self.create_publisher(
            Bool, CURRENTLY_DISPENSING_TOPIC, target_qos
        )
        self.uart_command_pub = self.create_publisher(
            String, DISPENSE_UART_COMMAND_TOPIC, 10
        )
        self.set_home_pub = self.create_publisher(Bool, SET_HOME_TOPIC, 10)
        self.flavor_selection_pub = self.create_publisher(
            UInt8, FLAVOR_SELECTION_TOPIC, target_qos
        )

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self.odom_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            TARGET_TOPIC,
            self.goal_callback,
            target_qos,
        )
        self.create_subscription(
            PoseStamped,
            HOME_TARGET_TOPIC,
            self.return_home_goal_callback,
            target_qos,
        )
        self.create_subscription(
            Bool,
            MOVEMENT_GATE_TOPIC,
            self.movement_gate_callback,
            target_qos,
        )
        if LEGACY_RETURN_HOME_TOPIC != MOVEMENT_GATE_TOPIC:
            self.create_subscription(
                Bool,
                LEGACY_RETURN_HOME_TOPIC,
                self.movement_gate_callback,
                target_qos,
            )
        self.create_subscription(
            Bool,
            CURRENTLY_DISPENSING_TOPIC,
            self.currently_dispensing_callback,
            target_qos,
        )
        self.create_subscription(
            Bool,
            DISPENSE_EMPTY_TOPIC,
            self.dispense_empty_callback,
            target_qos,
        )
        self.create_subscription(
            String,
            DISPENSE_UART_EVENT_TOPIC,
            self.uart_event_callback,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            MAP_TOPIC,
            self.map_callback,
            map_qos,
        )
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.goal_status_callback,
            10,
        )
        self.create_subscription(
            Empty,
            "/target_location_cleared",
            self.goal_cleared_callback,
            10,
        )
        self.create_subscription(
            MarkerArray,
            QUEUE_TOPIC,
            self.queue_callback,
            target_qos,
        )
        self.create_subscription(
            TFMessage,
            "/tf",
            self.tf_callback,
            50,
        )
        self.create_subscription(
            TFMessage,
            "/tf_static",
            self.tf_callback,
            map_qos,
        )
        self.create_timer(0.2, self.publish_live_state)

        self._last_pose_sent = 0.0
        self._last_goal_sent = None
        self._last_return_home_goal_sent = None
        self._last_queue_goals_sent = None
        self._last_queue_landmarks_sent = None
        self._last_queue_status_sent = None
        self._last_landmarks_sent = None
        self._map_dirty = False
        self._last_odom_stamp_ns: Optional[int] = None

        self.get_logger().info(
            f"[glyph_web] Publishing goals to {TARGET_TOPIC}, movement gate to {MOVEMENT_GATE_TOPIC} "
            f"(legacy alias {LEGACY_RETURN_HOME_TOPIC}), "
            f"home target from {HOME_TARGET_TOPIC}, set-home command to {SET_HOME_TOPIC}, "
            f"flavor selection to {FLAVOR_SELECTION_TOPIC}, dispensing state from {CURRENTLY_DISPENSING_TOPIC}, "
            f"empty state from {DISPENSE_EMPTY_TOPIC}, UART events from {DISPENSE_UART_EVENT_TOPIC}, "
            f"UART commands to {DISPENSE_UART_COMMAND_TOPIC}, "
            f"queue cancel to {QUEUE_CANCEL_TOPIC}, "
            f"reading pose from {ODOM_TOPIC}, map from {MAP_TOPIC}, queue from {QUEUE_TOPIC}"
        )
        self._sync_gate_from_dispensing_state(self._currently_dispensing())

    def publish_pose(self, ps: PoseStamped, source: str = "xy") -> None:
        """
        Sends navigation goal to nav2.

        Also:
        - blocks if dispenser empty
        - transforms pose into map frame for UI
        - updates global state
        - broadcasts to clients
        """
        if self._dispense_empty():
            raise RuntimeError("Navigation goal blocked because dispenser is empty")
        self.pub.publish(ps)
        goal_pose = pose_to_dict(ps)
        with state_lock:
            map_frame = latest_state["map"]["frame_id"] if latest_state["map"] is not None else None
        goal_pose = self.transform_pose_to_frame(goal_pose, map_frame)
        publish_result = {
            "ok": True,
            "source": source,
            "sent_at": time.time(),
        }
        with state_lock:
            latest_state["goal_pose"] = goal_pose
            latest_state["return_home_goal_pose"] = None
            latest_state["last_publish_result"] = publish_result
        schedule_broadcast(
            {
                "type": "goal",
                "goal_pose": goal_pose,
                "last_publish_result": publish_result,
            }
        )

    def publish_cancel_pose(self, ps: PoseStamped, source: str = "cancel_xy") -> None:
        self.cancel_pub.publish(ps)
        cancel_pose = pose_to_dict(ps)
        with state_lock:
            map_frame = latest_state["map"]["frame_id"] if latest_state["map"] is not None else None
        cancel_pose = self.transform_pose_to_frame(cancel_pose, map_frame)
        publish_result = {
            "ok": True,
            "source": source,
            "sent_at": time.time(),
        }
        with state_lock:
            canceled_order = cancel_order_for_pose_locked(
                float(cancel_pose["x"]),
                float(cancel_pose["y"]),
            )
            latest_state["last_publish_result"] = publish_result
        schedule_broadcast(
            {
                "type": "queue_cancel",
                "cancel_pose": cancel_pose,
                "canceled_order": canceled_order,
                "last_publish_result": publish_result,
            }
        )

    def publish_gate_state(self, state: bool) -> None:
        if state and self._currently_dispensing():
            raise RuntimeError("Movement gate open blocked while currently dispensing")
        if state and self._dispense_empty():
            raise RuntimeError("Movement gate open blocked because dispenser is empty")
        for timer in self._movement_gate_repeat_timers:
            timer.cancel()
        self._movement_gate_repeat_timers = []

        self._publish_movement_gate_state(state)
        for delay_sec in (0.2, 0.6):
            timer = threading.Timer(delay_sec, self._publish_movement_gate_state, args=(state,))
            timer.daemon = True
            timer.start()
            self._movement_gate_repeat_timers.append(timer)

        publish_result = {
            "ok": True,
            "source": "gate_state",
            "sent_at": time.time(),
            "detail": "HIGH" if state else "LOW",
        }
        with state_lock:
            latest_state["movement_gate_open"] = state
            latest_state["return_home_signal"] = state
            latest_state["last_publish_result"] = publish_result
        schedule_broadcast(
            {
                "type": "gate_state",
                "movement_gate_open": state,
                "return_home_signal": state,
                "last_publish_result": publish_result,
            }
        )

    def publish_currently_dispensing(self, state: bool) -> None:
        msg = Bool()
        msg.data = state
        self.currently_dispensing_pub.publish(msg)
        self._sync_gate_from_dispensing_state(state)

        publish_result = {
            "ok": True,
            "source": "currently_dispensing",
            "sent_at": time.time(),
            "detail": "true" if state else "false",
        }
        with state_lock:
            latest_state["currently_dispensing"] = state
            latest_state["last_publish_result"] = publish_result
        schedule_broadcast(
            {
                "type": "currently_dispensing",
                "currently_dispensing": state,
                "last_publish_result": publish_result,
            }
        )

    def publish_set_home_here(self) -> None:
        msg = Bool()
        msg.data = True
        self.set_home_pub.publish(msg)

        publish_result = {
            "ok": True,
            "source": "set_home",
            "sent_at": time.time(),
            "detail": "set_home_here",
        }
        with state_lock:
            latest_state["last_publish_result"] = publish_result
        schedule_broadcast(
            {
                "type": "set_home",
                "last_publish_result": publish_result,
            }
        )

    def publish_flavor_selection(self, selection: int) -> None:
        if selection not in (1, 2, 3):
            raise ValueError("Flavor selection must be one of 1, 2, or 3")
        if self._dispense_empty():
            raise RuntimeError("Flavor selection blocked because dispenser is empty")

        msg = UInt8()
        msg.data = selection
        self.flavor_selection_pub.publish(msg)

        publish_result = {
            "ok": True,
            "source": "flavor_selection",
            "sent_at": time.time(),
            "detail": f"{selection:02b}",
        }
        with state_lock:
            latest_state["flavor_selection"] = selection
            latest_state["last_publish_result"] = publish_result
        schedule_broadcast(
            {
                "type": "flavor_selection",
                "flavor_selection": selection,
                "last_publish_result": publish_result,
            }
        )

    def query_dispense_levels(self) -> None:
        msg = String()
        msg.data = "?"
        self.uart_command_pub.publish(msg)

        publish_result = {
            "ok": True,
            "source": "dispense_query",
            "sent_at": time.time(),
            "detail": "?",
        }
        with state_lock:
            latest_state["last_publish_result"] = publish_result
        schedule_broadcast(
            {
                "type": "dispense_query",
                "last_publish_result": publish_result,
            }
        )

    def _publish_movement_gate_state(self, state: bool) -> None:
        msg = Bool()
        msg.data = state
        for publisher in self.movement_gate_pubs:
            publisher.publish(msg)

    def odom_callback(self, msg: Odometry) -> None:
        raw_pose = odom_to_pose_dict(msg)
        map_frame = None
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        with state_lock:
            if latest_state["map"] is not None:
                map_frame = latest_state["map"]["frame_id"]
            if self._last_odom_stamp_ns is not None and stamp_ns < self._last_odom_stamp_ns:
                latest_state["goal_pose"] = None
                latest_state["last_publish_result"] = {
                    "ok": True,
                    "source": "system",
                    "detail": "goal_cleared_after_clock_reset",
                    "sent_at": time.time(),
                }
            latest_state["raw_robot_pose"] = raw_pose
            latest_state["robot_pose"] = self.transform_robot_pose_to_map(raw_pose, map_frame)
        self._last_odom_stamp_ns = stamp_ns

    def goal_callback(self, msg: PoseStamped) -> None:
        goal_pose = pose_to_dict(msg)
        with state_lock:
            map_frame = latest_state["map"]["frame_id"] if latest_state["map"] is not None else None
        goal_pose = self.transform_pose_to_frame(goal_pose, map_frame)
        with state_lock:
            latest_state["goal_pose"] = goal_pose
        schedule_broadcast(
            {
                "type": "goal",
                "goal_pose": goal_pose,
                "last_publish_result": snapshot_state()["last_publish_result"],
            }
        )

    def return_home_goal_callback(self, msg: PoseStamped) -> None:
        goal_pose = pose_to_dict(msg)
        with state_lock:
            map_frame = latest_state["map"]["frame_id"] if latest_state["map"] is not None else None
        goal_pose = self.transform_pose_to_frame(goal_pose, map_frame)
        with state_lock:
            latest_state["return_home_goal_pose"] = goal_pose
        schedule_broadcast(
            {
                "type": "return_home_goal",
                "return_home_goal_pose": goal_pose,
                "last_publish_result": snapshot_state()["last_publish_result"],
            }
        )

    def movement_gate_callback(self, msg: Bool) -> None:
        state = bool(msg.data)
        with state_lock:
            latest_state["movement_gate_open"] = state
            latest_state["return_home_signal"] = state
        schedule_broadcast(
            {
                "type": "movement_gate",
                "movement_gate_open": state,
                "return_home_signal": state,
            }
        )

    def currently_dispensing_callback(self, msg: Bool) -> None:
        state = bool(msg.data)
        self._sync_gate_from_dispensing_state(state)
        with state_lock:
            latest_state["currently_dispensing"] = state
        schedule_broadcast(
            {
                "type": "currently_dispensing",
                "currently_dispensing": state,
            }
        )

    def _currently_dispensing(self) -> bool:
        with state_lock:
            return bool(latest_state["currently_dispensing"])

    def _sync_gate_from_dispensing_state(self, dispensing: bool) -> None:
        gate_state = not dispensing
        for timer in self._movement_gate_repeat_timers:
            timer.cancel()
        self._movement_gate_repeat_timers = []

        self._publish_movement_gate_state(gate_state)
        for delay_sec in (0.2, 0.6):
            timer = threading.Timer(delay_sec, self._publish_movement_gate_state, args=(gate_state,))
            timer.daemon = True
            timer.start()
            self._movement_gate_repeat_timers.append(timer)
        with state_lock:
            latest_state["movement_gate_open"] = gate_state
            latest_state["return_home_signal"] = gate_state

    def dispense_empty_callback(self, msg: Bool) -> None:
        state = bool(msg.data)
        with state_lock:
            latest_state["dispense_empty"] = state
        schedule_broadcast(
            {
                "type": "dispense_empty",
                "dispense_empty": state,
            }
        )

    def uart_event_callback(self, msg: String) -> None:
        text = str(msg.data)
        levels = parse_dispense_levels_line(text)
        with state_lock:
            latest_state["last_uart_event"] = text
            if levels is not None:
                latest_state["dispense_levels"] = levels
        schedule_broadcast(
            {
                "type": "uart_event",
                "uart_event": text,
                "dispense_levels": levels,
            }
        )

    def _dispense_empty(self) -> bool:
        with state_lock:
            return bool(latest_state["dispense_empty"])

    def goal_status_callback(self, msg: GoalStatusArray) -> None:
        if not msg.status_list:
            return

        latest_status = msg.status_list[-1].status
        if latest_status not in CLEAR_GOAL_STATUSES:
            return

        with state_lock:
            if latest_state["goal_pose"] is None:
                return
            latest_state["goal_pose"] = None
            latest_state["last_publish_result"] = {
                "ok": True,
                "source": "nav2_status",
                "detail": f"goal_cleared_status_{latest_status}",
                "sent_at": time.time(),
            }

        schedule_broadcast(
            {
                "type": "goal_cleared",
                "goal_pose": None,
                "last_publish_result": snapshot_state()["last_publish_result"],
            }
        )

    def goal_cleared_callback(self, _msg: Empty) -> None:
        with state_lock:
            latest_state["goal_pose"] = None
            latest_state["last_publish_result"] = {
                "ok": True,
                "source": "location_subscriber",
                "detail": "goal_cleared",
                "sent_at": time.time(),
            }

        schedule_broadcast(
            {
                "type": "goal_cleared",
                "goal_pose": None,
                "last_publish_result": snapshot_state()["last_publish_result"],
            }
        )

    def queue_callback(self, msg: MarkerArray) -> None:
        """
        Parses behavior_supervisor queue visualization markers.

        Extracts:
        - ACTIVE + Q1, Q2, ... labels
        - coordinates
        - queue status text

        Then:
        - updates internal queue state
        - reconciles orders
        - triggers UART for executing orders (flavors)
        """
        queue_goals: list[Dict[str, Any]] = []
        queue_status: Optional[str] = None
        with state_lock:
            map_frame = latest_state["map"]["frame_id"] if latest_state["map"] is not None else None

        for marker in msg.markers:
            if marker.action != 0:
                continue
            if marker.ns == "behavior_supervisor_queue_labels":
                label = marker.text or ""
                if label == "ACTIVE" or label.startswith("Q"):
                    transformed = self.transform_pose_to_frame(
                        pose_dict_from_xy(
                            marker.pose.position.x,
                            marker.pose.position.y,
                            yaw_from_quaternion(
                                float(marker.pose.orientation.z),
                                float(marker.pose.orientation.w),
                            ),
                            marker.header.frame_id,
                        ),
                        map_frame,
                    )
                    queue_goals.append(
                        {
                            "label": label,
                            "x": float(transformed["x"]),
                            "y": float(transformed["y"]),
                        }
                    )
            elif marker.ns == "behavior_supervisor_status":
                queue_status = marker.text or None

        queue_goals.sort(
            key=lambda item: (0 if item["label"] == "ACTIVE" else 1, item["label"])
        )

        with state_lock:
            latest_state["queue_goals"] = queue_goals
            latest_state["queue_status"] = queue_status
            if queue_status not in {"Returning home"} and not (
                queue_status and queue_status.startswith("Home queued")
            ):
                latest_state["return_home_goal_pose"] = None
            reconcile_orders_locked()
            return_home_goal_pose = latest_state["return_home_goal_pose"]
            queue_landmarks = attach_order_ids_to_queue_summary_locked(
                build_queue_landmark_summary_locked()
            )

        schedule_broadcast(
            {
                "type": "queue",
                "queue_goals": queue_goals,
                "queue_status": queue_status,
                "queue_landmarks": queue_landmarks,
                "return_home_goal_pose": return_home_goal_pose,
            }
        )
        publish_uart_for_executing_orders()

    def map_callback(self, msg: OccupancyGrid) -> None:
        map_state = occupancy_grid_to_dict(msg)
        with state_lock:
            latest_state["map"] = map_state
            raw_pose = latest_state["raw_robot_pose"]
            if raw_pose is not None:
                latest_state["robot_pose"] = self.transform_robot_pose_to_map(
                    raw_pose, map_state["frame_id"]
                )
        self._map_dirty = True

    def tf_callback(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            pose = {
                "x": float(transform.transform.translation.x),
                "y": float(transform.transform.translation.y),
                "yaw": yaw_from_quaternion(
                    float(transform.transform.rotation.z),
                    float(transform.transform.rotation.w),
                ),
            }
            self.transforms[(parent, child)] = pose
            self.transforms[(child, parent)] = invert_2d_pose(pose)

        with state_lock:
            raw_pose = latest_state["raw_robot_pose"]
            map_frame = latest_state["map"]["frame_id"] if latest_state["map"] is not None else None
            if raw_pose is not None:
                latest_state["robot_pose"] = self.transform_robot_pose_to_map(raw_pose, map_frame)

    def transform_robot_pose_to_map(
        self,
        raw_pose: Dict[str, Any],
        map_frame: Optional[str],
    ) -> Dict[str, Any]:
        return self.transform_pose_to_frame(raw_pose, map_frame)

    def transform_pose_to_frame(
        self,
        pose: Dict[str, Any],
        target_frame: Optional[str],
    ) -> Dict[str, Any]:
        source_frame = pose.get("frame_id")

        if not target_frame or not source_frame or target_frame == source_frame:
            return pose

        direct = self.transforms.get((target_frame, source_frame))
        if direct is None:
            return pose

        transformed = compose_2d_pose(direct, pose)
        return {
            **pose,
            "frame_id": target_frame,
            "x": transformed["x"],
            "y": transformed["y"],
            "yaw": transformed["yaw"],
        }

    def publish_live_state(self) -> None:
        payload: Dict[str, Any] = {"type": "state"}
        changed = False
        with state_lock:
            robot_pose = latest_state["robot_pose"]
            goal_pose = latest_state["goal_pose"]
            return_home_goal_pose = latest_state["return_home_goal_pose"]
            queue_goals = latest_state["queue_goals"]
            queue_landmarks = attach_order_ids_to_queue_summary_locked(
                build_queue_landmark_summary_locked()
            )
            queue_status = latest_state["queue_status"]
            movement_gate_open = latest_state["movement_gate_open"]
            currently_dispensing = latest_state["currently_dispensing"]
            dispense_empty = latest_state["dispense_empty"]
            map_state = latest_state["map"]
            landmarks = latest_state["landmarks"]

        now = time.time()
        if robot_pose and now - self._last_pose_sent >= 0.2:
            payload["robot_pose"] = robot_pose
            self._last_pose_sent = now
            changed = True
        if goal_pose and goal_pose != self._last_goal_sent:
            payload["goal_pose"] = goal_pose
            self._last_goal_sent = goal_pose
            changed = True
        if return_home_goal_pose != self._last_return_home_goal_sent:
            payload["return_home_goal_pose"] = return_home_goal_pose
            self._last_return_home_goal_sent = return_home_goal_pose
            changed = True
        if queue_goals != self._last_queue_goals_sent:
            payload["queue_goals"] = queue_goals
            self._last_queue_goals_sent = queue_goals
            changed = True
        if queue_landmarks != self._last_queue_landmarks_sent:
            payload["queue_landmarks"] = queue_landmarks
            self._last_queue_landmarks_sent = queue_landmarks
            changed = True
        if queue_status != self._last_queue_status_sent:
            payload["queue_status"] = queue_status
            self._last_queue_status_sent = queue_status
            changed = True
        if landmarks != self._last_landmarks_sent:
            payload["landmarks"] = landmarks
            self._last_landmarks_sent = landmarks
            changed = True
        payload["movement_gate_open"] = movement_gate_open
        payload["return_home_signal"] = movement_gate_open
        payload["currently_dispensing"] = currently_dispensing
        payload["dispense_empty"] = dispense_empty
        if self._map_dirty and map_state:
            payload["map"] = map_state
            self._map_dirty = False
            changed = True

        if changed:
            schedule_broadcast(payload)


def rclpy_thread_fn() -> None:
    try:
        rclpy.init()
    except RuntimeError:
        pass

    node = RosBridgeNode()
    from rclpy.executors import SingleThreadedExecutor

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    ros_node_holder["node"] = node
    ros_node_holder["executor"] = executor
    ros_node_holder["running"] = True

    try:
        executor.spin()
    except Exception:
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        ros_node_holder["running"] = False


_rclpy_thread = threading.Thread(target=rclpy_thread_fn, daemon=True)
_rclpy_thread.start()

_wait_count = 0
while _wait_count < 20 and ros_node_holder.get("node") is None:
    time.sleep(0.05)
    _wait_count += 1

app = FastAPI()
app.mount("/static", StaticFiles(directory="web_static"), name="static")


@app.on_event("startup")
async def startup_event() -> None:
    async_loop_holder["loop"] = asyncio.get_running_loop()


@app.middleware("http")
async def disable_html_cache(request: Request, call_next):
    response = await call_next(request)
    if request.url.path in {"/", "/static/index.html"}:
        response.headers["Cache-Control"] = "no-store, max-age=0"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"
    return response


@app.get("/")
async def root_redirect():
    return RedirectResponse(url="/static/index.html")


@app.get("/user_interface/{landmark_id}")
async def user_interface_page(landmark_id: int):
    landmark = find_landmark_by_id(landmark_id)
    if landmark is None or not landmark.get("enabled", True):
        raise HTTPException(status_code=404, detail="Unknown landmark")
    return FileResponse("web_static/user.html")


@app.get("/api/landmarks")
async def get_landmarks():
    with state_lock:
        ensure_landmarks_compacted_locked()
        return {"landmarks": latest_state["landmarks"]}


@app.get("/api/landmarks/{landmark_id}")
async def get_landmark(landmark_id: int):
    with state_lock:
        ensure_landmarks_compacted_locked()
    landmark = find_landmark_by_id(landmark_id)
    if landmark is None:
        raise HTTPException(status_code=404, detail="Unknown landmark")
    log_second_ui_get(f"Order UI GET /api/landmarks/{landmark_id}")
    return landmark


@app.post("/api/landmarks")
async def upsert_landmark(request: LandmarkRequest):
    with state_lock:
        ensure_landmarks_compacted_locked()
        landmarks = list(latest_state["landmarks"])
        landmark_id = request.id if request.id is not None else next_landmark_id(landmarks)
        record = sanitize_landmark(
            {
                "id": landmark_id,
                "label": request.label or f"Landmark {landmark_id}",
                "x": request.x,
                "y": request.y,
                "yaw": request.yaw,
                "enabled": request.enabled,
            }
        )
        landmarks = [item for item in landmarks if int(item["id"]) != int(landmark_id)]
        landmarks.append(record)
        landmarks, _ = compact_landmark_ids(landmarks)
        save_landmarks(landmarks)
        latest_state["landmarks"] = landmarks
        reconcile_orders_locked()
    schedule_broadcast({"type": "landmarks", "landmarks": snapshot_state()["landmarks"]})
    return record


@app.delete("/api/landmarks/{landmark_id}")
async def delete_landmark(landmark_id: int):
    with state_lock:
        ensure_landmarks_compacted_locked()
        remaining = [item for item in latest_state["landmarks"] if int(item["id"]) != landmark_id]
        if len(remaining) == len(latest_state["landmarks"]):
            raise HTTPException(status_code=404, detail="Unknown landmark")
        landmarks, remap = compact_landmark_ids(remaining)
        save_landmarks(landmarks)
        latest_state["landmarks"] = landmarks
        for order in orders_state["orders"].values():
            if int(order["landmark_id"]) == landmark_id and order["status"] not in {
                "completed",
                "failed",
                "canceled",
            }:
                order["status"] = "canceled"
                order["queue_label"] = None
                order["queue_position"] = None
                continue
            new_landmark_id = remap.get(int(order["landmark_id"]))
            if new_landmark_id is not None:
                order["landmark_id"] = new_landmark_id
                matched_landmark = next(
                    (item for item in landmarks if int(item["id"]) == new_landmark_id),
                    None,
                )
                if matched_landmark is not None:
                    order["landmark_label"] = str(matched_landmark["label"])
        reconcile_orders_locked()
    schedule_broadcast({"type": "landmarks", "landmarks": snapshot_state()["landmarks"]})
    return {
        "ok": True,
        "deleted": landmark_id,
        "landmarks": snapshot_state()["landmarks"],
    }


@app.get("/api/queue")
async def get_queue():
    publish_uart_for_executing_orders()
    with state_lock:
        reconcile_orders_locked()
        queue_landmarks = attach_order_ids_to_queue_summary_locked(
            build_queue_landmark_summary_locked()
        )
        active = next((item for item in queue_landmarks if item["label"] == ACTIVE_QUEUE_LABEL), None)
        pending = [item for item in queue_landmarks if item["label"] != ACTIVE_QUEUE_LABEL]
        response = {
            "queue_status": latest_state["queue_status"],
            "currently_dispensing": latest_state["currently_dispensing"],
            "active": active,
            "pending": pending,
            "landmark_match_radius_m": LANDMARK_MATCH_RADIUS_METERS,
        }
    log_second_ui_get("Order UI GET /api/queue")
    return response


@app.post("/api/orders")
async def create_order(request: OrderRequest):
    """
    Creates a drink order:
      - Validates landmark + flavor
      - Stores order in memory
      - Publishes navigation goal to that landmark
      - Order lifecycle is later driven by queue + ROS
    """
    landmark = find_landmark_by_id(request.landmark_id)
    if landmark is None or not landmark.get("enabled", True):
        raise HTTPException(status_code=404, detail="Unknown landmark")
    if request.flavor not in (1, 2, 3):
        raise HTTPException(status_code=400, detail="Flavor must be 1, 2, or 3")

    node = ros_node_holder.get("node")
    if node is None:
        raise HTTPException(status_code=503, detail="ROS node not ready")

    with state_lock:
        order_id = int(orders_state["next_order_id"])
        orders_state["next_order_id"] += 1
        order = {
            "order_id": order_id,
            "landmark_id": int(landmark["id"]),
            "landmark_label": str(landmark["label"]),
            "flavor": int(request.flavor),
            "status": "submitted",
            "queue_label": None,
            "queue_position": None,
            "uart_published": False,
            "created_at": time.time(),
            "note": request.note,
        }
        orders_state["orders"][order_id] = order

    try:
        pose = make_pose_stamped(
            landmark["x"],
            landmark["y"],
            landmark.get("yaw", 0.0),
            frame_id="map",
        )
        node.publish_pose(pose, source=f"order:{request.landmark_id}")
    except Exception as exc:
        with state_lock:
            order["status"] = "failed"
            order["queue_label"] = None
            order["queue_position"] = None
        raise HTTPException(status_code=500, detail=str(exc))

    with state_lock:
        reconcile_orders_locked()
        public_order = order_to_public_dict(order)

    log_activity(
        f"Order UI POST /api/orders -> order {order_id} landmark {landmark['id']} flavor {request.flavor}"
    )

    return {
        "ok": True,
        "order": public_order,
        "landmark": landmark,
    }


@app.get("/api/orders/{order_id}")
async def get_order(order_id: int):
    publish_uart_for_executing_orders()
    with state_lock:
        reconcile_orders_locked()
        order = orders_state["orders"].get(order_id)
        if order is None:
            raise HTTPException(status_code=404, detail="Unknown order")
        public_order = order_to_public_dict(order)
    log_second_ui_get(f"Order UI GET /api/orders/{order_id}")
    return public_order


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    """
    Real-time bidirectional control channel

    Supports:
      - sending goals (xy)
      - cancel goals
      - gate control
      - flavor selection
      - dispenser query
      - full state sync
    """
    await manager.connect(ws)
    await manager.send_json(
        ws,
        {
            "type": "hello",
            "ok": True,
            "msg": "connected",
            **snapshot_state(),
        },
    )
    try:
        while True:
            text = await ws.receive_text()
            try:
                data = json.loads(text)
            except Exception as exc:
                await manager.send_json(
                    ws,
                    {"type": "error", "ok": False, "error": "invalid_json", "detail": str(exc)},
                )
                continue

            msg_type = data.get("type")
            frame_from_msg = data.get("frame_id")

            if msg_type == "xy":
                x = data.get("x")
                y = data.get("y")
                yaw = data.get("yaw", 0.0)
                if x is None or y is None:
                    await manager.send_json(
                        ws, {"type": "error", "ok": False, "error": "missing_xy"}
                    )
                    continue
                try:
                    ps = make_pose_stamped(float(x), float(y), float(yaw), frame_from_msg)
                    ros_node_holder["node"].publish_pose(ps, "xy")
                    await manager.send_json(
                        ws,
                        {
                            "type": "ack",
                            "ok": True,
                            "published": True,
                            "source": "xy",
                            "goal_pose": pose_to_dict(ps),
                        },
                    )
                except Exception as exc:
                    await manager.send_json(
                        ws,
                        {
                            "type": "error",
                            "ok": False,
                            "error": "publish_failed",
                            "detail": str(exc),
                        },
                    )

            elif msg_type == "cancel_xy":
                x = data.get("x")
                y = data.get("y")
                yaw = data.get("yaw", 0.0)
                if x is None or y is None:
                    await manager.send_json(
                        ws, {"type": "error", "ok": False, "error": "missing_xy"}
                    )
                    continue
                try:
                    ps = make_pose_stamped(float(x), float(y), float(yaw), frame_from_msg)
                    ros_node_holder["node"].publish_cancel_pose(ps, "cancel_xy")
                    await manager.send_json(
                        ws,
                        {
                            "type": "ack",
                            "ok": True,
                            "published": True,
                            "source": "cancel_xy",
                            "cancel_pose": pose_to_dict(ps),
                        },
                    )
                except Exception as exc:
                    await manager.send_json(
                        ws,
                        {
                            "type": "error",
                            "ok": False,
                            "error": "cancel_publish_failed",
                            "detail": str(exc),
                        },
                    )

            elif msg_type == "gate_state":
                try:
                    state_value = bool(data.get("state"))
                    ros_node_holder["node"].publish_gate_state(state_value)
                    await manager.send_json(
                        ws,
                        {
                            "type": "ack",
                            "ok": True,
                            "published": True,
                            "source": "gate_state",
                            "state": state_value,
                        },
                    )
                except Exception as exc:
                    await manager.send_json(
                        ws,
                        {
                            "type": "error",
                            "ok": False,
                            "error": "gate_state_publish_failed",
                            "detail": str(exc),
                        },
                    )

            elif msg_type == "currently_dispensing":
                try:
                    state_value = bool(data.get("state"))
                    ros_node_holder["node"].publish_currently_dispensing(state_value)
                    await manager.send_json(
                        ws,
                        {
                            "type": "ack",
                            "ok": True,
                            "published": True,
                            "source": "currently_dispensing",
                            "state": state_value,
                        },
                    )
                except Exception as exc:
                    await manager.send_json(
                        ws,
                        {
                            "type": "error",
                            "ok": False,
                            "error": "currently_dispensing_publish_failed",
                            "detail": str(exc),
                        },
                    )

            elif msg_type == "set_home":
                try:
                    ros_node_holder["node"].publish_set_home_here()
                    await manager.send_json(
                        ws,
                        {
                            "type": "ack",
                            "ok": True,
                            "published": True,
                            "source": "set_home",
                        },
                    )
                except Exception as exc:
                    await manager.send_json(
                        ws,
                        {
                            "type": "error",
                            "ok": False,
                            "error": "set_home_publish_failed",
                            "detail": str(exc),
                        },
                    )

            elif msg_type == "flavor_selection":
                try:
                    selection = int(data.get("selection"))
                    ros_node_holder["node"].publish_flavor_selection(selection)
                    await manager.send_json(
                        ws,
                        {
                            "type": "ack",
                            "ok": True,
                            "published": True,
                            "source": "flavor_selection",
                            "selection": selection,
                        },
                    )
                except Exception as exc:
                    await manager.send_json(
                        ws,
                        {
                            "type": "error",
                            "ok": False,
                            "error": "flavor_selection_publish_failed",
                            "detail": str(exc),
                        },
                    )

            elif msg_type == "dispense_query":
                try:
                    ros_node_holder["node"].query_dispense_levels()
                    await manager.send_json(
                        ws,
                        {
                            "type": "ack",
                            "ok": True,
                            "published": True,
                            "source": "dispense_query",
                        },
                    )
                except Exception as exc:
                    await manager.send_json(
                        ws,
                        {
                            "type": "error",
                            "ok": False,
                            "error": "dispense_query_failed",
                            "detail": str(exc),
                        },
                    )

            elif msg_type == "get_state":
                await manager.send_json(ws, {"type": "state", **snapshot_state()})

            else:
                await manager.send_json(
                    ws, {"type": "error", "ok": False, "error": "unknown_type"}
                )
    except WebSocketDisconnect:
        await manager.disconnect(ws)


@app.get("/healthz")
async def healthz():
    return {
        "ok": ros_node_holder.get("running", False),
        "target_topic": TARGET_TOPIC,
        "home_target_topic": HOME_TARGET_TOPIC,
        "movement_gate_topic": MOVEMENT_GATE_TOPIC,
        "return_home_topic": LEGACY_RETURN_HOME_TOPIC,
        "set_home_topic": SET_HOME_TOPIC,
        "flavor_selection_topic": FLAVOR_SELECTION_TOPIC,
        "currently_dispensing_topic": CURRENTLY_DISPENSING_TOPIC,
        "dispense_empty_topic": DISPENSE_EMPTY_TOPIC,
        "dispense_uart_event_topic": DISPENSE_UART_EVENT_TOPIC,
        "dispense_uart_command_topic": DISPENSE_UART_COMMAND_TOPIC,
        "frame_id": FRAME_ID,
        "map_topic": MAP_TOPIC,
        "odom_topic": ODOM_TOPIC,
    }


def run_uvicorn() -> None:
    uvicorn.run(app, host=HTTP_HOST, port=HTTP_PORT, log_level="info")


if __name__ == "__main__":
    print(
        f"Starting web server on {HTTP_HOST}:{HTTP_PORT} "
        f"(TARGET_TOPIC={TARGET_TOPIC}, HOME_TARGET_TOPIC={HOME_TARGET_TOPIC}, "
        f"MOVEMENT_GATE_TOPIC={MOVEMENT_GATE_TOPIC}, LEGACY_RETURN_HOME_TOPIC={LEGACY_RETURN_HOME_TOPIC}, "
        f"SET_HOME_TOPIC={SET_HOME_TOPIC}, "
        f"FRAME_ID={FRAME_ID}, MAP_TOPIC={MAP_TOPIC}, ODOM_TOPIC={ODOM_TOPIC})"
    )
    if not HAS_WEBSOCKET_RUNTIME:
        print(
            "WebSocket runtime dependency missing. Install with "
            "\"pip install -r requirements.txt\" or \"pip install 'uvicorn[standard]'\"."
        )
    try:
        run_uvicorn()
    except KeyboardInterrupt:
        print("Shutting down...")

    try:
        if ros_node_holder.get("executor"):
            ros_node_holder["executor"].shutdown()
    except Exception:
        pass
