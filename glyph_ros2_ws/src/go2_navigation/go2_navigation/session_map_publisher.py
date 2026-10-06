"""
    session_map_publisher merges the static map with the live global costmap into a derived /session_map. 
    It waits until it has both occupancy grids, refuses to merge if geometry differs, 
    then converts overlay cells above overlay_threshold to occupied, normalizing the rest to binary occupied/free based on the base map threshold. 
    
    TLDR: it creates a transient-local occupancy grid, and is launched in go2_nav_robot_localization.launch.
"""

from copy import deepcopy  # Used to clone the base map safely before modifying

import rclpy  # ROS2 Python client library
from nav_msgs.msg import OccupancyGrid  # Standard map message type
from rclpy.node import Node  # Base class for ROS2 nodes
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # QoS settings


class SessionMapPublisher(Node):
    def __init__(self) -> None:
        # Initialize node
        super().__init__("go2_session_map_publisher")

        # Declare configurable parameters
        self.declare_parameter("base_map_topic", "/map")  # Static map (latched)
        self.declare_parameter("overlay_topic", "/global_costmap/costmap")  # Live costmap
        self.declare_parameter("output_topic", "/session_map")  # Merged output
        self.declare_parameter("occupied_threshold", 50)  # Threshold for base map occupancy
        self.declare_parameter("overlay_threshold", 20)  # Threshold for overlay occupancy

        # Retrieve parameter values
        base_map_topic = str(self.get_parameter("base_map_topic").value)
        overlay_topic = str(self.get_parameter("overlay_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self._overlay_threshold = int(self.get_parameter("overlay_threshold").value)

        # QoS for static map (latched / transient local so late subscribers get last value)
        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # QoS for live overlay (no latching, just most recent data)
        volatile_qos = QoSProfile(depth=1)
        volatile_qos.reliability = ReliabilityPolicy.RELIABLE
        volatile_qos.durability = DurabilityPolicy.VOLATILE

        # Publisher for merged session map (latched so others can immediately consume it)
        self._publisher = self.create_publisher(OccupancyGrid, output_topic, transient_qos)

        # Internal storage for latest maps
        self._base_map = None
        self._overlay = None

        # Subscribe to base (static) map
        self.create_subscription(
            OccupancyGrid,
            base_map_topic,
            self._handle_base_map,
            transient_qos,
        )

        # Subscribe to overlay (dynamic global costmap)
        self.create_subscription(
            OccupancyGrid,
            overlay_topic,
            self._handle_overlay,
            volatile_qos,
        )

    def _handle_base_map(self, msg: OccupancyGrid) -> None:
        """
        Callback for receiving the static base map.
        """
        self._base_map = msg
        self._publish_if_ready()

    def _handle_overlay(self, msg: OccupancyGrid) -> None:
        """
        Callback for receiving the live global costmap overlay.
        """
        self._overlay = msg
        self._publish_if_ready()

    def _publish_if_ready(self) -> None:
        """
        Attempt to merge and publish maps if both inputs are available
        and geometrically compatible.
        """
        # Ensure both maps have been received
        if self._base_map is None or self._overlay is None:
            return

        # Check that both maps align (same resolution, size, origin)
        if not self._compatible(self._base_map, self._overlay):
            self.get_logger().warn(
                "Base map and global costmap geometry differ; not publishing merged session map"
            )
            return

        # Create a copy of the base map to modify
        merged = deepcopy(self._base_map)

        # Update timestamp so consumers know it's fresh
        merged.header.stamp = self.get_clock().now().to_msg()

        # Convert map data to mutable list
        merged_data = list(merged.data)
        overlay_data = self._overlay.data

        # Iterate over every cell
        for i, base_value in enumerate(merged_data):
            overlay_value = int(overlay_data[i])

            # Ignore unknown overlay cells
            if overlay_value < 0:
                continue

            # If overlay says "occupied enough", override to occupied
            if overlay_value >= self._overlay_threshold:
                merged_data[i] = 100

            # If base map cell is unknown, treat as free (0)
            elif base_value < 0:
                merged_data[i] = 0

            # If base map says occupied, keep it occupied
            elif int(base_value) >= self._occupied_threshold:
                merged_data[i] = 100

            # Otherwise mark as free
            else:
                merged_data[i] = 0

        # Assign modified data back
        merged.data = merged_data

        # Publish merged session map
        self._publisher.publish(merged)

    @staticmethod
    def _compatible(base_map: OccupancyGrid, overlay: OccupancyGrid) -> bool:
        """
        Check if two occupancy grids are compatible for merging.
        They must have identical dimensions, resolution, and origin.
        """
        base_info = base_map.info
        overlay_info = overlay.info

        return (
            base_info.width == overlay_info.width
            and base_info.height == overlay_info.height
            and abs(base_info.resolution - overlay_info.resolution) < 1e-6
            and abs(base_info.origin.position.x - overlay_info.origin.position.x) < 1e-6
            and abs(base_info.origin.position.y - overlay_info.origin.position.y) < 1e-6
        )


def main() -> None:
    # Initialize ROS2 runtime
    rclpy.init()

    # Create node instance
    node = SessionMapPublisher()

    try:
        # Spin to process subscriptions and publishing
        rclpy.spin(node)
    finally:
        # Cleanup resources on shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
