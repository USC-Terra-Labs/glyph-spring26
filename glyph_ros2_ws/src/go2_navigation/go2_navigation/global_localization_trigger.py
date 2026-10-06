"""
    global_localization_trigger is a one-shot AMCL helper.
    After a configurable delay, it waits for /reinitialize_global_localization,
    calls it once, logs success/failure, and cancels its timer so it never repeats.
    
    It is gated by global_localization:=true in go2_nav_robot_localization.launch
    (false by default, not recommended for use due to suboptimal performance)
"""

import rclpy  # ROS2 Python client library
from rclpy.node import Node  # Base class for ROS2 nodes
from std_srvs.srv import Empty  # Service type with no request/response fields


class GlobalLocalizationTrigger(Node):
    def __init__(self) -> None:
        # Initialize node with a unique name
        super().__init__("go2_global_localization_trigger")

        # Declare configurable parameters
        self.declare_parameter("service_name", "/reinitialize_global_localization")  # AMCL reset service
        self.declare_parameter("delay_sec", 8.0)  # Delay before attempting trigger

        # Retrieve parameter values
        self.service_name = str(self.get_parameter("service_name").value)
        delay_sec = float(self.get_parameter("delay_sec").value)

        # Create a client to call the AMCL global localization service
        self.client = self.create_client(Empty, self.service_name)

        # Create a one-shot timer that fires after delay_sec
        # (it will cancel itself after first successful trigger)
        self.timer = self.create_timer(delay_sec, self._trigger_once)

        # Internal flag to ensure we only send the request once
        self.sent = False

    def _trigger_once(self) -> None:
        """
        Timer callback that attempts to call the global localization service once.
        Waits for service availability, then sends request and cancels timer.
        """

        # If already triggered, do nothing
        if self.sent:
            return

        # Check if the service is available (non-blocking short wait)
        if not self.client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info(
                f"Waiting for AMCL global localization service '{self.service_name}'"
            )
            return

        # Call the service asynchronously (Empty request)
        future = self.client.call_async(Empty.Request())

        # Register callback to handle response
        future.add_done_callback(self._handle_response)

        # Mark as sent so we never call again
        self.sent = True

        # Cancel the timer so this function will not be invoked again
        self.timer.cancel()

        # Log request
        self.get_logger().info("Requested AMCL global localization")

    def _handle_response(self, future) -> None:
        """
        Callback for handling the async service response.
        Logs success or failure.
        """
        try:
            # Attempt to retrieve result (will raise if service failed)
            future.result()
            self.get_logger().info("AMCL global localization request completed")
        except Exception as exc:  # pragma: no cover - ROS async error path
            # Catch and log any exception from the async call
            self.get_logger().error(f"AMCL global localization request failed: {exc}")


def main() -> None:
    # Initialize ROS2 runtime
    rclpy.init()

    # Create node instance
    node = GlobalLocalizationTrigger()

    try:
        # Spin to process timer + async callbacks
        rclpy.spin(node)
    finally:
        # Cleanup resources on shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
