"""
    sim_body_motion_controller is a motion synthesizer for simulated “body motions.”
    It subscribes to a string mode topic, accepts only stop, dance1, and dance2, 
    then publishes synthetic Twist commands plus two state topics. 

    dance1: sinusoidal yaw twist
    dance2: forward/back bending-style linear oscillation 
    stop: publishes one zero twist then latches state 0

    In go2_nav_rlsar_localization.launch.py /sim_body_motion -> /cmd_vel. 
    In go2_nav_robot_localization.launch.py /body_motion -> /cmd_vel_dance.
"""

import math  # Used for sinusoidal motion generation

import rclpy  # ROS2 Python client library
from geometry_msgs.msg import Twist  # Velocity command message
from rclpy.node import Node  # Base class for ROS2 nodes
from std_msgs.msg import Float32, String, UInt8  # Simple message types for state topics


class SimBodyMotionController(Node):
    def __init__(self) -> None:
        # Initialize the ROS node with a unique name
        super().__init__("go2_sim_body_motion_controller")

        # Declare configurable ROS parameters with default values
        self.declare_parameter("motion_topic", "/sim_body_motion")  # Input command topic
        self.declare_parameter("state_topic", "/body_motion_state")  # Discrete state output
        self.declare_parameter("state_plot_topic", "/body_motion_state_plot")  # Float state (for plotting)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")  # Output velocity command topic
        self.declare_parameter("twist_angular_speed", 0.60)  # Amplitude of angular velocity for dance1
        self.declare_parameter("twist_frequency_hz", 0.65)  # Frequency of oscillation
        self.declare_parameter("bend_speed", 0.085)  # Linear speed for bending motion (dance2)
        self.declare_parameter("bend_angular_speed", 0.0)  # Optional constant angular speed during bend
        self.declare_parameter("publish_hz", 20.0)  # Control loop frequency

        # Retrieve parameter values
        motion_topic = str(self.get_parameter("motion_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        state_plot_topic = str(self.get_parameter("state_plot_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.twist_angular_speed = float(self.get_parameter("twist_angular_speed").value)
        self.twist_frequency_hz = float(self.get_parameter("twist_frequency_hz").value)
        self.bend_speed = float(self.get_parameter("bend_speed").value)
        self.bend_angular_speed = float(self.get_parameter("bend_angular_speed").value)

        # Ensure publish rate is not too low (minimum 5 Hz)
        publish_hz = max(5.0, float(self.get_parameter("publish_hz").value))

        # Create publishers for velocity command and state outputs
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(UInt8, state_topic, 10)
        self.state_plot_pub = self.create_publisher(Float32, state_plot_topic, 10)

        # Subscribe to motion command topic
        self.create_subscription(String, motion_topic, self._on_motion_command, 10)

        # Internal state tracking
        self.current_mode = "stop"  # Default mode
        self._last_published_stop = False  # Prevent repeated zero-twist publishing

        # Timer drives the control loop at the desired frequency
        self.timer = self.create_timer(1.0 / publish_hz, self._tick)

    def _on_motion_command(self, msg: String) -> None:
        """
        Callback for incoming motion commands.
        Accepts only 'stop', 'dance1', 'dance2'.
        Maps legacy 'arrival_twist' to 'dance1'.
        """
        mode = msg.data.strip().lower()  # Normalize input

        # Backward compatibility mapping
        if mode == "arrival_twist":
            mode = "dance1"

        # Validate mode
        if mode not in {"stop", "dance1", "dance2"}:
            self.get_logger().warn(f"Ignoring unsupported body motion mode '{msg.data}'")
            return

        # Update state
        self.current_mode = mode
        self._last_published_stop = False  # Allow publishing again if switching from stop

    def _tick(self) -> None:
        """
        Periodic control loop.
        Generates motion commands based on the current mode.
        """
        state = UInt8()       # Discrete state indicator
        state_plot = Float32()  # Float version for plotting/visualization

        # Handle STOP mode
        if self.current_mode == "stop":
            # Publish a single zero Twist to stop motion
            if not self._last_published_stop:
                self.cmd_pub.publish(Twist())

            # Publish state = 0
            state.data = 0
            state_plot.data = 0.0
            self.state_pub.publish(state)
            self.state_plot_pub.publish(state_plot)

            # Mark that stop command has been sent
            self._last_published_stop = True
            return

        # Get current time in seconds for phase computation
        now_sec = self.get_clock().now().nanoseconds / 1e9

        cmd = Twist()  # Output velocity command

        # DANCE1: sinusoidal angular (yaw) motion
        if self.current_mode == "dance1":
            # Compute sinusoidal phase
            phase = math.sin(2.0 * math.pi * self.twist_frequency_hz * now_sec)

            # Apply to angular velocity (z axis = yaw)
            cmd.angular.z = self.twist_angular_speed * phase

            state.data = 1
            state_plot.data = 1.0

        # DANCE2: bending-style forward/back oscillation
        else:
            # Slightly lower frequency than dance1 (scaled by 0.75)
            phase = 2.0 * math.pi * (self.twist_frequency_hz * 0.75) * now_sec
            bend_phase = math.sin(phase)

            # Nonlinear shaping: x * |x| gives smoother "push" motion
            cmd.linear.x = self.bend_speed * bend_phase * abs(bend_phase)

            # Optional constant angular component
            cmd.angular.z = self.bend_angular_speed

            state.data = 2
            state_plot.data = 2.0

        # Publish command and state
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(state)
        self.state_plot_pub.publish(state_plot)

        # Reset stop flag since we are actively publishing motion
        self._last_published_stop = False


def main(args=None) -> None:
    # Initialize ROS2
    rclpy.init(args=args)

    # Create node instance
    node = SimBodyMotionController()

    try:
        # Keep node alive and processing callbacks
        rclpy.spin(node)
    finally:
        # Cleanup on shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
