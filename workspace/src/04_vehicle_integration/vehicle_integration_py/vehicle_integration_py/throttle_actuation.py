# General ROS imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_interfaces.msg import ThrottleCommand

class ThrottleActuation(Node):

    def __init__(self):
        super().__init__('throttle_actuation')

        self.logger = rclpy.logging.get_logger(self.get_name())
        
        # ------------
        # Parse params
        # ------------
        throttle_cmd_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the throttle command msg will be shipped on.")
        self.declare_parameter("throttle_cmd_topic", "/actuation/commands/throttle", throttle_cmd_descriptor)
        self.throttle_cmd_topic = self.get_parameter("throttle_cmd_topic").value

        # ------------
        # ROS Entities
        # ------------

        # Create subscriber handles
        self.subscriber_handles = {}
        self.subscriber_handles[self.throttle_cmd_topic] = self.create_subscription(ThrottleCommand, self.throttle_cmd_topic, self.throttle_cmd_callback, 1)

    def throttle_cmd_callback(self, msg):
        """
        Callback for the throttle_cmd topic.
        """
        self.get_logger().info(f"Received {msg} on topic {self.throttle_cmd_topic}")
