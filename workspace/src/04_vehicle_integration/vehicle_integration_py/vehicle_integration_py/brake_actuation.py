# General ROS imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_interfaces.msg import BrakeCommand

class BrakeActuation(Node):

    def __init__(self):
        super().__init__('brake_actuation')

        self.logger = rclpy.logging.get_logger(self.get_name())
        
        # ------------
        # Parse params
        # ------------
        brake_cmd_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the brake command msg will be shipped on.")
        self.declare_parameter("brake_cmd_topic", "/actuation/commands/brake", brake_cmd_descriptor)
        self.brake_cmd_topic = self.get_parameter("brake_cmd_topic").value

        # ------------
        # ROS Entities
        # ------------

        # Create subscriber handles
        self.subscriber_handles = {}
        self.subscriber_handles[self.brake_cmd_topic] = self.create_subscription(BrakeCommand, self.brake_cmd_topic, self.brake_cmd_callback, 1)

    def brake_cmd_callback(self, msg):
        """
        Callback for the brake_cmd topic.
        """
        self.get_logger().info(f"Received {msg} on topic {self.brake_cmd_topic}")
