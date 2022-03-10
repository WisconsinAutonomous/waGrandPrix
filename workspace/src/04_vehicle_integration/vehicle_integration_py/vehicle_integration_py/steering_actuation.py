# General ROS imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_control_msgs.msg import SteeringCommand

class SteeringActuation(Node):

    def __init__(self):
        super().__init__('steering_actuation')

        self.logger = rclpy.logging.get_logger(self.get_name())
        
        # ------------
        # Parse params
        # ------------
        steering_cmd_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the steering command msg will be shipped on.")
        self.declare_parameter("steering_cmd_topic", "/actuation/commands/steering", steering_cmd_descriptor)
        self.steering_cmd_topic = self.get_parameter("steering_cmd_topic").value

        # ------------
        # ROS Entities
        # ------------

        # Create subscriber handles
        self.subscriber_handles = {}
        self.subscriber_handles[self.steering_cmd_topic] = self.create_subscription(SteeringCommand, self.steering_cmd_topic, self.steering_cmd_callback, 1)

    def steering_cmd_callback(self, msg):
        """
        Callback for the brake_cmd topic.
        """
        self.get_logger().info(f"Received {msg} on topic {self.steering_cmd_topic}")

