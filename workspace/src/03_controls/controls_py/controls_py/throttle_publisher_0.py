import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_control_msgs.msg import ThrottleCommand


class ThrottlePublisher0(Node):

    def __init__(self):
        super().__init__('throttle_publisher')

        self.logger = rclpy.logging.get_logger(self.get_name())


        # ------------
        # Parse params
        # ------------
        throttle_cmd_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the throttle command msg will be shipped on.")
        self.declare_parameter("throttle_cmd_topic", "/controls/throttle", throttle_cmd_descriptor)
        self.throttle_cmd_topic = self.get_parameter("throttle_cmd_topic").value

        # Create publisher handles
        self.publisher_handles = {}
        self.publisher_handles[self.throttle_cmd_topic] = self.create_publisher(ThrottleCommand, self.throttle_cmd_topic, 1)

        # Timer to make sure we publish at a controlled rate
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # It is important that i is a float
        self.i = 0.0


    def timer_callback(self):
        msg = ThrottleCommand()
        msg.value = 0.0
        self.publisher_handles[self.throttle_cmd_topic].publish(msg)
        self.get_logger().info(f"Sent {msg} on topic {self.throttle_cmd_topic}")
        self.i += 0.05
        if self.i > 1.5:
            self.i = -0.5





def main(args=None):
    rclpy.init(args=args)

    throttle_publisher = ThrottlePublisher0()

    rclpy.spin(throttle_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    throttle_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()