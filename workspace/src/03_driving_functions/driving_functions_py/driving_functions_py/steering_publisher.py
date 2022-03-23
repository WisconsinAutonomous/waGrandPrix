import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_control_msgs.msg import SteeringCommand


class SteeringPublisher(Node):

    def __init__(self):
        super().__init__('steering_publisher')

        self.logger = rclpy.logging.get_logger(self.get_name())


        # ------------
        # Parse params
        # ------------
        steering_cmd_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the steering command msg will be shipped on.")
        self.declare_parameter("steering_cmd_topic", "/actuation/commands/steering", steering_cmd_descriptor)
        self.steering_cmd_topic = self.get_parameter("steering_cmd_topic").value

        # Create publisher handles
        self.publisher_handles = {}
        self.publisher_handles[self.steering_cmd_topic] = self.create_publisher(SteeringCommand, self.steering_cmd_topic, 1)

        # Timer to make sure we publish at a controlled rate
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.0


    def timer_callback(self):
        msg = SteeringCommand()
        msg.value = self.i
        self.publisher_handles[self.steering_cmd_topic].publish(msg)
        self.get_logger().info(f"Sent {msg} on topic {self.steering_cmd_topic}")
        # It is important that i is a float
        self.i += 0.1
        if self.i > 3:
            self.i = -1.0





def main(args=None):
    rclpy.init(args=args)

    steering_publisher = SteeringPublisher()

    rclpy.spin(steering_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    steering_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()