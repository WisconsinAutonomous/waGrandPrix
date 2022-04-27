import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_control_msgs.msg import BrakingCommand


class BrakePublisher(Node):

    def __init__(self):
        super().__init__('brake_publisher')

        self.logger = rclpy.logging.get_logger(self.get_name())


        # ------------
        # Parse params
        # ------------
        brake_cmd_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the brake command msg will be shipped on.")
        self.declare_parameter("brake_cmd_topic", "/control/braking", brake_cmd_descriptor)
        self.brake_cmd_topic = self.get_parameter("brake_cmd_topic").value

        # Create publisher handles
        self.publisher_handles = {}
        self.publisher_handles[self.brake_cmd_topic] = self.create_publisher(BrakingCommand, self.brake_cmd_topic, 1)

        # Timer to make sure we publish at a controlled rate
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # It is important that i is a float
        self.i = 0.0


    def timer_callback(self):
        msg = BrakingCommand()
        msg.value = self.i
        self.publisher_handles[self.brake_cmd_topic].publish(msg)
        self.get_logger().info(f"Sent {msg} on topic {self.brake_cmd_topic}")
        self.i += 0.05
        if self.i > 1:
            self.i = -0.2





def main(args=None):
    rclpy.init(args=args)

    brake_publisher = BrakePublisher()

    rclpy.spin(brake_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    brake_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()