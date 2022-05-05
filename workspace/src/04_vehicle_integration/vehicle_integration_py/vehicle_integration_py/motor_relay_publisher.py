import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_vehicle_msgs.msg import MotorPower


class MotorRelayPublisher(Node):

    def __init__(self):
        super().__init__('steering_publisher')

        self.logger = rclpy.logging.get_logger(self.get_name())


        # ------------
        # Parse params
        # ------------
        motor_relay_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the motor relay msg will be shipped on.")
        self.declare_parameter("motor_relay_topic", "/control/motor_relay", motor_relay_descriptor)
        self.motor_relay_topic = self.get_parameter("motor_relay_topic").value

        # Create publisher handles
        self.publisher_handles = {}
        self.publisher_handles[self.motor_relay_topic] = self.create_publisher(MotorPower, self.motor_relay_topic, 1)

        # Timer to make sure we publish at a controlled rate
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.0


    def timer_callback(self):
        msg = MotorPower()
        msg.value = self.i
        self.publisher_handles[self.motor_relay_topic].publish(msg)
        self.get_logger().info(f"Sent {msg} on topic {self.motor_relay_topic}")
        if self.i < -1.0:
            self.i = 2.0
        elif self.i > 1.0:
            self.i = 0.0
        else:
            self.i = -2.0






def main(args=None):
    rclpy.init(args=args)

    motor_relay_publisher = MotorRelayPublisher()

    rclpy.spin(motor_relay_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_relay_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()