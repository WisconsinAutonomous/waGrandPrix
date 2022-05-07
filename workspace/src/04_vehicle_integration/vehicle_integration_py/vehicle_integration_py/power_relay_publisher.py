import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_vehicle_msgs.msg import MotorPower, ActuatorPower


class PowerRelayPublisher(Node):

    def __init__(self):
        super().__init__('power_relay_publisher')

        self.logger = rclpy.logging.get_logger(self.get_name())


        # ------------
        # Parse params
        # ------------
        motor_relay_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the motor relay msg will be shipped on.")
        self.declare_parameter("motor_relay_topic", "/control/motor_relay", motor_relay_descriptor)
        self.motor_relay_topic = self.get_parameter("motor_relay_topic").value

        actuator_relay_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the actuator relay msg will be shipped on.")
        self.declare_parameter("actuator_relay_topic", "/control/actuator_relay", actuator_relay_descriptor)
        self.actuator_relay_topic = self.get_parameter("actuator_relay_topic").value

        # Create publisher handles
        self.publisher_handles = {}
        self.publisher_handles[self.motor_relay_topic] = self.create_publisher(MotorPower, self.motor_relay_topic, 1)
        self.publisher_handles[self.actuator_relay_topic] = self.create_publisher(ActuatorPower, self.actuator_relay_topic, 1)

        # Timer to make sure we publish at a controlled rate
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 2.0


    def timer_callback(self):
        msg = MotorPower()
        msg.value = self.i
        self.publisher_handles[self.motor_relay_topic].publish(msg)
        self.get_logger().info(f"Sent {msg} on topic {self.motor_relay_topic}")

        msg2 = ActuatorPower()
        msg2.value = self.i
        self.publisher_handles[self.actuator_relay_topic].publish(msg2)
        self.get_logger().info(f"Sent {msg2} on topic {self.actuator_relay_topic}")
        self.i = 0.0






def main(args=None):
    rclpy.init(args=args)

    motor_relay_publisher = PowerRelayPublisher()

    rclpy.spin(motor_relay_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_relay_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()