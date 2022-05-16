import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_vehicle_msgs import VehicleState


class SpeedEncoderSubscriber(Node):

    def __init__(self):
        super().__init__('speed_encoder_publisher')

        self.logger = rclpy.logging.get_logger(self.get_name())
        self.create_subscription(VehicleState, "vehicle_state_topic", self.callback, 1)


    def callback(msg):
        mps = msg.twist.value
        mph = mps * 2.23693629
        kph = mps * 3.6
        print(f"speed: f{mps:.2f} m/s, f{mph:.2f} mph, f{kph:.2f} kph")




def main(args=None):
    rclpy.init(args=args)

    encoder_subscriber = SpeedEncoderSubscriber()

    rclpy.spin(encoder_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    steering_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()