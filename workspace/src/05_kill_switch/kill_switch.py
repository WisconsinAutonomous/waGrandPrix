# General ROS imports
from pickle import FALSE, TRUE
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import bool


class KILL_PUBLISHER(Node):

    def __init__(self):
        super().__init__('kill_publisher')
        self.publisher_ = self.create_publisher(bool, 'kill', 10)
        while TRUE:
            

    def publish(self):
        msg = bool()
        msg.data = FALSE
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing kill: "%b"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    kill_publisher = KILL_PUBLISHER()

    rclpy.spin(kill_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    kill_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()