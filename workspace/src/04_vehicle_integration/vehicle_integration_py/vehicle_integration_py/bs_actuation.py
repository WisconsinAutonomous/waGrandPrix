# General ROS imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from canlib import canlib, Frame
import threading
from .bs_brake_actuation import BSBrakeActuation
from .bs_steering_actuation import BSSteeringActuation

class BSActuation(Node):

    def __init__(self):
        super().__init__('bs_actuation')

        # can intialization
        self.get_logger().info("Initializing CAN messaging ...")
        self.ch = canlib.openChannel(
            channel=0,
            flags=canlib.Open.EXCLUSIVE | canlib.Open.REQUIRE_EXTENDED,
            bitrate= canlib.Bitrate.BITRATE_250K,
        )
        # Set the CAN bus driver type to NORMAL.
        self.ch.setBusOutputControl(canlib.Driver.NORMAL)
        # Activate the CAN chip.
        self.ch.busOn()

        self.thrd_stop = False
        self.lock = threading.Lock()

        # initialization of brake_actuation and steering_actuation 
        # should after the initialization of self.ch and self.thrd_stop        
        self.brake_actuation = BSBrakeActuation(self)
        self.steering_actuation = BSSteeringActuation(self)

    def can_write(self, msg):
        self.lock.acquire()
        self.ch.write(msg)
        self.lock.release()
        self.ch.writeSync(timeout=100)

def main(args=None):
    rclpy.init(args=args)
    bs_actuation = BSActuation()
    rclpy.spin(bs_actuation)

if __name__ == '__main__':
    main()
