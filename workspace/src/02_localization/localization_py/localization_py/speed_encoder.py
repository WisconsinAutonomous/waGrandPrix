import serial, math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from wagrandprix_control_msgs import ThrottleCommand
from wagrandprix_vehicle_msgs import VehicleState

# TODO may need to change
TEETH_NUM = 60
DIAMETER = 0.3048 # meters
VALUE_TO_FREQ_COEF = 3.0542

VALUE_TO_SPEED_COEF = VALUE_TO_FREQ_COEF * (math.pi * DIAMETER) / TEETH_NUM  


class SpeedEncoder(Node):
    
  def __init__(self):
    super().__init__('encoder')

    self.logger = rclpy.logging.get_logger(self.get_name())

    # ------------
    # Parse params
    # ------------
    vehicle_state_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic the vehicle state estimation will be shipped on.")
    self.declare_parameter("vehicle_state_topic", "vehicle/state", vehicle_state_descriptor)
    self.vehicle_state_topic = self.get_parameter("vehicle_state_topic").value

    # Create publisher handles
    self.publisher_handles = {}
    self.publisher_handles[self.vehicle_state_topic] = self.create_publisher(VehicleState, self.vehicle_state_topic, 1)

    # Timer to make sure we publish at a controlled rate
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    # It is important that i is a float
    self.speed = 0.0

    # Set up serial connection with Arduino
    self.ser = serial.Serial("/dev/ttyACM0", baudrate=9600, timeout=None)


def timer_callback(self):
    msg = VehicleState()
    # serial speed reading from Arduino
    self.speed = get_speed(self)
    msg.twist.value = self.speed
    self.publisher_handles[self.throttle_cmd_topic].publish(msg)
    self.get_logger().info(f"Sent {msg} on topic {self.throttle_cmd_topic}")


def get_speed(self) -> float:
    """
    return speed in m/s
    """
    self.ser.write(b'\1')
    read_value = self.ser.read_until(b'\n').endcode("utf-8")
    value = int(read_value) * VALUE_TO_SPEED_COEF
    return value


def main(args=None):
    rclpy.init(args=args)

    encoder = SpeedEncoder()

    rclpy.spin(encoder)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    encoder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    
