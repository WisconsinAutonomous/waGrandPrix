# General ROS imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_control_msgs.msg import ThrottleCommand

# ------------
# port from ROS1
# ------------

from wagrandprix_utilities import scale_to_range, clamp
import serial
import time

class ThrottleActuation(Node):

    def __init__(self):
        super().__init__('throttle_actuation')

        self.logger = rclpy.logging.get_logger(self.get_name())
        
        # ------------
        # Parse params
        # ------------
        throttle_cmd_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the throttle command msg will be shipped on.")
        self.declare_parameter("throttle_cmd_topic", "/control/throttle", throttle_cmd_descriptor)
        self.throttle_cmd_topic = self.get_parameter("throttle_cmd_topic").value

        # ------------
        # port from ROS1
        # ------------

        # Make global so shutdown sequence has access
        global throttle_min, throttle_max, serial_interface, serial_port, baudrate 
        # Load params from actuator_throttle.launch file (if those parameters do not exist, defaults are used instead)
        self.declare_parameter("~throttle_min", 0)
        self.declare_parameter("~throttle_max", 4095)
        self.declare_parameter("~serial_port", "/dev/ttyACM0")
        self.declare_parameter("~baudrate", 115200)
        throttle_min = int(self.get_parameter("~throttle_min").value)
        throttle_max = int(self.get_parameter("~throttle_max").value)
        serial_port = self.get_parameter("~serial_port").value 
        baudrate = int(self.get_parameter("~baudrate").value)

        # Setup bus connection
        try:
            serial_interface = serial.Serial(serial_port, baudrate=baudrate) # Create an object to communicate through
        except FileNotFoundError as e:
            # Report if the USB connection cannot be made
            self.get_logger().fatal("Cannot connect to Arduino usb. Exiting...")
            self.get_logger().fatal(str(e))
            exit(1)
        
        # ------------
        # ROS Entities
        # ------------

        # Create subscriber handles
        self.subscriber_handles = {}
        self.subscriber_handles[self.throttle_cmd_topic] = self.create_subscription(ThrottleCommand, self.throttle_cmd_topic, self.throttle_cmd_callback, 1)


    def throttle_cmd_callback(self, msg):
        """
        Callback for the throttle_cmd topic.
        """
        self.get_logger().info(f"Received {msg} on topic {self.throttle_cmd_topic}")

        # ------------
        # port from ROS1
        # ------------

        throttle = msg.value

        # Rescale from [0,1] to acceptable throttle range
        scaled_throttle = self.scale_value(throttle)

        # Send to dac 
        self.write_value(scaled_throttle)

    # ------------
    # port from ROS1
    # ------------

    def scale_value(self, val):
        """
        Scales the passed value to an acceptable range for the device
        Param:
            val - Value to convert, [0, 1]
        """
        val = scale_to_range(val, 0, 1, throttle_min, throttle_max) # note that this only works if val is correctly [0,1]
        val = clamp(val, throttle_min, throttle_max) # double check control value in case val is not in [0,1] (i.e. if val is from a raw command line input) 
        
        return int(val)

    def write_value(self, val):
        """
        Write a value to the Arduino to send to the DAC.
        Param:
            val - 12bit integer value to write, should be within range [0, 4095]
        """
        command = "cmd_t:"+str(val)+'\n'
        serial_interface.write(command.encode()) # send the command as bytes

    # This function must be a bound method !!! 
    def on_shutdown_callback(self):
        shutdown_val = 0
        command = "cmd_t:"+str(shutdown_val)+'\n'
        serial_interface.write(command.encode()) # send the command as bytes


def main(args=None):
    rclpy.init()
    actuator = ThrottleActuation()
    rclpy.get_default_context().on_shutdown(actuator.on_shutdown_callback)
    rclpy.spin(actuator)


if __name__ == '__main__':
    main()
