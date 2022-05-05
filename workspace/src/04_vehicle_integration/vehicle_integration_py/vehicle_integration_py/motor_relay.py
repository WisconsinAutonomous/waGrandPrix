# General ROS imports
import rclpy
from rclpy.node import Node
import time
import serial
import threading

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_vehicle_msgs.msg import MotorPower

class MotorRelay(Node):

    def __init__(self):
        super().__init__('motor_relay')

        motor_relay_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the motor relay msg will be shipped on.")
        self.declare_parameter("motor_relay_topic", "/control/motor_relay", motor_relay_descriptor)
        self.motor_relay_topic = self.get_parameter("motor_relay_topic").value

        self.subscriber_handles = {}
        self.subscriber_handles[self.motor_relay_topic] = self.create_subscription(MotorPower, self.motor_relay_topic, self.callback, 1)

        self.ser = serial.Serial("/dev/ttyACM1", baudrate=115200, timeout=1000)
        self.relay_off = False
        self.relay_restart = False


        def thrd_fcn():
            try:
                self.get_logger().info("thrd enter")
                last_time = time.time()
                self.ser.write(b'D')
                self.get_logger().info("thrd enter2")
                while True:
                    self.get_logger().info("thrd")
                    if self.relay_restart:
                        self.get_logger().info("relay_restart write")
                        self.ser.write(b'A')
                        self.relay_restart = False
                        self.relay_off = False
                    if not self.relay_off:
                        self.get_logger().info("not relay_off write")
                        self.ser.write(b'D')

                    curr_time = time.time()
                    stop_time = 0.1 - (curr_time - last_time)
                    if stop_time > 0:
                        time.sleep(stop_time)
                    last_time = curr_time
            except serial.SerialTimeoutException:
                self.get_logger().info("serialException")

        self.thrd = threading.Thread(target=thrd_fcn)
        self.thrd.start()
    
    def callback(self, msg):
        if msg.value > 1.0:
            self.get_logger().info("relay_restart")
            self.relay_restart = True
        elif msg.value < -1.0:
            self.get_logger().info("relay_off")
            self.relay_off = True

def main(args=None):
    rclpy.init(args=args)
    relay = MotorRelay()
    rclpy.spin(relay)

if __name__ == '__main__':
    main()
