# General ROS imports
import rclpy
from rclpy.node import Node
import serial
import time

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_vehicle_msgs.msg import MotorPower, ActuatorPower

class PowerRelay(Node):

    class MotorRelayData:
        RESTART = b'R'
        ERROR = b'E'
        GUARD = b'G'

        def __init__(self):
            self.relay_off = False
            self.relay_restart = False
            self.msg_received = False
        
        def subscriber_callback(self, msg):
            if msg is not None:
                self.msg_received = True
                if msg.value > 1.0:
                    self.relay_restart = True
                elif msg.value < -1.0:
                    self.relay_off = True
        
        def timer_callback(self, ser):
            if self.relay_restart:
                ser.write(self.RESTART)
                self.relay_restart = False
                self.relay_off = False
            elif self.relay_off:
                ser.write(self.ERROR)
            elif self.msg_received:
                self.msg_received = False
                ser.write(self.GUARD)
    
    class ActuationRelayData:
        RESTART = b'r'
        ERROR = b'e'
        GUARD = b'g'

        def __init__(self, logger):
            self.relay_off = False
            self.relay_restart = False
            self.braking_on = False
            self.steering_on = False
            self.logger = logger
        
        def subscriber_callback_brake(self, msg):
            self.logger.info("callback brake")
            if msg is not None:
                self.logger.info(f"{msg} {self.braking_on} {self.steering_on}")
                if msg.value > 0.0:
                    self.braking_on = True
                    time.sleep(1)
            if self.steering_on and self.braking_on:
                self.logger.info("start")
                self.relay_restart = True

        def subscriber_callback_steering(self, msg):
            self.logger.info("callback steering")
            if msg is not None:
                self.logger.info(f"{msg} {self.braking_on} {self.steering_on}")
                if msg.value > 0.0:
                    self.steering_on = True
                    time.sleep(1)
            if self.steering_on and self.braking_on:
                self.logger.info("start")
                self.relay_restart = True
        
        def timer_callback(self, ser):
            if self.relay_restart:
                self.logger.info("restart!!!")
                ser.write(self.RESTART)                                                                 
                self.relay_restart = False
                self.relay_off = False
            elif self.relay_off:
                ser.write(self.ERROR)
            else:
                ser.write(self.GUARD)


    def __init__(self, port):
        super().__init__('motor_relay')

        self.logger = rclpy.logging.get_logger(self.get_name())

        self.motor_relay = self.MotorRelayData()
        self.actuator_relay = self.ActuationRelayData(self.logger)

        motor_relay_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the motor relay msg will be shipped on.")
        self.declare_parameter("motor_relay_topic", "/controls/motor_relay", motor_relay_descriptor)
        self.motor_relay_topic = self.get_parameter("motor_relay_topic").value

        brake_actuator_relay_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the brake actuator relay msg will be shipped on.")
        self.declare_parameter("brake_actuator_relay_topic", "/controls/brake_actuator_relay", brake_actuator_relay_descriptor)
        self.brake_actuator_relay_topic = self.get_parameter("brake_actuator_relay_topic").value

        steering_actuator_relay_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the steering actuator relay msg will be shipped on.")
        self.declare_parameter("steering_actuator_relay_topic", "/controls/steering_actuator_relay", steering_actuator_relay_descriptor)
        self.steering_actuator_relay_topic = self.get_parameter("steering_actuator_relay_topic").value

        self.subscriber_handles = {}
        self.subscriber_handles[self.motor_relay_topic] = self.create_subscription(MotorPower, self.motor_relay_topic, self.motor_relay.subscriber_callback, 1)
        self.subscriber_handles[self.brake_actuator_relay_topic] = self.create_subscription(ActuatorPower, self.brake_actuator_relay_topic, self.actuator_relay.subscriber_callback_brake, 1)
        self.subscriber_handles[self.steering_actuator_relay_topic] = self.create_subscription(ActuatorPower, self.steering_actuator_relay_topic, self.actuator_relay.subscriber_callback_steering, 1)

        self.ser = serial.Serial(port, baudrate=115200, timeout=1000)
        self.relay_off = False
        self.relay_restart = False
        self.msg_received = False

        self.ser.write(self.motor_relay.RESTART)     
        self.ser.write(self.actuator_relay.RESTART)
        self.timer = self.create_timer(0.5, self.timer_callback)
    
    def timer_callback(self):
        self.motor_relay.timer_callback(self.ser)
        self.actuator_relay.timer_callback(self.ser)


        

def main(args=None):
    rclpy.init(args=args)
    relay = PowerRelay("/dev/ttyACM1")
    rclpy.spin(relay)

if __name__ == '__main__':
    main()