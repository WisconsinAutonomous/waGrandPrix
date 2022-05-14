import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_vehicle_msgs.msg import MotorPower, ActuatorPower

import time


class Kill_Switch_Publisher(Node):

    global kill_value # This will be used to monitor all processes. 0.0 means good, -2.0 means bad
    kill_value = 0.0

    class BrakingData:
        def __init__(self):
            self.last = time.time() # time last message was received
            self.received = NULL # time received of current message
        
        def subscriber_callback(self, msg):
            second = time.time()
            if self.last - self.received > .2: # if time between messages is greater than .2 seconds, kill
                kill_value = -2.0
            self.last = self.received

    class SteeringData:
        def __init__(self):
            self.last = time.time() # time last message was received
            self.received = NULL # time received of current message
        
        def subscriber_callback(self, msg):
            second = time.time()
            if self.last - self.received > .2: # if time between messages is greater than .2 seconds, kill
                kill_value = -2.0
            self.last = self.received

    class ThrottleData:
        def __init__(self):
            self.last = time.time() # time last message was received
            self.received = NULL # time received of current message
        
        def subscriber_callback(self, msg):
            second = time.time()
            if self.last - self.received > .2: # if time between messages is greater than .2 seconds, kill
                kill_value = -2.0
            self.last = self.received

    def __init__(self):
        super().__init__('kill_switch_publisher')

        self.logger = rclpy.logging.get_logger(self.get_name())

        self.braking = self.BrakingData()
        self.actuator_relay = self.ActuationRelayData()


        # ------------
        # Parse params
        # ------------
        motor_relay_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the motor relay msg will be shipped on.")
        self.declare_parameter("motor_relay_topic", "/control/motor_relay", motor_relay_descriptor)
        self.motor_relay_topic = self.get_parameter("motor_relay_topic").value

        actuator_relay_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the actuator relay msg will be shipped on.")
        self.declare_parameter("actuator_relay_topic", "/control/actuator_relay", actuator_relay_descriptor)
        self.actuator_relay_topic = self.get_parameter("actuator_relay_topic").value

        braking_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the braking msg will be shipped on.")
        self.declare_parameter("braking_topic", "/control/braking", braking_descriptor)
        self.braking_topic = self.get_parameter("braking_topic").value

        steering_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the steering msg will be shipped on.")
        self.declare_parameter("steering_topic", "/control/steering", steering_descriptor)
        self.steering_topic = self.get_parameter("steering_topic").value

        throttle_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the throttle msg will be shipped on.")
        self.declare_parameter("throttle_topic", "/control/throttle", throttle_descriptor)
        self.throttle_topic = self.get_parameter("throttle_topic").value

        # Create publisher handles
        self.publisher_handles = {}
        self.publisher_handles[self.motor_relay_topic] = self.create_publisher(MotorPower, self.motor_relay_topic, 1)
        self.publisher_handles[self.actuator_relay_topic] = self.create_publisher(ActuatorPower, self.actuator_relay_topic, 1)

        # Create Subscriber handles
        self.subscriber_handles = {}
        self.subscriber_handles[self.motor_relay_topic] = self.create_subscription(MotorPower, self.motor_relay_topic, self.motor_relay.subscriber_callback, 1)
        self.subscriber_handles[self.actuator_relay_topic] = self.create_subscription(ActuatorPower, self.actuator_relay_topic, self.actuator_relay.subscriber_callback, 1)

        # Timer to make sure we publish at a controlled rate
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        kill_value = 2.0


    def timer_callback(self):
        msg = MotorPower()
        # if statement here that parses topics for errors
        # error found: self.i = -2.0
        # nothing found: self.i = 0.0
        msg.value = kill_value
        self.publisher_handles[self.motor_relay_topic].publish(msg)
        self.get_logger().info(f"Sent {msg} on topic {self.motor_relay_topic}")

        msg2 = ActuatorPower()
        # if statement here that parses topics for errors
        # error found: self.i = -2.0
        # nothing found: self.i = 0.0
        if kill_value != -2.0:
            msg2.value = kill_value 
        else:
            msg2.value = 0.0 # never publish -2.0 (error) on actuator topic
        self.publisher_handles[self.actuator_relay_topic].publish(msg2)
        self.get_logger().info(f"Sent {msg2} on topic {self.actuator_relay_topic}")

        msg3 = e_Brake() # need to create and import this message type
        # if something goes wrong, publish .6
        if kill_value == -2.0:
            msg3.value = .6 # apply e brakes if error is found
        else:
            msg3.value = 0.0
        self.publisher_handles[self.e_brake_topic].publish(msg3)
        self.get_logger().info(f"Sent {msg3} on topic {self.e_brake_topic}")






def main(args=None):
    rclpy.init(args=args)

    kill_switch_publisher = Kill_Switch_Publisher()

    rclpy.spin(kill_switch_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    kill_switch_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()