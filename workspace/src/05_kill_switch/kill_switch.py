import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_vehicle_msgs.msg import MotorPower, ActuatorPower
from wagrandprix_control_msgs.msg import SteeringCommand, ThrottleCommand, BrakingCommand

import time

class Kill_Switch_Publisher(Node):

    class BrakingData(Kill_Switch_Publisher):
        def __init__(self):
            self.last_brake = time.time() # time last message was received
        
        def subscriber_callback(self, msg):
            self.latest_brake = time.time() # time received of current message
            if self.last_brake - self.latest_brake > .2: # if time between messages is greater than .2 seconds, kill
                super.kill_value = -2.0
            else:
                super.kill_value = 0.0
            self.last_brake = self.latest_brake

    class SteeringData(Kill_Switch_Publisher):
        def __init__(self):
            self.last_steer = time.time() # time last message was received
        
        def subscriber_callback(self, msg):
            self.latest_steer = time.time() # time received of current message
            if self.last_steer - self.latest_steer > .2: # if time between messages is greater than .2 seconds, kill
                super.kill_value = -2.0
            else:
                super.kill_value = 0.0
            self.last_steer = self.latest_steer

    class ThrottleData(Kill_Switch_Publisher):
        def __init__(self):
            self.last_throttle = time.time() # time last message was received
        
        def subscriber_callback(self, msg):
            self.latest_throttle = time.time() # time received of current message
            if self.last_throttle - self.latest_throttle > .2: # if time between messages is greater than .2 seconds, kill
                super.kill_value = -2.0
            else:
                super.kill_value = 0.0
            self.last_throttle = self.latest_throttle

    def __init__(self):
        super().__init__('kill_switch_publisher')

        self.logger = rclpy.logging.get_logger(self.get_name())

        self.braking = self.BrakingData()
        self.steering = self.SteeringData()
        self.throttle = self.ThrottleData()

        self.kill_value = 2.0 # initialized to 2.0 for restart

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
        self.publisher_handles[self.e_brake_topic] = self.create_publisher(BrakingCommand, self.e_brake_topic, 1)

        msg = MotorPower() # initialize motor
        msg.value = 2.0
        self.publisher_handles[self.motor_relay_topic].publish(msg)
        self.get_logger().info(f"Sent {msg} on topic {self.motor_relay_topic}")

        # msg2 = ActuatorPower() # initialize actuator
        # msg2.value = 2.0
        # self.publisher_handles[self.motor_relay_topic].publish(msg2)
        # self.get_logger().info(f"Sent {msg2} on topic {self.actuator_relay_topic}")

        # Create Subscriber handles
        self.subscriber_handles = {}
        self.subscriber_handles[self.braking_topic] = self.create_subscription(BrakingCommand, self.braking_topic, self.braking.subscriber_callback, 1)
        self.subscriber_handles[self.steering_topic] = self.create_subscription(SteeringCommand, self.steering_topic, self.steering.subscriber_callback, 1)
        self.subscriber_handles[self.throttle_topic] = self.create_subscription(ThrottleCommand, self.throttle_topic, self.throttle.subscriber_callback, 1)

        # Timer to make sure we publish at a controlled rate
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg = MotorPower()
        # if statement here that parses topics for errors
        # error found: self.i = -2.0
        # nothing found: self.i = 0.0
        msg.value = super.kill_value
        self.publisher_handles[self.motor_relay_topic].publish(msg)
        self.get_logger().info(f"Sent {msg} on topic {self.motor_relay_topic}")

        # msg2 = ActuatorPower()
        # if statement here that parses topics for errors
        # error found: self.i = -2.0
        # nothing found: self.i = 0.0
        # msg2.value = 0.0 # never publish -2.0 (error) on actuator topic
        # self.publisher_handles[self.actuator_relay_topic].publish(msg2)
        # self.get_logger().info(f"Sent {msg2} on topic {self.actuator_relay_topic}")

        msg3 = BrakingCommand()
        # if something goes wrong, publish .6
        if super.kill_value == -2.0:
            msg3.value = .6 # apply e_brakes if error is found
        else:
            return # do not publish anything to brakes if nothing is wrong
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