import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_vehicle_msgs.msg import MotorPower
from wagrandprix_control_msgs.msg import SteeringCommand, ThrottleCommand, BrakingCommand
from sensor_msgs.msg import PointCloud2, Imu
from geometry_msgs.msg import TwistStamped
from sbg_driver.msg import SbgGpsPos

import time

class Kill_Switch_Publisher(Node):
    
    def publish_kill(self):
        msg = MotorPower()
        msg.value = -2.0
        self.publisher_handles[self.motor_relay_topic].publish(msg)
        self.get_logger().info(f"Sent {msg} on topic {self.motor_relay_topic}")
        msg = BrakingCommand()
        msg.value = 0.6
        self.publisher_handles[self.e_brake_topic].publish(msg)
        self.get_logger().info(f"Sent {msg} on topic {self.e_brake_topic}")

    def braking_subscriber_callback(self, msg):
        self.latest_throttle = time.time() # time received of current message
        if self.latest_throttle - self.last_throttle > 0.015: # if time between messages is greater than 0.015 seconds, kill
            self.publish_kill
        self.last_throttle = self.latest_throttle

    def steering_subscriber_callback(self, msg):
        self.latest_steer = time.time() # time received of current message
        if self.latest_steer - self.last_steer > 0.015: # if time between messages is greater than 0.015 seconds, kill
            self.publish_kill
        self.last_steer = self.latest_steer

    def throttle_subscriber_callback(self, msg):
        self.latest_throttle = time.time() # time received of current message
        if self.latest_throttle - self.last_throttle > 0.015: # if time between messages is greater than 0.015 seconds, kill
            self.publish_kill
        self.last_throttle = self.latest_throttle

    def zed_subscriber_callback(self, msg):
        self.latest_zed = time.time() # time received of current message
        if self.latest_zed - self.last_zed > 0.3: # if time between messages is greater than 0.28 seconds, kill
            self.publish_kill
        self.last_zed = self.latest_zed

    def imu_subscriber_callback(self, msg):
        self.latest_imu = time.time() # time received of current message
        if self.latest_imu - self.last_imu > 0.06: # if time between messages is greater than 0.06 seconds, kill
            self.publish_kill
        self.last_imu = self.latest_imu

    def vel_subscriber_callback(self, msg):
        self.latest_vel = time.time() # time received of current message
        if self.latest_vel - self.last_vel > 0.015: # if time between messages is greater than 0.28 seconds, kill
            self.publish_kill
        self.last_vel = self.latest_vel

    def gps_subscriber_callback(self, msg):
        self.latest_gps = time.time() # time received of current message
        if self.latest_gps - self.last_gps > 0.3: # if time between messages is greater than 0.28 seconds, kill
            self.publish_kill
        self.last_gps = self.latest_gps

    def __init__(self):
        super().__init__('kill_switch_publisher')

        self.logger = rclpy.logging.get_logger(self.get_name())

        # ------------
        # Parse params
        # ------------
        motor_relay_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the motor relay msg will be shipped on.")
        self.declare_parameter("motor_relay_topic", "/control/motor_relay", motor_relay_descriptor)
        self.motor_relay_topic = self.get_parameter("motor_relay_topic").value

        braking_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the braking msg will be shipped on.")
        self.declare_parameter("braking_topic", "/control/braking", braking_descriptor)
        self.braking_topic = self.get_parameter("braking_topic").value

        steering_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the steering msg will be shipped on.")
        self.declare_parameter("steering_topic", "/control/steering", steering_descriptor)
        self.steering_topic = self.get_parameter("steering_topic").value

        throttle_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the throttle msg will be shipped on.")
        self.declare_parameter("throttle_topic", "/control/throttle", throttle_descriptor)
        self.throttle_topic = self.get_parameter("throttle_topic").value

        zed_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the zed camera msg will be shipped on.")
        self.declare_parameter("zed_topic", "/zed/zed_node/point_cloud/cloud_registered", zed_descriptor)
        self.zed_topic = self.get_parameter("zed_topic").value

        imu_data_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the imu data msg will be shipped on.")
        self.declare_parameter("imu_data_topic", "/imu/data", imu_data_descriptor)
        self.imu_data_topic = self.get_parameter("imu_data_topic").value

        velocity_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the imu velocity msg will be shipped on.")
        self.declare_parameter("velocity_topic", "/imu/velocity", velocity_descriptor)
        self.velocity_topic = self.get_parameter("velocity_topic").value

        gps_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the sbg gps pos msg will be shipped on.")
        self.declare_parameter("gps_topic", "/sbg/gps_pos", gps_descriptor)
        self.gps_topic = self.get_parameter("gps_topic").value

        # Create publisher handles
        self.publisher_handles = {}
        self.publisher_handles[self.motor_relay_topic] = self.create_publisher(MotorPower, self.motor_relay_topic, 1)
        self.publisher_handles[self.e_brake_topic] = self.create_publisher(BrakingCommand, self.e_brake_topic, 1)

        msg = MotorPower() # initialize motor
        msg.value = 2.0
        self.publisher_handles[self.motor_relay_topic].publish(msg)
        self.get_logger().info(f"Sent {msg} on topic {self.motor_relay_topic}")

        self.last_brake = time.time() # initialize first message for braking
        self.last_steer = time.time() # initialize first message for steering
        self.last_throttle = time.time() # initialize first message for throttle
        self.last_zed = time.time() # initialize first message for zed
        self.last_imu = time.time() # initialize first message for imu data
        self.last_vel = time.time() # initialize first message for velocity
        self.last_gps = time.time() # initialize first message for gps

        # Create Subscriber handles
        self.subscriber_handles = {}
        self.subscriber_handles[self.braking_topic] = self.create_subscription(BrakingCommand, self.braking_topic, self.braking_subscriber_callback, 1)
        self.subscriber_handles[self.steering_topic] = self.create_subscription(SteeringCommand, self.steering_topic, self.steering_subscriber_callback, 1)
        self.subscriber_handles[self.throttle_topic] = self.create_subscription(ThrottleCommand, self.throttle_topic, self.throttle_subscriber_callback, 1)
        self.subscriber_handles[self.zed_topic] = self.create_subscription(PointCloud2, self.zed_topic, self.zed_subscriber_callback, 1)
        self.subscriber_handles[self.imu_data_topic] = self.create_subscription(Imu, self.imu_data_topic, self.imu_subscriber_callback, 1)
        self.subscriber_handles[self.velocity_topic] = self.create_subscription(TwistStamped, self.velocity_topic, self.vel_subscriber_callback, 1)
        self.subscriber_handles[self.gps_topic] = self.create_subscription(SbgGpsPos, self.gps_topic, self.gps_subscriber_callback, 1)

        # Timer to make sure we publish at a controlled rate
        timer_period = 0.01  # 100 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if min(self.last_brake, self.last_gps, self.last_imu, 
        self.last_steer, self.last_imu, self.last_vel, self.last_zed) - time.time() > .3: # If any messages are beyond maximum threshold
            self.publish_kill
        msg = MotorPower()
        msg.value = 0.0
        self.publisher_handles[self.motor_relay_topic].publish(msg)
        self.get_logger().info(f"Sent {msg} on topic {self.motor_relay_topic}")

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