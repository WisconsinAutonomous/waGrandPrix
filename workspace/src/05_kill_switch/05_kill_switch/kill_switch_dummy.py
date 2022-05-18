import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_vehicle_msgs.msg import MotorPower
from wagrandprix_control_msgs.msg import SteeringCommand, ThrottleCommand, BrakingCommand
from sensor_msgs.msg import PointCloud2, Imu
from geometry_msgs.msg import TwistStamped
from sng_driver.msg import SbgGpsPos

import time

class Kill_Switch_Publisher_Dummy(Node):
    def __init__(self):
            super().__init__('kill_switch_publisher_dummy')

            self.logger = rclpy.logging.get_logger(self.get_name())

            # ------------
            # Parse params
            # ------------
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

            self.publisher_handles[self.braking_topic] = self.create_publisher(BrakingCommand, self.braking_topic, 1)

            msg = BrakingCommand()
            msg.value = 0.0
            self.publisher_handles[self.braking_topic].publish(msg)
            self.get_logger().info(f"Sent {msg} on topic {self.braking_topic}")

            time.sleep(1)

            msg = BrakingCommand()
            msg.value = 0.0
            self.publisher_handles[self.braking_topic].publish(msg)
            self.get_logger().info(f"Sent {msg} on topic {self.braking_topic}")

    def main(args=None):
        rclpy.init(args=args)

        kill_switch_publisher_dummy = Kill_Switch_Publisher_Dummy()

        rclpy.spin(kill_switch_publisher_dummy)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        kill_switch_publisher_dummy.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()