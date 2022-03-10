# General ROS imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_vehicle_msgs import VehicleState
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu


class VehicleStateEstimator(Node):

    def __init__(self):
        super().__init__('vehicle_state_estimator')

        self.logger = rclpy.logging.get_logger(self.get_name())

        # ------------
        # Parse params
        # ------------
        imu_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that imu data will be shipped on.")
        self.declare_parameter("imu_topic", "/sensor/imu/data", imu_descriptor)
        self.imu_topic = self.get_parameter("imu_topic").value

        gps_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that gps data will be shipped on.")
        self.declare_parameter("gps_topic", "/sensor/gps/data", gps_descriptor)
        self.gps_topic = self.get_parameter("gps_topic").value

        wheel_encoder_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that wheel_encoder data will be shipped on.")
        self.declare_parameter("wheel_encoder_topic", "/sensor/wheel_encoder/data", wheel_encoder_descriptor)
        self.wheel_encoder_topic = self.get_parameter("wheel_encoder_topic").value

        steering_feedback_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that steering feedback data will be shipped on.")
        self.declare_parameter("steering_feedback_topic", "/sensor/steering_feedback/data", steering_feedback_descriptor)
        self.steering_feedback_topic = self.get_parameter("steering_feedback_topic").value

        fake_with_sim_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description="Whether to run this nodes algorithms or fake it with sim data.")
        self.declare_parameter("fake_with_sim", False, fake_with_sim_descriptor)
        self.fake_with_sim = self.get_parameter("fake_with_sim").value

        sim_vehicle_state_topic_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic sim data is published that provides vehicle state information")
        self.declare_parameter("sim_vehicle_state_topic", "/sim/vehicle/state", sim_vehicle_state_topic_descriptor)
        self.sim_vehicle_state_topic = self.get_parameter("sim_vehicle_state_topic").value

        self.logger.info(f"fake_with_sim: {self.fake_with_sim}")

        # ------------
        # ROS Entities
        # ------------

        # Create publisher handles
        self.publisher_handles = {}
        self.publisher_handles["vehicle/state"] = self.create_publisher(VehicleState, "vehicle/state", 1)

        # Create subscriber handles
        self.subscriber_handles = {}

        # If desired, we may "fake" the vehicle state estimator node
        # This can be done using sim, where we grab all of the vehicle state information directly from the simulation
        if self.fake_with_sim:
            from wa_simulator_ros_msgs.msg import WAVehicle

            self.logger.info(f"sim_vehicle_state_topic: {self.sim_vehicle_state_topic}")

            self.subscriber_handles[self.sim_vehicle_state_topic] = self.create_subscription(WAVehicle, self.sim_vehicle_state_topic, self.sim_vehicle_state_callback, 1)
        else:
            self.logger.info(f"imu_topic: {self.imu_topic}")
            self.logger.info(f"gps_topic: {self.gps_topic}")
            self.logger.info(f"wheel_encoder_topic: {self.wheel_encoder_topic}")
            self.logger.info(f"steering_feedback_topic: {self.steering_feedback_topic}")

            self.subscriber_handles[self.imu_topic] = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 1)
            self.subscriber_handles[self.gps_topic] = self.create_subscription(NavSatFix, self.gps_topic, self.gps_callback, 1)
            self.subscriber_handles[self.wheel_encoder_topic] = self.create_subscription(Float64, self.wheel_encoder_topic, self.wheel_encoder_callback, 1)
            self.subscriber_handles[self.steering_feedback_topic] = self.create_subscription(Float64, self.steering_feedback_topic, self.steering_feedback_callback, 1)

        # ------------------------
        # Initialize Class Members
        # ------------------------

        self.vehicle_state = VehicleState()

    def imu_callback(self, msg):
        """
        Callback for the imu data topic.
        """
        self.logger.debug(f"Received {msg} on topic {self.imu_topic}")

    def gps_callback(self, msg):
        """
        Callback for the gps data topic.
        """
        self.logger.debug(f"Received {msg} on topic {self.gps_topic}")

    def wheel_encoder_callback(self, msg):
        """
        Callback for the wheel encoder data topic.
        """
        self.logger.debug(f"Received {msg} on topic {self.wheel_encoder_topic}")

    def steering_feedback_callback(self, msg):
        """
        Callback for the steering feedback data topic.
        """
        self.logger.debug(f"Received {msg} on topic {self.steering_feedback_topic}")

    def sim_vehicle_state_callback(self, msg):
        """
        Callback for the sim vehicle state data
        """
        self.logger.debug(f"Received {msg} on topic {self.sim_vehicle_state_topic}")

        self.vehicle_state.accel = msg.accel
        self.vehicle_state.twist = msg.twist
        self.vehicle_state.pose = msg.pose
        self.publisher_handles["vehicle/state"].publish(self.vehicle_state)


def main(args=None):
    rclpy.init(args=args)

    vehicle_state_estimator = VehicleStateEstimator()

    rclpy.spin(vehicle_state_estimator)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
