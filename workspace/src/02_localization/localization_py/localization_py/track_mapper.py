# General ROS imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_vehicle_msgs.msg import VehicleState
from wagrandprix_map_msgs.msg import Track, DetectedTrack, Waypoint, GPSCoordinate
from wa_simulator_ros_msgs.msg import WATrack


class TrackMapper(Node):

    def __init__(self):
        super().__init__('track_mapper')

        self.logger = rclpy.logging.get_logger(self.get_name())

        # ------------
        # Parse params
        # ------------
        detected_track_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic the perceived track detections will be shipped on.")
        self.declare_parameter("detected_track_topic", "/perception/track/detected", detected_track_descriptor)
        self.detected_track_topic = self.get_parameter("detected_track_topic").value

        vehicle_state_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic the vehicle state estimation will be shipped on.")
        self.declare_parameter("vehicle_state_topic", "vehicle/state", vehicle_state_descriptor)
        self.vehicle_state_topic = self.get_parameter("vehicle_state_topic").value

        fake_with_sim_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description="Whether to run this nodes algorithms or fake it with sim data.")
        self.declare_parameter("fake_with_sim", False, fake_with_sim_descriptor)
        self.fake_with_sim = self.get_parameter("fake_with_sim").value

        sim_track_topic_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic sim data is published that provides mapped track information")
        self.declare_parameter("sim_track_topic", "/sim/track/mapped", sim_track_topic_descriptor)
        self.sim_track_topic = self.get_parameter("sim_track_topic").value

        self.logger.info(f"fake_with_sim: {self.fake_with_sim}")

        # ------------
        # ROS Entities
        # ------------

        # Create publisher handles
        self.publisher_handles = {}
        self.publisher_handles["track/mapped"] = self.create_publisher(Track, "track/mapped", 1)

        # Create subscriber handles
        self.subscriber_handles = {}

        # If desired, we may "fake" the track mapper node
        # This can be done using sim, where we grab all of the vehicle state information directly from the simulation
        if self.fake_with_sim:
            from wa_simulator_ros_msgs.msg import WAVehicle

            self.logger.info(f"sim_track_topic: {self.sim_track_topic}")

            self.subscriber_handles[self.sim_track_topic] = self.create_subscription(WATrack, self.sim_track_topic, self.sim_track_callback, 1)
        else:
            self.logger.info(f"detected_track_topic: {self.detected_track_topic}")
            self.logger.info(f"vehicle_state_topic: {self.vehicle_state_topic}")

            self.subscriber_handles[self.detected_track_topic] = self.create_subscription(DetectedTrack, self.detected_track_topic, self.detected_track_callback, 1)
            self.subscriber_handles[self.vehicle_state_topic] = self.create_subscription(VehicleState, self.vehicle_state_topic, self.vehicle_state_callback, 1)

        # ------------------------
        # Initialize Class Members
        # ------------------------

    def detected_track_callback(self, msg):
        """
        Callback for the detected track data topic.
        """
        self.logger.debug(f"Received {msg} on topic {self.detected_track_topic}")

    def vehicle_state_callback(self, msg):
        """
        Callback for the vehicle state data topic.
        """
        self.logger.debug(f"Received {msg} on topic {self.vehicle_state_topic}")

    def sim_track_callback(self, msg):
        """
        Callback for the sim mapped track data
        """
        self.logger.debug(f"Received {msg} on topic {self.sim_track_topic}")

        track = Track()

        for i, (coord, point, width) in enumerate(zip(msg.mapped_coords, msg.mapped_points, msg.mapped_widths)):
            wp = Waypoint()
            wp.coordinate = GPSCoordinate(latitude=coord.x, longitude=coord.y, altitude=coord.z)
            wp.point = point
            wp.left_width = width.x
            wp.right_width = width.y
            wp.idx = i
            track.waypoints.append(wp)

        self.publisher_handles["track/mapped"].publish(track)


def main(args=None):
    rclpy.init(args=args)

    track_mapper = TrackMapper()

    rclpy.spin(track_mapper)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
