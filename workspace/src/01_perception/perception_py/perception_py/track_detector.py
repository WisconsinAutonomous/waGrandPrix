# General ROS imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseArray
from wagrandprix_map_msgs.msg import DetectedTrack, Point

class TrackDetector(Node):

    def __init__(self):
        super().__init__('track_detector')

        self.logger = rclpy.logging.get_logger(self.get_name())

        # ------------
        # Parse params
        # ------------
        # camera_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that image/camera data will be shipped on.")
        # self.declare_parameter("camera_topic", "/sensor/camera/front/image", camera_descriptor)
        # self.camera_topic = self.get_parameter("camera_topic").value

        pointcloud_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that pointcloud data will be shipped on.")
        self.declare_parameter("pointcloud_topic", "zed/zed_node/point_cloud/cloud_registered", pointcloud_descriptor)
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").value

        fake_with_sim_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description="Whether to run the perception algorithms or fake it with sim data.")
        self.declare_parameter("fake_with_sim", False, fake_with_sim_descriptor)
        self.fake_with_sim = self.get_parameter("fake_with_sim").value

        sim_track_topic_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the simulation publishes to for use as a replacement for the actual track detector.")
        self.declare_parameter("sim_track_topic", "/sim/track/visible", sim_track_topic_descriptor)
        self.sim_track_topic = self.get_parameter("sim_track_topic").value

        self.logger.info(f"fake_with_sim: {self.fake_with_sim}")

        # ------------
        # ROS Entities
        # ------------

        # Create publisher handles
        self.publisher_handles = {}
        self.publisher_handles["detected/track"] = self.create_publisher(DetectedTrack, "detected/track", 1)

        # Create subscriber handles
        self.subscriber_handles = {}

        # If desired, we may "fake" the perception node
        # This can be done in simulation, and essentially grabs the simulation state and determines the track absent of any perception algorithm
        if self.fake_with_sim:
            from wa_simulator_ros_msgs.msg import WATrack

            self.logger.info(f"sim_track_topic: {self.sim_track_topic}")

            self.subscriber_handles[self.sim_track_topic] = self.create_subscription(WATrack, self.sim_track_topic, self.sim_track_callback, 1)
        else:
            # self.logger.info(f"camera_topic: {self.camera_topic}")
            self.logger.info(f"pointcloud_topic: {self.pointcloud_topic}")

            # self.subscriber_handles[self.camera_topic] = self.create_subscription(Image, self.camera_topic, self.camera_callback, 1)
            self.subscriber_handles[self.pointcloud_topic] = self.create_subscription(PointCloud2, self.pointcloud_topic, self.pointcloud_callback, 1)

        # ------------------------
        # Initialize Class Members
        # ------------------------

    # def camera_callback(self, msg):
    #     """
    #     Callback for the camera image topic.
    #     """
    #     self.logger.debug(f"Received {msg} on topic {self.camera_topic}")

    def pointcloud_callback(self, msg):
        """
        Callback for the zed pointcloud topic.
        """
        self.logger.debug(f"Received {msg} on topic {self.pointcloud_topic}")

    def sim_track_callback(self, msg):
        """
        Callback for the sim track data
        """
        self.logger.debug(f"Received {msg} on topic {self.sim_track_topic}")

        detected_track = DetectedTrack(left_points=msg.left_points, right_points=msg.right_points)
        self.publisher_handles["detected/track"].publish(detected_track)


def main(args=None):
    rclpy.init(args=args)

    track_detector = TrackDetector()

    rclpy.spin(track_detector)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
