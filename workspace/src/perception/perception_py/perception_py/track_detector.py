# General ROS imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseArray
from wagrandprix_interfaces.msg import DetectedTrack, Point

# Other imports
import functools


class TrackDetector(Node):

    def __init__(self):
        super().__init__('track_detector')

        self.logger = rclpy.logging.get_logger(self.get_name())

        # ------------
        # Parse params
        # ------------
        camera_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING, description="The topic that image/camera data will be shipped on.")
        self.declare_parameter(
            "camera_topic", "/sensor/camera/front/image", camera_descriptor)
        self.camera_topic = self.get_parameter("camera_topic").value

        lidar_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING, description="The topic that pointcloud/lidar data will be shipped on.")
        self.declare_parameter(
            "lidar_topic", "/sensor/lidar/pointcloud", lidar_descriptor)
        self.lidar_topic = self.get_parameter("lidar_topic").value

        fake_with_sim_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL, description="Whether to run the perception algorithms or fake it with sim data.")
        self.declare_parameter("fake_with_sim", False,
                               fake_with_sim_descriptor)
        self.fake_with_sim = self.get_parameter("fake_with_sim").value

        sim_track_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING, description="Whether to run the perception algorithms or fake it with sim data.")
        self.declare_parameter(
            "sim_track_topic", "/sim/track", sim_track_topic_descriptor)
        self.sim_track_topic = self.get_parameter("sim_track_topic").value
        self.sim_left_track_topic = self.sim_track_topic + "/left"
        self.sim_right_track_topic = self.sim_track_topic + "/right"

        self.logger.info(f"fake_with_sim: {self.fake_with_sim}")

        # ------------
        # ROS Entities
        # ------------

        # Create publisher handles
        self.publisher_handles = {}
        self.publisher_handles["detected/track"] = self.create_publisher(
            DetectedTrack, "detected/track", 1)

        # Create subscriber handles
        self.subscriber_handles = {}

        # If desired, we may "fake" the perception node
        # This can be done in simulation, and essentially grabs the simulation state and determines the track absent of any perception algorithm
        if self.fake_with_sim:
            self.logger.info(
                f"sim_left_track_topic: {self.sim_left_track_topic}")
            self.logger.info(
                f"sim_right_track_topic: {self.sim_right_track_topic}")

            self.subscriber_handles[self.sim_left_track_topic] = self.create_subscription(
                PoseArray, self.sim_left_track_topic, functools.partial(self.sim_track_callback, "left"), 1)
            self.subscriber_handles[self.sim_right_track_topic] = self.create_subscription(
                PoseArray, self.sim_right_track_topic, functools.partial(self.sim_track_callback, "right"), 1)
        else:
            self.logger.info(f"camera_topic: {self.camera_topic}")
            self.logger.info(f"lidar_topic: {self.lidar_topic}")

            self.subscriber_handles[self.camera_topic] = self.create_subscription(
                Image, self.camera_topic, self.camera_callback, 1)
            self.subscriber_handles[self.lidar_topic] = self.create_subscription(
                PointCloud2, self.lidar_topic, self.lidar_callback, 1)

        # ------------------------
        # Initialize Class Members
        # ------------------------

        self.detected_track = DetectedTrack()
        self.left = []
        self.right = []
        self.left_detected = False
        self.right_detected = False

    def camera_callback(self, msg):
        """
        Callback for the camera image topic.
        """
        self.get_logger().info(
            f"Received {msg} on topic /sensor/camera/front/image")

    def lidar_callback(self, msg):
        """
        Callback for the lidar pointcloud topic.
        """
        self.get_logger().info(
            f"Received {msg} on topic /sensor/lidar/pointcloud")

    def sim_track_callback(self, side, msg):
        """
        Callback for the sim track data
        """
        if side == 'left':
            self.left = [Point(x=p.position.x, y=p.position.y,
                               z=p.position.z) for p in msg.poses]
            self.left_detected = True

        if side == 'right':
            self.right = [Point(x=p.position.x, y=p.position.y,
                               z=p.position.z) for p in msg.poses]
            self.right_detected = True

        if self.left_detected and self.right_detected:
            detected_track = DetectedTrack(left_points=self.left, right_points=self.right)
            self.publisher_handles["detected/track"].publish(detected_track)


def main(args=None):
    rclpy.init(args=args)

    track_detector = TrackDetector()

    rclpy.spin(track_detector)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
