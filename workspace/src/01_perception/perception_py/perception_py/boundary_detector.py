# General ROS imports
from numpy import append
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseArray
from wagrandprix_map_msgs.msg import DetectedTrack, Point

class BoundaryDetector(Node):

    def __init__(self):
        super().__init__('boundary_detector')

        self.logger = rclpy.logging.get_logger(self.get_name())

        # ------------
        # Parse params
        # ------------
        pointcloud_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that pointcloud data will be shipped on.")
        self.declare_parameter("pointcloud_topic", "zed/zed_node/point_cloud/cloud_registered", pointcloud_descriptor)
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").value
        
        # ------------
        # ROS Entities
        # ------------

        # Create publisher handles
        self.publisher_handles = {}
        self.publisher_handles["detected/track"] = self.create_publisher(DetectedTrack, "detected/track", 1)

        self.left_track = []
        self.right_track = []

        # Create subscriber handles
        self.subscriber_handles = {}
        self.logger.info(f"pointcloud_topic: {self.pointcloud_topic}")
        self.subscriber_handles[self.pointcloud_topic] = self.create_subscription(PointCloud2, self.pointcloud_topic, self.pointcloud_callback, 1)


    def pointcloud_callback(self, msg):
        """
        Callback for the zed pointcloud topic.
        """
        self.logger.debug(f"Received {msg} on topic {self.pointcloud_topic}")

        # NOTE: I'm assuming msg.data is an array of Point msgs (probably incorrect)
        self.get_left_right_tracks(msg.data)
        self.publish_tracks()


    def get_left_right_tracks(self, points):
        """
        Determine left and right tracks from array of Point msgs
        Assumption 1: Y axis is front distance from camera, X axis is side distance from camera
        Assumption 2: Any pair of left and right cones are at about the same Y distance
        """

        Y_THRESHOLD = 1 # NOTE: value needs to be fine tuned
        # variables used to avoid IOB exception
        is_first_point = False
        is_last_point = False

        points.sort(key=lambda point: point.y, reverse=False)

        for i in range(len(points)):
            is_first_point = False
            is_last_point = False

            if (i == 0): is_first_point = True
            elif (i == (len(points)-1)): is_last_point = True

            else:
                # compare current point with prior point
                if (not is_last_point and (abs(points[i].y - points[i+1].y) < Y_THRESHOLD)):
                    if (points[i+1].x < points[i].x): self.right_track.append(points[i])
                    else: self.left_track.append(points[i])
                # compare current point with following point
                elif (not is_first_point and (abs(points[i].y - points[i-1].y) < Y_THRESHOLD)):
                    if (points[i-1].x < points[i].x): self.right_track.append(points[i])
                    else: self.left_track.append(points[i])
                # default to assigning point to left/right of y-axis
                else:
                    if (points[i].x < 0): self.left_track.append(points[i])
                    else: self.right_track.append(points[i])


    def publish_tracks(self):
        """
        Publish left and right tracks
        """
        detected_track = DetectedTrack(left_points=self.left_track, right_points=self.right_track)
        self.publisher_handles["detected/track"].publish(detected_track)
        


def main(args=None):
    rclpy.init(args=args)

    boundary_detector = BoundaryDetector()

    rclpy.spin(boundary_detector)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
