# General ROS imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseArray
from wagrandprix_map_msgs.msg import DetectedTrack, Point

import numpy as np
import sensor_msgs.point_cloud2 as pc2

DARKNET_BATCH_SIZE = 1
DARKNET_THRESH = 0.6
DARKNET_CONFIG_PATH = '<todo>/cfg/yolov4-tiny-evgrandprix22.cfg'
DARKNET_WEIGHTS_PATH = '<todo>/backup/evgrandprix22/yolov4-tiny-evgrandprix22_last.weights'
DARKNET_DATA_PATH = '<todo>/data/evgrandprix22/evgrandprix22.data'

network = None
network_width, network_height = None, None
network_img_gpu = None
class_names = None
class_colors = None


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
      
      
    def deserialize_zed_pc2_to_numpy(self, pc_msg):
        image_list = []
        depth_list = []
        
        for p in pc2.read_points(msg):
            image_list.append((p[0], p[1], p[2])) # read r,g,b from ZED depth cloud point
            depth_list.append((p[3], p[4], p[5])) # read x,y,z from ZED depth cloud point
            
        image_np = np.array(image_list)
        depth_np = np.array(depth_list)
        
        return (image_np, depth_np)
        
        
    def preprocess_image(self, image):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(image_rgb, (network_width, network_height), interpolation=cv2.INTER_LINEAR) # resize ZED image to size network expects (416x416)
        return image_resized
        

    def pointcloud_callback(self, msg):
        """
        Callback for the zed pointcloud topic.
        """
        self.logger.debug(f"Received {msg} on topic {self.pointcloud_topic}")
        
        raw_image, depth_cloud = deserialize_zed_pc2_to_numpy(msg)
        
        image = preprocess_image(raw_image)
        
        darknet.copy_image_from_bytes(network_img_gpu, image.tobytes()) # copy image data to GPU
        
        detections = darknet.detect_image(network, class_names, network_img_gpu, thresh=DARKNET_THRESH)

        

        detected_track = DetectedTrack(left_points=msg.left_points, right_points=msg.right_points)
        self.publisher_handles["detected/track"].publish(detected_track)

    def sim_track_callback(self, msg):
        """
        Callback for the sim track data
        """
        self.logger.debug(f"Received {msg} on topic {self.sim_track_topic}")

        detected_track = DetectedTrack(left_points=msg.left_points, right_points=msg.right_points)
        self.publisher_handles["detected/track"].publish(detected_track)
        
        

def check_darknet_args(thresh, config_path, weights_path, data_path):
    assert 0 < thresh < 1, "Threshold should be a float between zero and one (non-inclusive)"
    if not os.path.exists(config_path):
        raise(ValueError("Invalid config path {}".format(os.path.abspath(config_path))))
    if not os.path.exists(weights_path):
        raise(ValueError("Invalid weight path {}".format(os.path.abspath(weights_path))))
    if not os.path.exists(data_path):
        raise(ValueError("Invalid data file path {}".format(os.path.abspath(data_path))))

def init_model():
    check_darknet_args(DARKNET_THRESH, DARKNET_CONFIG_PATH, DARKNET_WEIGHTS_PATH, DARKNET_DATA_PATH)
    
    network, class_names, class_colors = darknet.load_network(
            DARKNET_CONFIG_PATH,
            DARKNET_DATA_PATH,
            DARKNET_WEIGHTS_PATH,
            batch_size=DARKNET_BATCH_SIZE
    )
    
    network_width = darknet.network_width(network)
    network_height = darknet.network_height(network)
    
    network_img_gpu = darknet.make_image(network_width, network_height, 3)
    
    self.logger.debug('Initialized YOLOv4 (darknet) model')


def main(args=None):
    rclpy.init(args=args)
    
    init_model()
    track_detector = TrackDetector()

    rclpy.spin(track_detector)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
