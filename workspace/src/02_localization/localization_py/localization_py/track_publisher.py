# General ROS imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wa_simulator_ros_msgs.msg import WATrack
from geometry_msgs.msg import Point


class TrackPublisher(Node):

    def __init__(self):
        super().__init__('track_mapper')

        self.logger = rclpy.logging.get_logger(self.get_name())

        # ------------
        # ROS Entities
        # ------------

        # Create publisher handles
        self.publisher_handles = {}
        self.publisher_handles["track/mapped"] = self.create_publisher(WATrack, "track/mapped", 1)

        track = WATrack()
        left_points = [
            [43.07313251009781, -89.41296630318291, 0.0],
            [43.07310287254102, -89.41296630318291, 0.0],
            [43.07307380492331, -89.41298112777243, 0.0],
            [43.07304701710655, -89.4130099967099, 0.0],
            [43.07302706872477, -89.41299127091261, 0.0],
            [43.07300826024454, -89.41296786366601, 0.0],
            [43.07298158644825, -89.41297051648716, 0.0]
        ]

        right_points = [
            [43.073133194099526, -89.41301030880639, 0.0],
            [43.073106976261855, -89.41300874832328, 0.0],
            [43.073077338692684, -89.41303059508678, 0.0],
            [43.07305226073839, -89.4130618047489, 0.0],
            [43.07302547291219, -89.4130430789516, 0.0],
            [43.0730055245234, -89.41302123218814, 0.0],
            [43.07298215640256, -89.41302435315434, 0.0]
        ]

        for point in left_points:
            track.left_visible_points.append(Point(x=point[0], y=point[1], z=point[2]))
        
        for point in right_points:
            track.right_visible_points.append(Point(x=point[0], y=point[1], z=point[2]))

        self.logger.info("Publishing track")

        while True:
            self.publisher_handles["track/mapped"].publish(track)


def main(args=None):
    rclpy.init(args=args)

    track_mapper = TrackPublisher()

    rclpy.spin(track_mapper)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
