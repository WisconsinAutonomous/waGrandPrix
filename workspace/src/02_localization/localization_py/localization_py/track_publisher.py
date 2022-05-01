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
            [42.96876879357948, -89.50527087819115,0.0],
            [42.96876879357948, -89.50520019341766,0.0],
            [42.968766912785256, -89.50511922722255,0.0],
            [42.96877255516771, -89.50505111280444,0.0],
            [42.9687622107995, -89.50497143178706,0.0]
        ]

        right_points = [
            [42.968736820070134, -89.50526959301345,0.0],
            [42.9687396412628, -89.50520404895076,0.0],
            [42.968736820070134, -89.50512179757796,0.0],
            [42.96874246245534, -89.50505111280444,0.0],
            [42.9687396412628, -89.50496372072085,0.0]
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
