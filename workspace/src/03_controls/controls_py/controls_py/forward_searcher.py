"""
Calculates a few waypoints that define the future high level path that the vehicle will attempt to follow

Does not account for obstacles or other vehicles
"""

# General ROS imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_map_srvs.srv import SearchForward


class ForwardSearcher(Node):

    def __init__(self):
        super().__init__('forward_searcher')

        self.logger = rclpy.logging.get_logger(self.get_name())

        # ------------
        # Parse params
        # ------------
        mapped_track_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic the perceived track detections will be shipped on.")
        self.declare_parameter("mapped_track_topic", "/localization/track/mapped", mapped_track_descriptor)
        self.mapped_track_topic = self.get_parameter("mapped_track_topic").value

        # ------------
        # ROS Entities
        # ------------

        self.logger.info(f"mapped_track_topic: {self.mapped_track_topic}")

        # Create subscriber handles
        self.subscriber_handles = {}
        self.subscriber_handles[self.mapped_track_topic] = self.create_subscription(MappedTrack, self.mapped_track_topic, self.mapped_track_callback, 1)

        # Create service handles
        self.service_handles = {}
        self.service_handles["planner/global"] = self.create_service(SearchForward, "planner/global", self.forward_search_request_callback)

        # ------------------------
        # Initialize Class Members
        # ------------------------

    def mapped_track_callback(self, msg):
        """
        Callback for the mapped track topic.
        """
        self.logger.debug(f"Received {msg} on topic {self.mapped_track_topic}")

    def forward_search_request_callback(self, request, response):
        """
        Callback for the forward search request.
        """
        self.logger.debug(f"Received {request} on service topic planner/global")

        if request.type == request.GLOBAL_PATH_REQUEST_SAFE:
            self.logger.debug("Creating safe path.")
        else:
            raise NotImplementedError(f"{request.type} is unimplemented.")

        self.logger.debug(f"Responding with {response}")
        return response


def main(args=None):
    rclpy.init(args=args)

    forward_searcher = ForwardSearcher()

    rclpy.spin(forward_searcher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
