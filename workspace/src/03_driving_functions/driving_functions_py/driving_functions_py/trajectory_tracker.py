# General ROS imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_interfaces.msg import Waypoint
from wagrandprix_interfaces.srv import SearchForward

class TrajectoryTracker(Node):

    def __init__(self):
        super().__init__('trajectory_tracker')

        self.logger = rclpy.logging.get_logger(self.get_name())

        # ------------
        # Parse params
        # ------------
        forward_search_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic a request can be made to do a forward search")
        self.declare_parameter("forward_search_topic", "planner/global", forward_search_descriptor)
        self.forward_search_topic = self.get_parameter("forward_search_topic").value

        # ------------
        # ROS Entities
        # ------------

        # Create client handles
        self.client_handles = {}
        self.client_handles["planner/global"] = self.create_client(SearchForward, self.forward_search_topic)

        for name, client in self.client_handles.items():
            while not client.wait_for_service(timeout_sec=1.0):
                self.logger.info(f"Service '{name}' is not ready. Waiting again...")


        # ------------------------
        # Initialize Class Members
        # ------------------------

        self.forward_search_request = SearchForward.Request()

        # TODO: hardcode for now
        self.start_idx = 0
        self.end_idx = 0
        self.path_type = self.forward_search_request.GLOBAL_PATH_REQUEST_SAFE
        self.path = []
        self.future = None

        # ----------
        # Timer loop
        # ----------
        # Periodic publishing
        timer_period = 1 / 100  # Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        This callback runs at 100Hz. It will request a forward search and update the path to follow.
        """

        if self.future is None or self.future.done():
            if self.future is not None and self.future.done():
                self.path = self.future.result().path

            self.forward_search_request.start_idx = self.start_idx
            self.forward_search_request.end_idx = self.end_idx

            self.forward_search_request.type = self.path_type

            self.future = self.client_handles["planner/global"].call_async(self.forward_search_request)


def main(args=None):
    rclpy.init(args=args)

    trajectory_tracker = TrajectoryTracker()

    rclpy.spin(trajectory_tracker)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
