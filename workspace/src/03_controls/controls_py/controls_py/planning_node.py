#! /usr/bin/env python3

__author__ = "Victor Freire"
__email__ = "freiremelgiz@wisc.edu"

"""
Publishes Point msgs to /control/planning.
The node listens for Track and VehicleState
msgs and generates appropriate trajectories.

Subscribers
/localization/track              - Track
/localization/state              - VehicleState

Publishers
/control/planning                - Path

The trajectory will be published in state-space

"""

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.parameter import Parameter
from rclpy.node import Node
from controls_py.CenterlinePlanner import CenterlinePlanner
import numpy as np

from wa_simulator_ros_msgs.msg import WATrack, WAVehicle
from wagrandprix_vehicle_msgs.msg import VehicleState
from geometry_msgs.msg import Point

## Class to generate and publish waypoint msgs
class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')
        
        self.logger = rclpy.logging.get_logger(self.get_name())
        # ------------
        # Parse params
        # ------------
        vehicle_state_topic_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that provides vehicle state information")
        self.declare_parameter("vehicle_state_topic", "/sim/vehicle/state", vehicle_state_topic_descriptor)
        self.vehicle_state_topic = self.get_parameter("vehicle_state_topic").value

        track_topic_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that provides track information")
        self.declare_parameter("track_topic", "/sim/track/visible", track_topic_descriptor)
        self.track_topic = self.get_parameter("track_topic").value
        # ------------
        # ROS Entities
        # ------------
        
        # Create publisher handles
        self.publisher_handles = {}

        self.publisher_handles["planning"] = self.create_publisher(Point, 'planning', 1)

        # Create subcriber handles
        self.subscriber_handles = {}

        self.logger.info(f"vehicle_state_topic: {self.vehicle_state_topic}")
        self.logger.info(f"track_topic: {self.track_topic}")

        self.subscriber_handles[self.track_topic] = self.create_subscription(WATrack, self.track_topic, self._receive_track, 1)
        self.subscriber_handles[self.vehicle_state_topic] = self.create_subscription(WAVehicle, self.vehicle_state_topic, self._receive_state, 1)

        # makes it so that nodes with timers use simulation time published in /clock instead of the cpu wall clock
        # sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        # self.set_parameters([sim_time])

        # Create persistent path msg
        self.msg_waypoint = Point()

        # Create Centerline Planner class
        self.cp = CenterlinePlanner()

        self.received_track = False
        self.received_state = False

        # self.pointList = [[55,132.9,0], [55.2,134,0], [55.4,134.1,0], [55.6,133.5,0], [55.8,133,0], [56,130,0], [56.2,128,0]]
        # self.pointList = [[40.43747729776361, -86.94419320655354, 0.0], [40.43746096602283, -86.94420929980735, 0.0], [40.437444634278066, -86.94422002864324, 0.0]]
        # self.idx = 0
        # self.idx2 = 0

        # [ 54.8 132.9   0. ]

    # Recieve set of track waypoints
    def _receive_track(self, msg):
        self.received_track = True
        self.cp.track_left = []
        for point in msg.left_visible_points:
            self.cp.track_left.append([point.x, point.y, point.z])

        self.cp.track_right = []
        for point in msg.right_visible_points:
            self.cp.track_right.append([point.x, point.y, point.z])
        self.send_waypoint()

    # Recieve vehicle state
    def _receive_state(self ,msg):
        self.received_state = True
        self.cp.pos = msg.pose.position
        self.send_waypoint()

    # Publish waypoint to follow
    def send_waypoint(self):
        # if self.received_track and self.received_state and self.idx < len(self.pointList):
        #     self.msg_waypoint.x, self.msg_waypoint.y, self.msg_waypoint.z = [float(self.pointList[self.idx][0]), float(self.pointList[self.idx][1]), float(self.pointList[self.idx][2])]
        #     self.publisher_handles["planning"].publish(self.msg_waypoint)
        #     self.idx += 1
        
        # if self.received_state:
        #     self.msg_waypoint.x, self.msg_waypoint.y, self.msg_waypoint.z = [float(self.pointList[self.idx][0]), float(self.pointList[self.idx][1]), float(self.pointList[self.idx][2])]
        #     self.publisher_handles["planning"].publish(self.msg_waypoint)
        #     self.idx2 += 1
        #     if self.idx2 % 10 == 0:
        #         self.idx += 1




        if self.received_track and self.received_state:
            waypoint = self.cp.get_waypoint()
            if waypoint != None:
                self.msg_waypoint.x, self.msg_waypoint.y, self.msg_waypoint.z = waypoint
                self.publisher_handles["planning"].publish(self.msg_waypoint)
            # self.logger.info('Publishing waypoint')


# Entry point
def main(args=None):
    rclpy.init(args=args)

    # Create the PlanningNode Object
    pn = PlanningNode()

    rclpy.spin(pn)

    # Destroy the nodes explicitly
    pn.destroy_node()
    rclpy.shutdown()


if __name__  == "__main__":
    main()
