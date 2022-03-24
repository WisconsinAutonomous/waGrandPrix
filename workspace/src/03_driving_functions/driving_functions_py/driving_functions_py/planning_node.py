#! /usr/bin/env python3

__author__ = "Victor Freire"
__email__ = "freiremelgiz@wisc.edu"

"""
Publishes Path msgs to /control/planning.
The node listens for Track msgs and generates
appropriate trajectories.

Subscribers
/localization/state              - Track
/localization/state              - VehicleState

Publishers
/control/planning                - Path

The trajectory will be published in state-space

"""

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from driving_functions_py.CenterlinePlanner import CenterlinePlanner
import numpy as np
from wagrandprix_map_msgs.msg import TrackBoundaries, Point
from wagrandprix_vehicle_msgs.msg import VehicleState

## Class to generate and publish path msgs
# T_mat         - Trajectory matrix [nx7]
class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')
        # Use sim time by default
        sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])
        # Create persistent path msg
        self.msg_waypoint = Point()
        # Create publisher
        self.pub_waypoint = self.create_publisher(Point, '/control/planning', 1)
        # Create subscribers
        self.sub_track = self.create_subscription(TrackBoundaries, '/localization/track', self._receive_track, 1)
        self.sub_state = self.create_subscription(VehicleState, '/localization/state', self._receive_state, 1)
        # Create Centerline Planner class
        self.cp = CenterlinePlanner()

        self.timer = self.create_timer(0.5, self.send_waypoint)
        self.received_Track = False
        self.received_State = False

    # Recieve set of track waypoints
    def _receive_track(self,msg):
        self.received_Track = True
        self.track = msg

    # Recieve vehicle state
    def _receive_state(self,msg):
        self.received_State = True
        self.state = msg

    # Publish waypoint to follow
    def send_waypoint(self):
        if self.received_Track and self.received_State:
            # get path
            waypoints = self.cp.get_path(self.track)

            # build path msg
            # for wp in waypoints:
            #     point = Point()
            #     point.x, point.y, point.z = wp
            #     self.msg_path.waypoints.append(point)

            # publish path
            self.get_logger().info('Publishing')
            self.pub_waypoint.publish(self.msg_waypoint)


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
