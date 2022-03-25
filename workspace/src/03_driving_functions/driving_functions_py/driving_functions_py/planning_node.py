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
from rclpy.parameter import Parameter
from rclpy.node import Node
from driving_functions_py.CenterlinePlanner import CenterlinePlanner
import numpy as np

from wa_simulator_ros_msgs.msg import WATrack
from wagrandprix_vehicle_msgs.msg import VehicleState
from geometry_msgs.msg import Point

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
        self.sub_track = self.create_subscription(WATrack, '/localization/track', self._receive_track, 1)
        self.sub_state = self.create_subscription(VehicleState, '/localization/state', self._receive_state, 1)
        # Create Centerline Planner class
        self.cp = CenterlinePlanner()

        self.timer = self.create_timer(0.5, self.send_waypoint)
        self.received_Track = False
        self.received_State = False

    # Recieve set of track waypoints
    def _receive_track(self,msg):
        self.received_Track = True
        self.cp.track = msg

    # Recieve vehicle state
    def _receive_state(self,msg):
        self.received_State = True
        self.cp.pos = msg.position

    # Publish waypoint to follow
    def send_waypoint(self):
        self.get_logger().info('attempt publish')
        if self.received_Track and self.received_State:
            # get waypoint
            waypoint = self.cp.get_waypoint(self.track)

            # publish path
            self.get_logger().info('Publishing waypoint')
            self.pub_waypoint.publish(waypoint)


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
