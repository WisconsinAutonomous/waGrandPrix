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
from controls_py.TestingPlanner import TestingPlanner
import numpy as np

from wa_simulator_ros_msgs.msg import WATrack
from wagrandprix_vehicle_msgs.msg import VehicleState
from geometry_msgs.msg import Point

## Class to generate and publish waypoint msgs
class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')
        # makes it so that nodes with timers use simulation time published in /clock instead of the cpu wall clock
        # sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        # self.set_parameters([sim_time])

        # Create persistent path msg
        self.msg_waypoint = Point()

        # Create publisher and subscribers
        self.pub_waypoint = self.create_publisher(Point, '/control/planning', 1)
        self.sub_track = self.create_subscription(WATrack, '/localization/track/mapped', self._receive_track, 1)
        self.sub_state = self.create_subscription(VehicleState, '/localization/vehicle/state', self._receive_state, 1)

        # Create Centerline Planner class
        self.tp = TestingPlanner()

        self.received_track = False
        self.received_state = False

    # Recieve set of track waypoints
    def _receive_track(self, msg):
        self.received_track = True
        self.tp.track_left = []
        for point in msg.left_visible_points:
            self.tp.track_left.append([point.x, point.y, point.z])

        self.tp.track_right = []
        for point in msg.right_visible_points:
            self.tp.track_right.append([point.x, point.y, point.z])
        self.send_waypoint()

    # Recieve vehicle state
    def _receive_state(self ,msg):
        self.received_state = True
        self.tp.pos = msg.pose.position
        self.send_waypoint()

    # Publish waypoint to follow
    def send_waypoint(self):
        if self.received_track and self.received_state:
            waypoint = self.tp.get_waypoint()
            if waypoint != None:
                self.msg_waypoint.x, self.msg_waypoint.y, self.msg_waypoint.z = waypoint
                self.pub_waypoint.publish(self.msg_waypoint)
            # self.get_logger().info('Publishing waypoint')


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