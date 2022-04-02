#! /usr/bin/env python3

__author__ = "Victor Freire"
__email__ = "freiremelgiz@wisc.edu"

"""
Test file which uses VehicleState and preset
track data to publish a WATrack msg for planning.

Subscribers
/localization/state              - VehicleState

Publishers
/localization/track              - WATrack
"""

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from controls_py.CenterlinePlanner import CenterlinePlanner
import numpy as np

from wa_simulator_ros_msgs.msg import WATrack
from wagrandprix_vehicle_msgs.msg import VehicleState
from geometry_msgs.msg import Point

## Class to generate and publish waypoint msgs
class TrackNode(Node):
    def __init__(self):
        super().__init__('track_node')
        # makes it so that nodes with timers use simulation time published in /clock instead of the cpu wall clock
        # sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        # self.set_parameters([sim_time])

        # Create persistent path msg
        self.msg_track = WATrack()

        # Create publisher and subscribers
        self.pub_track = self.create_publisher(WATrack, '/localization/track', 1)
        self.sub_state = self.create_subscription(VehicleState, '/localization/state', self._receive_state, 1)

        self.timer = self.create_timer(0.5, self.send_track)    
        self.received_state = False

    # Recieve vehicle state
    def _receive_state(self,msg):
        self.received_state = True
        self.cp.pos = msg.position

    # Publish waypoint to follow
    def send_track(self):
        self.get_logger().info('attempt publish')
        if self.received_state:
            # self.msg_track = 0
            self.get_logger().info('Publishing track')
            self.pub_track.publish(self.msg_track)


# Entry point
def main(args=None):
    rclpy.init(args=args)

    # Create the PlanningNode Object
    tn = TrackNode()

    rclpy.spin(tn)

    # Destroy the nodes explicitly
    tn.destroy_node()
    rclpy.shutdown()


if __name__  == "__main__":
    main()
