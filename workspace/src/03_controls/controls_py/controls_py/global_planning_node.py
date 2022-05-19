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
from controls_py.GlobalPlanner import GlobalPlanner
import numpy as np

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
        self.declare_parameter("vehicle_state_topic", "/localization/vehicle/state", vehicle_state_topic_descriptor)
        self.vehicle_state_topic = self.get_parameter("vehicle_state_topic").value

        # ------------
        # ROS Entities
        # ------------
        
        # Create publisher handles
        self.publisher_handles = {}

        self.publisher_handles["planning"] = self.create_publisher(Point, 'planning', 1)

        # Create subcriber handles
        self.subscriber_handles = {}

        self.logger.info(f"vehicle_state_topic: {self.vehicle_state_topic}")

        self.subscriber_handles[self.vehicle_state_topic] = self.create_subscription(VehicleState, self.vehicle_state_topic, self._receive_state, 1)

        # makes it so that nodes with timers use simulation time published in /clock instead of the cpu wall clock
        # sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        # self.set_parameters([sim_time])

        # Create persistent path msg
        self.msg_waypoint = Point()

        # Create Centerline Planner class
        self.gp = GlobalPlanner("/home/wagrandprix/wagrandprix/workspace/src/03_controls/controls_py/resource/evgrandprix_center_gps.csv")

        self.received_state = False

    # Recieve vehicle state
    def _receive_state(self, msg):
        self.received_state = True
        self.gp.pos_x, self.gp.pos_y = msg.pose.position.x, msg.pose.position.y
        self.send_waypoint()

    # Publish waypoint to follow
    def send_waypoint(self):
        if self.received_state:
            waypoint = self.gp.SearchTargetPoint()
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
