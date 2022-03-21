#! /usr/bin/env python3

__author__ = "Victor Freire"
__email__ = "freiremelgiz@wisc.edu"

"""
This node uses the classes in Controllers.py and provides
an interface to the rest of the ROS2 network. The node will
not start publishing control inputs until it has received a
message in /controller/trajectory

This node takes 1 argument:
1 - mode: controller type (required)

Subscribers:
    /localization/state - CarState
    /controller/trajectory - CarTrajectory

Publishers:
    /control/input - CarInput

"""

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from sys import argv
from wauto_control_msgs.msg import CarState, CarInput, CarTrajectory


class ControllerNode(Node):
    def __init__(self, mode):
        super().__init__('controller_node')
        # Use sim time by default
        sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])
        # Import the right controller
        if 'fbl'==mode:
            from control.Controllers import FBL
            self.controller = FBL(CarState(), CarTrajectory(), CarInput())
        elif 'none'==mode:
            raise RuntimeError("ERROR: You must pass a controller " +
                    "type (mode) as argument to this node.")
        else:
            raise RuntimeError("ERROR: Controller {} is not " +
                    "implemented.".format(mode))
        self.mode = mode

        # Subs and Pubs
        self.sub_state = self.create_subscription(CarState,
                "/localization/state", self._save_state, 1)
        self.sub_safe = None # TODO Add subscriber for Safety
        self.pub_cmd = self.create_publisher(CarInput,'/control/input',1)
        self.sub_traj = self.create_subscription(CarTrajectory,
                "/control/trajectory", self._save_trajectory, 1)

        # Send cmd at 100 Hz
        self.received_traj = False
        self.timer = self.create_timer(0.01, self.send_control)

    # Callback to store trajectory setpoint
    def _save_trajectory(self, msg):
        self.received_traj = True
        self.controller.sp = msg

    # Callback to store state data from estimator
    def _save_state(self, msg):
        self.controller.x = msg

    # Send appropriate control signal to input topic
    def send_control(self):
        if self.received_traj:
            self.pub_cmd.publish(self.controller.u) # Send control
            self.controller.update_u() # Get next control



# Entry point
def main(args=None):
    rclpy.init(args=args)

    # Process mode argument
    if(len(argv) != 1):
        mode = str(argv[1]).lower()
    else:
        mode = "none"
    print("Controller Type: {}".format(mode))

    cn = ControllerNode(mode)

    rclpy.spin(cn)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cn.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
