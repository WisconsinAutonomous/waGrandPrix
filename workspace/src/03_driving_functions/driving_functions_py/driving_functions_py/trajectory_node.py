#! /usr/bin/env python3

__author__ = "Victor Freire"
__email__ = "freiremelgiz@wisc.edu"

"""
Publishes CarTrajectory msgs to /control/trajectory.
The node listens for CarFlatData msgs and generates
appropriate trajectories.

Subscribers
/localization/state              - CarState

Publishers
/control/trajectory              - CarTrajectory

The trajectory will be published in state-space

"""

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from BicyclePlanner import FlatData, BicyclePlanner
import numpy as np
from wagrandprix_map_msgs.msg import DetectedTrack

## Class to publish a stream of setpoints as the trajectory
# T_mat         - Trajectory matrix [nx7]
class TrajStream(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        # Use sim time by default
        sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])
        # Don't publish until we receive T mat
        self.updating = True # Semaphore for updating T mat
        # Create persistent traj msg
        self.msg_traj = Trajectory()
        # Create publishers
        self.pub_traj = self.create_publisher(Trajectory,'/control/trajectory', 1)
        # Initialize empty Trajectory
        self.T = []
        self.counter = 0 # Number of setpoints published
        # Create FlatVCP class
        self.bp = BicyclePlanner()
        self.data = FlatData()
        # Create subs
        self.sub_plan = self.create_subscription(CarFlatData,
                '/control/planning', self._receive_flat_data, 1)
        self.sub_state = self.create_subscription(CarState,
                '/localization/state', self._receive_state, 1)
        self.x = np.zeros((4,1))

        # Publish traj at 100 Hz
        self.timer = self.create_timer(0.01, self._stream)

    # Receive a set of Flat Data and generate trajectory
    def _receive_flat_data(self,msg):
        # Populate FlatData
        self.data.x_f[0][0] = msg.x.x
        self.data.x_f[1][0] = msg.x.y
        self.data.x_f[2][0] = msg.x.v
        self.data.x_f[3][0] = msg.x.psi
        if msg.v_max:
            self.data.v_max = msg.v_max
        else:
            self.data.v_max = 1000.
        if msg.a_max:
            self.data.a_max = msg.a_max
        else:
            self.data.a_max = 1000.
        if msg.gamma_max:
            self.data.gamma_max = msg.gamma_max
        else:
            self.data.gamma_max = np.radians(89.5)
        if msg.l:
            self.data.L = msg.l
        else:
            self.data.L =  2.685 # Chevy Bolt L
        if msg.nu:
            self.data.nu = msg.nu
        else:
            self.data.nu = 1.
        self.data.x_0 = self.x.copy() # Initial state
        self.data.x_0[2,0] = max(0.0, self.data.x_0[2,0])
        # Solve problem
        errorcode = self.bp.solve(self.data)
        if not errorcode: # Ignore infeasibles
            x, u, t = self.bp.full_traj() # Compute state-space
            # Update TrajStream
            self.update(np.vstack((x,u)).T)
        else:
            print("Infeasible problem. Skipping...")


    # Receive state msgs for x_0 initial state
    def _receive_state(self,msg):
        self.x[0][0] = msg.x
        self.x[1][0] = msg.y
        self.x[2][0] = msg.v
        self.x[3][0] = msg.psi

    # Stage msg and publish
    def _stream(self):
        self.prepare_next()
        self.send_next()

    # Update the trajectory with a new matrix [nx7]
    def update(self, T_mat):
        if T_mat.shape[1] == 7: # Assert columns
            self.updating = True # Block CarTrajectory publisher while updating
            self.T = T_mat # Store trajectory
            self.counter = 0 # Reset counter
            self.updating = False # Green light for publisher

    # Prepare the next setpoint
    def prepare_next(self):
        if not self.updating and self.has_next():
            self.msg_traj.x.x       = self.T[self.counter][0]
            self.msg_traj.x.y       = self.T[self.counter][1]
            self.msg_traj.x.v       = self.T[self.counter][2]
            self.msg_traj.x.psi     = self.T[self.counter][3]
            self.msg_traj.u.v_dot   = self.T[self.counter][4]
            self.msg_traj.u.psi_dot = self.T[self.counter][5]
            self.msg_traj.u.gamma   = self.T[self.counter][6]

    # Publish next setpoint
    def send_next(self):
        if not self.updating:
            # Publish
            self.pub_traj.publish(self.msg_traj)
            # Increment published count
            if self.has_next():
                self.counter +=1

    # Returns True if there are more setpoints to publish
    def has_next(self):
        return len(self.T)>self.counter




# Entry point
def main(args=None):
    rclpy.init(args=args)

    # Create the TrajStream Object
    ts = TrajStream()

    rclpy.spin(ts)

    # Destroy the nodes explicitly
    ts.destroy_node()
    rclpy.shutdown()


if __name__  == "__main__":
    main()
