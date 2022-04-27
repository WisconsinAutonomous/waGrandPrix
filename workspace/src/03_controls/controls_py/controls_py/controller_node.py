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
    /localization/state - VehicleState
    /control/planning - Point

Publishers:
    /control/input - VehicleCommand

"""

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from sys import argv
from geometry_msgs.msg import Point
from wagrandprix_vehicle_msgs.msg import VehicleState
from wagrandprix_control_msgs.msg import VehicleCommand, SteeringCommand, BrakingCommand, ThrottleCommand
from controls_py.StanleyController import StanleyController
import wa_simulator as wa


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        # Use sim time by default
        # sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        # self.set_parameters([sim_time])

        # self.controller = StanleyController(VehicleState(), [0,0,0]) #need to add target point info
        self.controller = StanleyController(wa.WASystem(), VehicleState(), [1,1,1], wa.WAVehicleInputs())
        # We could just use cars current pos as a placeholder for target to initialize it if we need
        # So [vehicle_state.pose.position.x, ...y, ...z]
        # - Raj

        self.vehicle_command = VehicleCommand()

        # Subs and Pubs   -----------   Replace with right ones -- done??
        self.sub_state = self.create_subscription(VehicleState, "/localization/state", self._save_state, 1) #need another one
        self.sub_state = self.create_subscription(Point, "/control/planning", self._save_target, 1)
        # self.pub_cmd = self.create_publisher(VehicleCommand,'/control/input',1)
        self.pub_steering = self.create_publisher(SteeringCommand,'/control/steering',1)
        self.pub_braking = self.create_publisher(BrakingCommand,'/control/braking',1)
        self.pub_throttle = self.create_publisher(ThrottleCommand,'/control/throttle',1)


        # self.sub_traj = self.create_subscription(CarTrajectory,      --- remove?
        #         "/control/trajectory", self._save_trajectory, 1)

        # Send cmd at 100 Hz
        self.step = 0.01  # just for reference for now
        self.received_VehicleState = False
        self.received_VehicleTarget = False
        self.timer = self.create_timer(0.5, self.send_control)

        #delete
        # self.fakeValues = [[0, .15, 0],[-.2, .30, 0], [-.4, .60, 0], [-.6, .80, 0], [-.8, 1, 0], [-1, .8, 0], [-.5, 0, 0.1], [0, 0, 0.4], [.3, 0, 0.8], [.6, 0, 1], [.8, 0, 1], [1, 0, 1]]
        # self.idx = 0

    # Callback to store trajectory setpoint
    # def _save_trajectory(self, msg):
    #     self.received_traj = True
    #     self.controller.sp = msg

    # Callback to store state data from estimator
    def _save_state(self, msg):
        self.received_VehicleState = True
        self.controller.VehicleState = msg
    
    def _save_target(self, msg):
        self.received_VehicleTarget = True
        self.controller.target_point = msg.x, msg.y, msg.z


    # Send appropriate control signal to input topic
    def send_control(self):
        self.controller.VehicleState = ( ((0,0,0),(0,0,0,0)) , ((0,0,0),(0,0,0)) , ((0,0,0),(0,0,0)) ) 
        self.received_VehicleTarget = True
        if self.received_VehicleTarget:

            self.controller.advance(self.step)
            self.get_logger().info('Publishing vehicle command')
            print(self.controller.steering)
            self.vehicle_command.steering.value, self.vehicle_command.throttle.value, self.vehicle_command.braking.value = self.controller.steering, self.controller.throttle, self.controller.braking
            # self.vehicle_command.steering.value, self.vehicle_command.throttle.value, self.vehicle_command.braking.value = self.fakeValues[self.idx][0], self.fakeValues[self.idx][1], self.fakeValues[self.idx][2]
            # self.idx += 1
            self.pub_steering.publish(self.vehicle_command.steering)
            self.pub_throttle.publish(self.vehicle_command.throttle)
            self.pub_braking.publish(self.vehicle_command.braking)
            # self.pub_cmd.publish(self.vehicle_command) # Send control
            # self.controller.update_u() # Get next control


# Entry point
def main(args=None):
    rclpy.init(args=args)
    cn = ControllerNode()
    rclpy.spin(cn)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cn.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
