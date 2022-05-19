#! /usr/bin/env python3

__author__ = "Sriram Ashokkumar"
__email__ = "ashokkumar2@wisc.edu"

__author__ = "Victor Freire"
__email__ = "freiremelgiz@wisc.edu"

# ROS imports
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node

# Message imports
from geometry_msgs.msg import Point
from wagrandprix_vehicle_msgs.msg import VehicleState
from wagrandprix_control_msgs.msg import VehicleCommand, SteeringCommand, BrakingCommand, ThrottleCommand

from controls_py.StanleyController import StanleyController
from controls_py.pid import PIDController
import wa_simulator as wa


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.logger = rclpy.logging.get_logger(self.get_name())

        # ------------
        # Parse params
        # ------------
        target_point_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that target point will be shipped on.")
        self.declare_parameter("target_point", "planning", target_point_descriptor)
        self.target_point_topic = self.get_parameter("target_point").value

        vehicle_state_topic_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that provides vehicle state information")
        self.declare_parameter("vehicle_state_topic", "/localization/vehicle/state", vehicle_state_topic_descriptor)
        self.vehicle_state_topic = self.get_parameter("vehicle_state_topic").value
        
        control_algorithm_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="Which control agorithm to run")
        self.declare_parameter("control_algorithm", "StanleyController", control_algorithm_descriptor)
        self.control_algorithm = self.get_parameter("control_algorithm").value


        self.logger.info(f"control_algorithm: {self.control_algorithm}")


        # ------------
        # ROS Entities
        # ------------
        
        # Create publisher handles
        self.publisher_handles = {}

        self.publisher_handles["steering"] = self.create_publisher(SteeringCommand,'/controls/steering',1)
        self.publisher_handles["braking"] = self.create_publisher(BrakingCommand,'/controls/braking',1)
        self.publisher_handles["throttle"] = self.create_publisher(ThrottleCommand,'/controls/throttle',1)

        # Create subcriber handles
        self.subscriber_handles = {}

        self.logger.info(f"vehicle_state_topic: {self.vehicle_state_topic}")
        self.logger.info(f"target_point_topic: {self.target_point_topic}")

        self.subscriber_handles[self.vehicle_state_topic] = self.create_subscription(VehicleState, self.vehicle_state_topic, self._save_state, 1)
        self.subscriber_handles[self.target_point_topic] = self.create_subscription(Point, self.target_point_topic, self._save_target, 1)
        
        # ------------------------
        # Initialize Class Members
        # ------------------------

        # if self.control_algorithm == "PID":
        #     TODO: add PID controller file and intialize it here

        # else:    # self.control_algorithm == "StanleyController"":
        #     self.controller = StanleyController(wa.WASystem(), VehicleState(), [1,1,1], wa.WAVehicleInputs())

        


        self.controller = PIDController(wa.WASystem(), wa.WAVehicleInputs(), VehicleState(), [1,1,1])

        self.vehicle_command = VehicleCommand()

        # Send cmd at 100 Hz
        self.step = 0.01
        self.received_VehicleState = False
        self.received_VehicleTarget = False
        self.timer = self.create_timer(0.01, self.send_control)

        # # Use sim time by default
        # # sim_time = Parameter('use_sim_time', Parameter_handles["steering"].publish(self.vehicle_command.steering)
        # self.publisher_handles["braking"].publish(self.vehicle_command.braking)
        # self.publisher_handles["throttle"].publishtate(), [0,0,0]) #need to add target point info
        # self.controller = StanleyController(wa.WASystem(), VehicleState(), [1,1,1], wa.WAVehicleInputs())
        # # We could just use cars current pos as a placeholder for target to initialize it if we need
        # # So [vehicle_state.pose.position.x, ...y, ...z]
        # # - Raj

        # self.vehicle_command = VehicleCommand()

        # # Subs and Pubs   -----------   Replace with right ones -- done??
        # self.sub_state = self.create_subscription(VehicleState, "/localization/state", self._save_state, 1) #need another one
        # self.sub_point = self.create_subscription(Point, "/control/planning", self._save_target, 1)
        # # self.pub_cmd = self.create_publisher(VehicleCommand,'/control/input',1)
        # self.pub_steering = self.create_publisher(SteeringCommand,'/control/steering',1)
        # self.pub_braking = self.create_publisher(BrakingCommand,'/control/braking',1)
        # self.pub_throttle = self.create_publisher(ThrottleCommand,'/control/throttle',1)


        # # self.sub_traj = self.create_subscription(CarTrajectory,      --- remove?
        # #         "/control/trajectory", self._save_trajectory, 1)

        # # Send cmd at 100 Hz
        # self.step = 0.01  # just for reference for now
        # self.received_VehicleState = False
        # self.received_VehicleTarget = False
        # self.timer = self.create_timer(0.5, self.send_control)

        #delete
        # self.fakeValues = [[0, .15, 0],[-.2, .30, 0], [-.4, .60, 0], [-.6, .80, 0], [-.8, 1, 0], [-1, .8, 0], [-.5, 0, 0.1], [0, 0, 0.4], [.3, 0, 0.8], [.6, 0, 1], [.8, 0, 1], [1, 0, 1]]
        # self.idx = 0

    # Callback to store trajectory setpoint
    # def _save_trajectory(self, msg):
    #     self.received_traj = True
    #     self.controller.sp = msg

    # Callback to store state data from estimator
    def _save_state(self, msg):
        # self.get_logger().info('received state')
        self.received_VehicleState = True
        self.controller.VehicleState = msg
    
    def _save_target(self, msg):
        # self.get_logger().info('received target')
        self.received_VehicleTarget = True
        self.controller.target_point = msg.x, msg.y, msg.z


    # Send appropriate control signal to input topic
    def send_control(self):
        # self.controller.VehicleState = ( ((0,0,0),(0,0,0,0)) , ((0,0,0),(0,0,0)) , ((0,0,0),(0,0,0)) ) 
        # self.received_VehicleTarget = True
        if self.received_VehicleTarget:

            self.controller.advance(self.step)
            # self.logger.info('Publishing vehicle command')
            self.vehicle_command.steering.value, self.vehicle_command.throttle.value, self.vehicle_command.braking.value = self.controller.steering, self.controller.throttle, self.controller.braking
            # self.vehicle_command.steering.value, self.vehicle_command.throttle.value, self.vehicle_command.braking.value = self.fakeValues[self.idx][0], self.fakeValues[self.idx][1], self.fakeValues[self.idx][2]
            # self.idx += 1
            self.publisher_handles["steering"].publish(self.vehicle_command.steering)
            self.publisher_handles["braking"].publish(self.vehicle_command.braking)
            self.publisher_handles["throttle"].publish(self.vehicle_command.throttle)
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