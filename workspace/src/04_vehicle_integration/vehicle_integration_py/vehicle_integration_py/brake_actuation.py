# General ROS imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_control_msgs.msg import BrakingCommand
from wagrandprix_vehicle_msgs.msg import ActuatorPower

# ------------
# Port from ROS1
# ------------
# import can
from canlib import canlib, Frame
import time
import threading
from wagrandprix_utilities import clamp, scale_to_range


class BrakeActuation(Node):

    def __init__(self):
        super().__init__('brake_actuation')

        self.logger = rclpy.logging.get_logger(self.get_name())
        
        # ------------
        # Parse params
        # ------------
        brake_cmd_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the brake command msg will be shipped on.")
        self.declare_parameter("brake_cmd_topic", "/control/braking", brake_cmd_descriptor)
        self.brake_cmd_topic = self.get_parameter("brake_cmd_topic").value

        actuator_relay_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the actuator relay msg will be shipped on.")
        self.declare_parameter("brake_actuator_relay_topic", "/control/brake_actuator_relay", actuator_relay_descriptor)
        self.actuator_relay_topic = self.get_parameter("brake_actuator_relay_topic").value

        # ------------
        # ROS Entities
        # ------------

        # Create subscriber handles
        self.subscriber_handles = {}
        self.subscriber_handles[self.brake_cmd_topic] = self.create_subscription(BrakingCommand, self.brake_cmd_topic, self.brake_cmd_callback, 1)
        self.publisher_handles = {}
        self.publisher_handles[self.actuator_relay_topic] = self.create_publisher(ActuatorPower, self.actuator_relay_topic, 1)

        # ------------
        # Port from ROS1
        # ------------
        # Actuator values
        self.max = 100 # Max brake request percentage
        self.min = 0 # Min brake request percentage

        # remote brake control (RBC)
        self.rbc_ID = int("CFF7DFE", 16) # "CFF7D8C" "CFF7DFE" "CFF7D0D"
        self.rbc_can_scale_factor = 1

        # create default value to begin with
        init_braking_percentage = self.braking_to_percentage(0)
        self.braking_percentage = self.braking_percentage_to_can(init_braking_percentage)
        # self.rbc_MSG = can.Message(arbitration_id=self.rbc_ID, data=[self.braking_percentage, 255, 255, 255, 255, 255, 255, 255], is_extended_id=True)
        self.rbc_MSG = Frame(id_=self.rbc_ID, data=[self.braking_percentage, 255, 255, 255, 255, 255, 255, 255], dlc=8, flags=4)

        # can intialization
        self.get_logger().info("Initializing CAN messaging to iBooster...")
        # self.bustype = 'socketcan'
        # self.channel = 'can0'
        # self.bus = can.interface.Bus(channel=self.channel, bustype=self.bustype)
        # self.rbc_TASK = self.bus.send_periodic(self.rbc_MSG, .01) # send message at 100hz
        self.ch = canlib.openChannel(
            channel=0,
            flags=canlib.Open.REQUIRE_EXTENDED,
            bitrate= canlib.Bitrate.BITRATE_250K,
        )
        # Set the CAN bus driver type to NORMAL.
        self.ch.setBusOutputControl(canlib.Driver.NORMAL)
        # Activate the CAN chip.
        self.ch.busOn()

        self.thrd_stop = False

        # self.rbc_TASK = self.bus.send_periodic(self.rbc_MSG, .01) # send message at 100hz
        self.get_logger().info("Ready for iBooster power on!")

        # Set default position of the actuator
        self.set_braking_percentage(self.braking_to_percentage(0)) # Should check position of the actuator and set value that way

        hz100 = 1/100 # 100hz
        self.timer100 = self.create_timer(hz100, self.timer100_callback)

        msg = ActuatorPower()
        msg.value = +1.0 # > 0 for on
        self.publisher_handles[self.actuator_relay_topic].publish(msg)

    def timer100_callback(self):
        # Wait until the message is sent or at most 100 ms.
        self.ch.writeWait(self.rbc_MSG, timeout=100)


    def brake_cmd_callback(self, msg):
        """
        Callback for the brake_cmd topic.
        """
        self.get_logger().info(f"Received {msg} on topic {self.brake_cmd_topic}")

        # ------------
        # Port from ROS1
        # ------------

        braking = msg.value
        # calculate new CAN formatted data
        braking_percentage = self.braking_to_percentage(braking)
        self.set_braking_percentage(braking_percentage)


    # ------------
    # Port from ROS1
    # ------------
    def set_braking_percentage(self, braking_percentage):
        """
        Actual setter for the brake percentage request
        Params:
            braking_percentage - level of braking request as percentage of brake force available to iBooster actuator
        """

        # convert percentage for CAN
        self.braking_percentage = self.braking_percentage_to_can(braking_percentage)

        # # create new message
        # new_MSG = can.Message(arbitration_id=self.rbc_ID, data=[self.braking_percentage, 255, 255, 255, 255, 255, 255, 255], is_extended_id=True)

        # # update active message data
        # self.rbc_TASK.modify_data(new_MSG)
        self.rbc_MSG = Frame(id_=self.rbc_ID, data=[self.braking_percentage, 255, 255, 255, 255, 255, 255, 255], dlc=8, flags=4)

    # ------------
    # Port from ROS1
    # ------------
    def braking_to_percentage(self, braking):
        """
        Converts braking input to a percentage
        Params:
            braking - [-1,1]; input from the controller
        """
        new_braking = scale_to_range(braking, 0., 1., self.min, self.max)
        new_braking = clamp(new_braking, self.min, self.max) # definitely overkill
        return new_braking

    # ------------
    # Port from ROS1
    # ------------
    def braking_percentage_to_can(self, braking_percentage):
        """
        Converts braking percentage to value for CAN Message
        Params:
            braking_percentage - level of braking request as percentage of brake force available to iBooster actuator
        """

        braking_can = int(braking_percentage/self.rbc_can_scale_factor)
        return braking_can


def main(args=None):
    rclpy.init(args=args)
    brake = BrakeActuation()
    rclpy.spin(brake)


if __name__ == '__main__':
    main()
