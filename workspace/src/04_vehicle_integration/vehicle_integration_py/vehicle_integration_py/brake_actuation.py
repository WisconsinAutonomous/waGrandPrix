# General ROS imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import specific message types
from wagrandprix_control_msgs.msg import BrakeCommand

# ------------
# Port from ROS1
# ------------
import can
from common.utilities.utilities import clamp, scale_to_range


class BrakeActuation(Node):

    def __init__(self):
        super().__init__('brake_actuation')

        self.logger = rclpy.logging.get_logger(self.get_name())
        
        # ------------
        # Parse params
        # ------------
        brake_cmd_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the brake command msg will be shipped on.")
        self.declare_parameter("brake_cmd_topic", "/actuation/commands/brake", brake_cmd_descriptor)
        self.brake_cmd_topic = self.get_parameter("brake_cmd_topic").value

        # ------------
        # ROS Entities
        # ------------

        # Create subscriber handles
        self.subscriber_handles = {}
        self.subscriber_handles[self.brake_cmd_topic] = self.create_subscription(BrakeCommand, self.brake_cmd_topic, self.brake_cmd_callback, 1)

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
        self.rbc_MSG = can.Message(arbitration_id=self.rbc_ID, data=[self.braking_percentage, 255, 255, 255, 255, 255, 255, 255], is_extended_id=True)

        # can intialization
        rospy.logdebug("Initializing CAN messaging to iBooster...")
        self.bustype = 'socketcan'
        self.channel = 'can0'
        self.bus = can.interface.Bus(channel=self.channel, bustype=self.bustype)
        self.rbc_TASK = self.bus.send_periodic(self.rbc_MSG, .01) # send message at 100hz
        rospy.logdebug("Ready for iBooster power on!")

        # Set default position of the actuator
        self.set_braking_percentage(self.braking_to_percentage(0)) # Should check position of the actuator and set value that way


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

        # create new message
        new_MSG = can.Message(arbitration_id=self.rbc_ID, data=[self.braking_percentage, 255, 255, 255, 255, 255, 255, 255], is_extended_id=True)

        # update active message data
        self.rbc_TASK.modify_data(new_MSG)

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
