# General ROS imports
import rclpy
from canlib import canlib, Frame
import time
import threading
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from wagrandprix_utilities import clamp, scale_to_range

# Import specific message types
from wagrandprix_control_msgs.msg import SteeringCommand

class SteeringActuation(Node):

    def __init__(self):
        super().__init__('steering_actuation')

        self.logger = rclpy.logging.get_logger(self.get_name())
        
        # ------------
        # Parse params
        # ------------
        steering_cmd_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that the steering command msg will be shipped on.")
        self.declare_parameter("steering_cmd_topic", "/control/steering", steering_cmd_descriptor)
        self.steering_cmd_topic = self.get_parameter("steering_cmd_topic").value

        # ------------
        # ROS Entities
        # ------------

        # Create subscriber handles
        self.subscriber_handles = {}
        self.subscriber_handles[self.steering_cmd_topic] = self.create_subscription(SteeringCommand, self.steering_cmd_topic, self.steering_cmd_callback, 1)

        # Actuator values
        self.offset = 195 # offset from "global 0" to "local 0" (local 0 is defined as the postion of the actuator where the wheels point directly forward)
        self.max = 64 # Max angle in degrees from local 0
        self.min = -64 # Min angle in degrees from local 0

	    # cruise control vehicle speed (CCVS)
        # TO DO: need to modify to work with canlib
        self.ccvs_ID = int("18FEF127", 16)
        # self.ccvs_MSG = can.Message(arbitration_id=self.ccvs_ID, data=[255, 0, 0, 255, 255, 255, 255, 255], is_extended_id=True) # use fake data, only commanding steering position for now
        self.ccvs_MSG = Frame(id_=self.ccvs_ID, data=[255, 0, 0, 255, 255, 255, 255, 255], dlc=8)

        # remote eps control (REC)
        self.rec_ID = int("18FF7325", 16)
        self.mode = 4 # 0=off, 2=torque assist, 4=position (speed ignored), 5=position with speed

        # Create default value to begin with
        # TO DO: need to modify to work with canlib
        init_angular_position = self.steering_to_angular_position(0)
        self.ang_WX_DATA, self.ang_YZ_DATA = self.angular_position_to_can(init_angular_position)
        # self.rec_MSG = can.Message(arbitration_id=self.rec_ID, data=[self.mode, 0, 255, 255, self.ang_YZ_DATA, self.ang_WX_DATA, 0, 0], is_extended_id=True)
        self.rec_MSG = Frame(id_=self.rec_ID, data=[self.mode, 0, 255, 255, 0, 0, 0, 0], dlc=8)

        # can intialization
        # TO DO: need to modify to work with canlib
        self.get_logger().info(f"Received Initializing CAN messaging to EPS... on topic {self.steering_cmd_topic}")
        self.ch = canlib.openChannel(
            channel=0,
            flags=canlib.Open.REQUIRE_EXTENDED,
            bitrate= canlib.Bitrate.BITRATE_250K,
        )
        # Set the CAN bus driver type to NORMAL.
        self.ch.setBusOutputControl(canlib.Driver.NORMAL)
        # Activate the CAN chip.
        self.ch.busOn()

        # self.bustype = 'socketcan'
        # self.channel = 'can0'
        # self.bus = can.interface.Bus(channel=self.channel, bustype=self.bustype)

        self.thrd_stop = False

        def thrd_ccvs_fcn():
            last_time = time.time()
            while not self.thrd_stop:
                curr_time = time.time()
                # send message at 5hz
                sleep_time = 0.2 - (curr_time - last_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                self.ch.write(self.ccvs_MSG)

                # Wait until the message is sent or at most 100 ms.
                self.ch.writeSync(timeout=100)
                last_time = curr_time
        
        def thrd_rec_fcn():
            last_time = time.time()
            while not self.thrd_stop:
                curr_time = time.time()
                # send message at 1000hz
                sleep_time = 0.001 - (curr_time - last_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                self.ch.write(self.rec_MSG)

                # Wait until the message is sent or at most 100 ms.
                self.ch.writeSync(timeout=100)
                last_time = curr_time

        self.thrd_ccvs = threading.Thread(target=thrd_ccvs_fcn)
        self.thrd_rec = threading.Thread(target=thrd_rec_fcn)
        self.thrd_ccvs.start()
        self.thrd_rec.start()



        # self.ccvs_TASK = self.bus.send_periodic(self.ccvs_MSG, 0.2) # send message at 5hz
        # self.rec_TASK = self.bus.send_periodic(self.rec_MSG, 0.001) # send message at 1000hz
        # self.get_logger().info(f"Received Ready for EPS power on! on topic {self.steering_cmd_topic}")

        # Set default position of the actuator
        self.set_actuator_position(self.steering_to_angular_position(0)) # Should check position of the actuator and set value that way


    def steering_cmd_callback(self, msg):
        """
        Callback for the steering_cmd topic.
        """
        self.get_logger().info(f"Received {msg} on topic {self.steering_cmd_topic}")


        steering = msg.value

        # calculate new CAN formatted data
        angular_position = self.steering_to_angular_position(steering)
        self.set_actuator_position(angular_position)

    def set_actuator_position(self, angular_position):
        """
        Actual setter for the angular position of the actuator

        Params:
            angular_position - position to set the actuator to (in degrees)
        """
        self.ang_WX_DATA, self.ang_YZ_DATA = self.angular_position_to_can(angular_position)

        # create new message
        # TO DO: change this for message sending
        # new_MSG = can.Message(arbitration_id=self.rec_ID, data=[self.mode, 0, 255, 255, self.ang_YZ_DATA, self.ang_WX_DATA, 0, 0], is_extended_id=True)
        self.rec_MSG = Frame(id_=self.rec_ID, data=[self.mode, 0, 255, 255, self.ang_YZ_DATA, self.ang_WX_DATA, 0, 0], dlc=8)
        # update active message data
        # self.rec_TASK.modify_data(new_MSG)

    def steering_to_angular_position(self, steering):
        """
        Converts steering input to an angular position in degrees

        Params:
            steering - [-1,1]; input from the controller
        """
        new_steering = scale_to_range(steering, -1., 1., self.min, self.max) # note that this only works if steering is correctly [-1,1] 
        new_steering = clamp(new_steering, self.min, self.max) # double check control value in case steering is not in [-1,1] (i.e. if steering is raw command line input)
        return new_steering + self.offset # adjusts to be at local 0

    def angular_position_to_can(self, angular_position):
        """
        Converts angular position given in degrees to CAN message format per Polaris spec.
        See documentation on drive.

        Params:
            angular_position - shaft position in degrees
        """
        converted = angular_position*10+32765
        converted = hex(int(converted)).lstrip('0x')
        WX = converted[0:2]
        WX_DATA = int(WX, 16)
        YZ = converted[2:4]
        YZ_DATA = int(YZ, 16)

        return WX_DATA, YZ_DATA

def main(args=None):
    rclpy.init(args=args)
    steering = SteeringActuation()
    rclpy.spin(steering)


if __name__ == '__main__':
    main()
