#
# MIT License
#
# Copyright (c) 2018-2022 Wisconsin Autonomous
#
# See https://wa.wisc.edu
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
import rclpy
from common_msgs.msg import ControlMsg
from rclpy.node import Node

from .Joystick import Joystick


class JoystickControllerNode(Node):
    def __init__(self):
        super().__init__("joystick_controller")

        self.joystick = Joystick()

        # Create publisher handles
        self.publisher_handles = {}
        self.publisher_handles["actuation_control"] = self.create_publisher(
            ControlMsg, "/control/actuation_control", 10
        )

        # Periodic publishing
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Basic method for publishing. Get rid of or modify this method,
        but please include a description of the method as is done here.
        """
        msg = ControlMsg()
        joystick_inputs = self.joystick.getJoystickValues()
        msg.throttle = ((joystick_inputs["throttle"] + 1) / 2) ** 2 / 2 * -1
        msg.steering = joystick_inputs["steering"]
        msg.braking = ((joystick_inputs["braking"] + 1) / 2) ** 2

        self.publisher_handles["actuation_control"].publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = JoystickControllerNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
