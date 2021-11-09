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
from rclpy.node import Node
from std_msgs.msg import String


class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit")

        # Create publisher handles
        self.publisher_handles = {}

        self.publisher_handles["actuation_control"] = self.create_publisher(
            String, "actuation_control", 10
        )

        # Create subscriber handles
        self.subscriber_handles = {}

        self.subscriber_handles["vehicle_state"] = self.create_subscription(
            String, "vehicle_state", self.vehicle_state_callback, 10
        )

        # Periodic publishing
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        """
        Basic method for publishing. Get rid of or modify this method,
        but please include a description of the method as is done here.
        """
        msg = String()
        msg.data = "Hello World: %d" % self.i

        self.publisher_handles["actuation_control"].publish(msg)

        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def vehicle_state_callback(self, msg):
        """
        Callback for the vehicle_state topic.
        """
        self.get_logger().info(f"Received {msg} on topic vehicle_state")


def main(args=None):
    rclpy.init(args=args)

    pure_pursuit = PurePursuit()

    rclpy.spin(pure_pursuit)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
