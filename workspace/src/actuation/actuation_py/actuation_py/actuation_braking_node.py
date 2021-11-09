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


class ActuationBrakingNode(Node):
    def __init__(self):
        super().__init__("actuation_braking_node")

        # Create subscriber handles
        self.subscriber_handles = {}

        self.subscriber_handles[
            "/control/actuation_control"
        ] = self.create_subscription(
            String,
            "/control/actuation_control",
            self.control_actuation_control_callback,
            10,
        )

    def control_actuation_control_callback(self, msg):
        """
        Callback for the /control/actuation_control topic.
        """
        self.get_logger().info(
            f"Received {msg} on topic /control/actuation_control"
        )


def main(args=None):
    rclpy.init(args=args)

    actuation_braking_node = ActuationBrakingNode()

    rclpy.spin(actuation_braking_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
