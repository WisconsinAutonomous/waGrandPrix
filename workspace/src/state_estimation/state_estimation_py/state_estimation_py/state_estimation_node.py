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
import numpy as np
import rclpy
from common_msgs.msg import VehStateMsg
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class StateEstimation(Node):
    def __init__(self):
        super().__init__("perception_wasim_stub")

        # Create publisher handles
        self.publisher_handles = {}
        self.publisher_handles["vehicle_state"] = self.create_publisher(
            VehStateMsg, "vehicle_state", 1
        )

        # Create subscriber handles
        self.subscriber_handles = {}
        self.subscriber_handles["vehicle_state"] = self.create_subscription(
            NavSatFix, "/imu/nav_sat_fix", self.navsat_callback, 1
        )

        self.reference_pt = [
            -8.694445754000000193e01,
            4.043767351000000332e01,
            1.732609999999999957e02,
        ]

    def gps_to_cartesian(self, coords, ref):
        """Convert a gps coordinate to cartesian given some reference

        Args:
          coords: The coordinate to convert
          ref: The "origin" or reference point

        Returns:
          ndarray: The x, y, z point in cartesian
        """
        lat = coords[0]
        lon = coords[1]
        alt = coords[2]

        earth_rad = 6371000.0  # meters

        x = ((lon - ref[0]) * np.pi / 180.0) * (
            earth_rad * np.cos(lat * np.pi / 180.0)
        )  # noqa
        y = ((lat - ref[1]) * np.pi / 180.0) * earth_rad
        z = alt - ref[2]

        return np.array([x, y, z])

    def navsat_callback(self, msg):
        """
        Callback for the vehicle_state topic.
        """
        lat = msg.latitude
        lng = msg.longitude
        alt = msg.altitude
        new_msg = VehStateMsg()
        pos = self.gps_to_cartesian([lat, lng, alt], self.reference_pt)
        new_msg.x = pos[0]
        new_msg.y = pos[1]

        self.publisher_handles["vehicle_state"].publish(new_msg)


def main(args=None):
    rclpy.init(args=args)

    node = StateEstimation()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
