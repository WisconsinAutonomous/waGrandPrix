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
import transforms3d as tf
from common_msgs.msg import ObstacleMsg, ObstaclesMsg, RoadEdge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from wa_simulator_bridge.msg import WAVehicleStateMsg


class WASimStub(Node):
    """
    This reads in the path file and outputs the next bit of the edges of the
    track. Make sure that simulator and this are using the same path CSV file,
    or it won't make any sense what is output.
    """

    def __init__(self):
        super().__init__("perception_wasim_stub")

        # Create publisher handles
        self.publisher_handles = {}
        self.publisher_handles["road_edge"] = self.create_publisher(
            RoadEdge, "road_edge", 1
        )
        self.publisher_handles["obstacles"] = self.create_publisher(
            ObstaclesMsg, "obstacles", 1
        )

        # Create subscriber handles
        self.subscriber_handles = {}
        self.subscriber_handles["vehicle_state"] = self.create_subscription(
            WAVehicleStateMsg,
            "/wa_simulator/vehicle_state/go_kart",
            self.vehicle_state_callback,
            1,
        )
        self.subscriber_handles["op1"] = self.create_subscription(
            WAVehicleStateMsg,
            "/wa_simulator/vehicle_state/vehicle1",
            self.op1_callback,
            1,
        )
        self.subscriber_handles["op2"] = self.create_subscription(
            WAVehicleStateMsg,
            "/wa_simulator/vehicle_state/vehicle2",
            self.op2_callback,
            1,
        )

        self.op1_pos = [0.0, 0.0]
        self.op1_rot = 0.0

        self.op2_pos = [0.0, 0.0]
        self.op2_rot = 0.0

        # Read in the path file (lng, lat, el)
        self.path_points = np.loadtxt(
            open(
                "/home/ianruh/Dev/software-dev/ros2_workspace/src/perception/"
                "perception/purdue_path.csv",
                "rb",
            ),
            delimiter=",",
        )

        self.position = [0.0, 0.0]  # current position of the vehicle

        # Periodic publishing
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def get_closest_point(self, x, y):
        """
        Get the closest point on the path to the given point
        """
        min_index = 0
        min_dist = np.sqrt(
            (self.path_points[0, 0] - x) ** 2
            + (self.path_points[0, 1] - y) ** 2
        )
        for i in range(1, self.path_points.shape[0]):
            dist = np.sqrt(
                (self.path_points[i, 0] - x) ** 2
                + (self.path_points[i, 1] - y) ** 2
            )
            if dist < min_dist:
                min_dist = dist
                min_index = i
        return min_index

    def timer_callback(self):
        """
        Basic method for publishing. Get rid of or modify this method,
        but please include a description of the method as is done here.
        """
        closest_index = self.get_closest_point(
            self.position[0], self.position[1]
        )
        indexes = [
            closest_index,
            (closest_index + 1) % self.path_points.shape[0],
            (closest_index + 2) % self.path_points.shape[0],
            (closest_index + 3) % self.path_points.shape[0],
            (closest_index + 4) % self.path_points.shape[0],
            (closest_index + 5) % self.path_points.shape[0],
            (closest_index + 6) % self.path_points.shape[0],
            (closest_index + 7) % self.path_points.shape[0],
            (closest_index + 8) % self.path_points.shape[0],
            (closest_index + 9) % self.path_points.shape[0],
            (closest_index + 10) % self.path_points.shape[0],
            (closest_index + 11) % self.path_points.shape[0],
            (closest_index + 12) % self.path_points.shape[0],
            (closest_index + 13) % self.path_points.shape[0],
            (closest_index + 14) % self.path_points.shape[0],
            (closest_index + 15) % self.path_points.shape[0],
            (closest_index + 16) % self.path_points.shape[0],
            (closest_index + 17) % self.path_points.shape[0],
            (closest_index + 18) % self.path_points.shape[0],
            (closest_index + 19) % self.path_points.shape[0],
            (closest_index + 20) % self.path_points.shape[0],
        ]

        forward_vectors = []
        for i in range(0, len(indexes) - 1):
            forward_vectors.append(
                self.path_points[indexes[i + 1], 0:2]
                - self.path_points[indexes[i], 0:2]
            )

        left_pts = []
        right_pts = []

        for i in range(0, len(forward_vectors)):
            normal_forward = (
                forward_vectors[i] / np.linalg.norm(forward_vectors[i])
            ) * 5
            left_vector = self.path_points[
                (closest_index + i) % self.path_points.shape[0]
            ][0:2] + np.array([-1 * normal_forward[1], normal_forward[0]])
            right_vector = self.path_points[
                (closest_index + i) % self.path_points.shape[0]
            ][0:2] + np.array([normal_forward[1], -1 * normal_forward[0]])

            left_msg = Vector3()
            left_msg.x = left_vector[0]
            left_msg.y = left_vector[1]

            right_msg = Vector3()
            right_msg.x = right_vector[0]
            right_msg.y = right_vector[1]

            left_pts.append(left_msg)
            right_pts.append(right_msg)

        edge_msg = RoadEdge()
        edge_msg.left = left_pts
        edge_msg.right = right_pts

        self.publisher_handles["road_edge"].publish(edge_msg)

        obs_msg = ObstaclesMsg()

        op1_msg = ObstacleMsg()
        op1_center = Vector3()
        op1_center.x = self.op1_pos[0]
        op1_center.y = self.op1_pos[1]
        op1_center.z = 0.0
        op1_msg.center = op1_center
        op1_msg.width = 2.0
        op1_msg.height = 4.0
        op1_msg.heading = self.op1_rot + 1.5708
        obs_msg.obstacles.append(op1_msg)

        op2_msg = ObstacleMsg()
        op2_center = Vector3()
        op2_center.x = self.op2_pos[0]
        op2_center.y = self.op2_pos[1]
        op2_center.z = 0.0
        op2_msg.center = op2_center
        op2_msg.width = 2.0
        op2_msg.height = 4.0
        op2_msg.heading = self.op2_rot + 1.5708
        obs_msg.obstacles.append(op2_msg)

        self.publisher_handles["obstacles"].publish(obs_msg)

    def vehicle_state_callback(self, msg):
        """
        Callback for the vehicle_state topic.
        """
        self.position = [msg.position.x, msg.position.y]

    def op1_callback(self, msg):
        """
        Callback for the vehicle_state topic.
        """
        self.op1_pos = [msg.position.x, msg.position.y]
        quat = np.array(
            [msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w]
        )
        euler = tf.euler.quat2euler(quat)
        self.op1_rot = euler[0]  # yaw

    def op2_callback(self, msg):
        """
        Callback for the vehicle_state topic.
        """
        self.op2_pos = [msg.position.x, msg.position.y]
        quat = np.array(
            [msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w]
        )
        euler = tf.euler.quat2euler(quat)
        self.op2_rot = euler[0]  # yaw


def main(args=None):
    rclpy.init(args=args)

    node = WASimStub()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
