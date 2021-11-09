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
from common_msgs.msg import ObstaclesMsg, RoadEdge, VehStateMsg
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import Image  # for the visualization only

from .occupancy_graph_utils import OccupancyGraph, PathSplineDense, RectObstacle


def fancy_legend(
    ax, handles=[], labels=[], frame_transparency=0.5, location="best"
):
    if len(handles) == 0 or len(labels) == 0:
        handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(
        by_label.values(),
        by_label.keys(),
        fancybox=True,
        framealpha=frame_transparency,
        loc=location,
    )


def greedy_node_trajectory(start_node, search_depth):
    node_traj = [start_node]
    curr_node = start_node
    for i in range(search_depth):
        best_cost = 100  # must be >= max of all possible node costs
        if len(curr_node.children) == 0:
            print(
                f"[WARN] Search depth {search_depth} exceeded available"
                "children."
            )
            print("Returning node list so far.")
            return node_traj
        for child in curr_node.children:
            if child.cost < best_cost:
                best_cost = child.cost
                next_node = child
        node_traj.append(next_node)
        curr_node = next_node

    return node_traj


class OccupancyGraphROSNode(Node):
    def __init__(self):
        super().__init__("occ_graph")

        # Create publisher handles
        self.publisher_handles = {}
        self.publisher_handles["occ_graph_vis"] = self.create_publisher(
            Image, "occ_graph_vis", 1
        )

        # Create subscriber handles
        self.subscriber_handles = {}
        self.subscriber_handles["road_edge"] = self.create_subscription(
            RoadEdge, "/road_edge", self.road_edge_callback, 10
        )
        self.subscriber_handles["obstacles"] = self.create_subscription(
            ObstaclesMsg, "/obstacles", self.obstacles_callback, 10
        )
        self.subscriber_handles["vehicle_state"] = self.create_subscription(
            VehStateMsg, "vehicle_state", self.vehicle_state_callback, 1
        )

        self.road_left = np.array([0.0, 0.0])
        self.road_right = np.array([0.0, 0.0])

        self.obstacles = []
        self.bridge = CvBridge()

        self.pos = [0.0, 0.0]  # Initial vehicle position

        # Periodic publishing
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        """
        Basic method for publishing. Get rid of or modify this method,
        but please include a description of the method as is done here.
        """

        # Create the left and right boundaries
        left_path = PathSplineDense(
            self.road_left, min_spline_spacing=0.4, make_dense=False
        )
        right_path = PathSplineDense(
            self.road_right, min_spline_spacing=0.4, make_dense=False
        )

        # Initialize graph
        num_lon = 25  # Number of nodes along longitudinal direction
        num_lat = 15  # Number of nodes along lateral direction
        # Number of lateral nodes to connect with in next longitudinal layer,
        # including the node with the same lateral index, must be >= 0
        num_child_lr = 2
        init_cost_mode = (
            "Centerline"  # Mode for initializing node costs in empty graph
        )
        occupancy_graph = OccupancyGraph(
            left_path,
            right_path,
            num_lat,
            num_lon,
            num_child_lr,
            init_cost_mode="Centerline",
        )

        # Add some obstacles
        # To update state and nodes:
        # 1. Use obstacle.update_state() function for update position and reset
        #    node costs
        # 2. Use occupancy_graph.place_rect_obstacle_as_rect() to update node
        #    costs
        # - Make sure to then re-run the local search to account for the new
        #   obstacle(s) position(s)
        self.rect_obstacles = []
        for obstacle in self.obstacles:
            obs_rect = RectObstacle(
                x=obstacle.center.x,
                y=obstacle.center.y,
                length=obstacle.height,
                width=obstacle.width,
                heading=obstacle.heading,
                padding=0.1,
            )
            self.rect_obstacles.append(obs_rect)
            occupancy_graph.place_rect_obstacle_as_rect(obs_rect)

        start_s_i = 0
        start_d_i = num_lat - 1
        search_depth = num_lon - 1
        start_node = occupancy_graph.df_nodes["node"][
            occupancy_graph.df_nodes["s_i"] == start_s_i
        ][occupancy_graph.df_nodes["d_i"] == start_d_i].values[0]
        start_dist = np.sqrt(
            (start_node.x - self.pos[0]) ** 2
            + (start_node.y - self.pos[1]) ** 2
        )

        # set the start node to be the one closest to the current position
        # TODO: Idk how pandas works, we shouldn't have to search the entire
        #       data frame I think
        for i, row in occupancy_graph.df_nodes.iterrows():
            row_dist = np.sqrt(
                (row["node"].x - self.pos[0]) ** 2
                + (row["node"].y - self.pos[1]) ** 2
            )
            if row_dist < start_dist:
                start_node = row["node"]
                start_dist = row_dist

        node_traj = greedy_node_trajectory(start_node, search_depth)

        # Plotting (shouldn't really happen here, but ah well for now)
        fig, ax = plt.subplots(figsize=(11, 9))
        fig.suptitle("Occupancy Graph", fontsize=20)
        ax.set_title(
            f"longitudinal nodes: {num_lon}, lateral nodes: {num_lat}, node "
            f"children: {num_child_lr*2+1}\nCost initialization mode: "
            f"'{init_cost_mode}'",
            fontsize=12,
        )
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        fig.subplots_adjust(top=0.9)
        ax.set_aspect("equal")

        occupancy_graph.plot(ax, edges=False)
        for rect_obstacle in self.rect_obstacles:
            if (
                np.sqrt(
                    (rect_obstacle.x - self.pos[0]) ** 2
                    + (rect_obstacle.y - self.pos[1]) ** 2
                )
                < 40
            ):
                rect_obstacle.plot(
                    ax, padding=True, body_corners=False, padding_corners=False
                )

        ax.plot(
            [node.x for node in node_traj],
            [node.y for node in node_traj],
            marker="*",
            markersize=12,
            color="magenta",
            label="Node Trajectory",
        )

        ax.plot(
            self.road_left[:, 0],
            self.road_left[:, 1],
            marker="o",
            linestyle="",
            color="blue",
            zorder=2,
            label="Left Waypoints",
        )
        ax.plot(
            left_path.x,
            left_path.y,
            linestyle="--",
            color="blue",
            zorder=2,
            label="Left Spline",
        )

        ax.plot(
            self.road_right[:, 0],
            self.road_right[:, 1],
            marker="o",
            linestyle="",
            color="purple",
            zorder=2,
            label="Right Waypoints",
        )
        ax.plot(
            right_path.x,
            right_path.y,
            linestyle="--",
            color="purple",
            zorder=2,
            label="Right Spline",
        )

        fancy_legend(ax)

        fig.canvas.draw()

        data = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep="")
        data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        plt.close(fig)

        self.publisher_handles["occ_graph_vis"].publish(
            self.bridge.cv2_to_imgmsg(data, "rgb8")
        )

    def road_edge_callback(self, msg):
        """
        Callback for the road edge messages
        """
        leftside = []
        for v in msg.left:
            leftside.append(np.array([v.x, v.y]))
        self.road_left = np.array(leftside)

        rightside = []
        for v in msg.right:
            rightside.append(np.array([v.x, v.y]))
        self.road_right = np.array(rightside)

    def obstacles_callback(self, msg):
        """
        Callback for the obstacles messages
        """
        self.obstacles = msg.obstacles

    def vehicle_state_callback(self, msg):
        self.pos = [msg.x, msg.y]


def main(args=None):
    rclpy.init(args=args)

    node = OccupancyGraphROSNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
