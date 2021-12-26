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
import math
import warnings

import matplotlib.path as mplPath
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.transforms import Affine2D
from scipy.interpolate import splev, splprep


class PathSplineDense:
    def __init__(
        self, points, min_spline_spacing=0.5, make_dense=True, num_fill=100
    ):
        """
        Parameters
        ----------
            points : 2D array of points as [[x1, y1], ..., [xn, yn]], should be
                passed as shape=(n,2)
                The waypoints describing the path
            min_spline_spacing : float
                The minimum desired spacing between waypoints for creating the
                spline
            make_dense : bool
                Flag to make the points dense before creating the spline
            num_fill : int
                Number of points to use for filling the spline. Only matters
                for plotting (can sample exact parameterized points with
                sample_pt)
        """

        if make_dense:
            points = self.increase_point_resolution(points, min_spline_spacing)

        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", category=RuntimeWarning)
            tck, u = splprep(points.T, s=0.0, per=False)

        u_new = np.linspace(u.min(), u.max(), num_fill)

        self.tck = tck
        self.x, self.y = splev(u_new, self.tck, der=0)

    def euclidean_distance(self, p1, p2):
        euclidean_dist = ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5
        return euclidean_dist

    def increase_point_resolution(self, sparse_path, min_spline_spacing):
        # evenly space points along each segment of sparse_path to maintain a
        # minumum spacing distance between points
        dense_path = np.array(
            sparse_path[0]
        )  # initialize with first point of sparse_path
        for i, _ in enumerate(sparse_path[:-1]):
            segment_length = self.euclidean_distance(
                sparse_path[i], sparse_path[i + 1]
            )  # distance of current segment
            num_new_points = math.ceil(segment_length / min_spline_spacing)
            # find number of points in segment to satisfy spacing, using
            # ceiling function to round and for min=1
            # point (for same as original path)
            new_points = np.linspace(
                sparse_path[i], sparse_path[i + 1], num_new_points + 1
            )[1:]
            # create more points in segment, but don't use the first point
            # since it's a repeat of sparse_path
            dense_path = np.vstack(
                (dense_path, new_points)
            )  # append to current path

        return dense_path

    def sample_pt(self, u):
        return splev(u, self.tck, der=0)


class RectObstacle:
    def __init__(self, x, y, length, width, heading, padding=0):
        self.x = x
        self.y = y
        self.length = length
        self.width = width
        # in radians, following RH rule (i.e. +heading is counterclockwise
        # rotation as viewed from top-down birdseye view)
        self.heading = heading
        self.padding = padding  # buffer from each edge to also avoid
        self.update_corners()
        self.nodes = []

    def plot(
        self, ax=None, padding=False, body_corners=False, padding_corners=False
    ):
        if ax is None:
            fig, ax = plt.subplots()

        ax.plot(self.x, self.y, color="red", marker="o", markersize=10)
        rect_obs = plt.Rectangle(
            (self.x - self.width / 2, self.y - self.length / 2),
            width=self.width,
            height=self.length,
            color="red",
            alpha=0.5,
            transform=Affine2D().rotate_around(self.x, self.y, self.heading)
            + ax.transData,
            zorder=4,
            label="Obstacle",
        )
        ax.add_patch(rect_obs)

        if padding:
            rect_pad = plt.Rectangle(
                (
                    self.x - self.width / 2 - self.padding,
                    self.y - self.length / 2 - self.padding,
                ),
                width=self.width + 2 * self.padding,
                height=self.length + 2 * self.padding,
                color="orange",
                alpha=0.4,
                transform=Affine2D().rotate_around(self.x, self.y, self.heading)
                + ax.transData,
                zorder=3,
                label="Obstacle Padding",
            )
            ax.add_patch(rect_pad)

        if body_corners:
            ax.plot(
                [self.fl[0], self.fr[0], self.rl[0], self.rr[0]],
                [self.fl[1], self.fr[1], self.rl[1], self.rr[1]],
                marker="o",
                linestyle="",
                color="black",
            )
            ax.annotate("fr", (self.fr[0], self.fr[1]))
            ax.annotate("fl", (self.fl[0], self.fl[1]))
            ax.annotate("rr", (self.rr[0], self.rr[1]))
            ax.annotate("rl", (self.rl[0], self.rl[1]))

        if padding_corners:
            ax.plot(
                [self.fl_p[0], self.fr_p[0], self.rl_p[0], self.rr_p[0]],
                [self.fl_p[1], self.fr_p[1], self.rl_p[1], self.rr_p[1]],
                marker="o",
                linestyle="",
                color="black",
            )
            ax.annotate("fr_p", (self.fr_p[0], self.fr_p[1]))
            ax.annotate("fl_p", (self.fl_p[0], self.fl_p[1]))
            ax.annotate("rr_p", (self.rr_p[0], self.rr_p[1]))
            ax.annotate("rl_p", (self.rl_p[0], self.rl_p[1]))

    def update_corners(self):
        self.fr, self.fl, self.rr, self.rl = self.get_rect_corners(
            self.length, self.width
        )
        self.fr_p, self.fl_p, self.rr_p, self.rl_p = self.get_rect_corners(
            self.length + 2 * self.padding, self.width + 2 * self.padding
        )

    def get_rect_corners(self, length, width):
        v_lon = {
            "x": -math.sin(self.heading) * length / 2,
            "y": math.cos(self.heading) * length / 2,
        }
        v_lat = {
            "x": -math.sin((self.heading - math.pi / 2)) * width / 2,
            "y": math.cos((self.heading - math.pi / 2)) * width / 2,
        }

        fr = [
            self.x + v_lat["x"] + v_lon["x"],
            self.y + v_lat["y"] + v_lon["y"],
        ]
        fl = [
            self.x - v_lat["x"] + v_lon["x"],
            self.y - v_lat["y"] + v_lon["y"],
        ]
        rr = [
            self.x + v_lat["x"] - v_lon["x"],
            self.y + v_lat["y"] - v_lon["y"],
        ]
        rl = [
            self.x - v_lat["x"] - v_lon["x"],
            self.y - v_lat["y"] - v_lon["y"],
        ]

        return fr, fl, rr, rl

    def update_state(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading = heading
        for node in self.nodes:
            node.cost = node.init_cost
        self.nodes = []
        self.update_corners()


class OccupancyNode:
    """
    Node in the drivable space.
    """

    def __init__(self, x, y, s, s_i, d, d_i, children=[], cost=0):
        self.x = x
        self.y = y
        self.cost = cost
        # original cost to return to after obstacles, etc., are removed
        self.init_cost = cost
        self.children = children  # list of OccupancyNode objects
        self.s = s
        self.s_i = s_i
        self.d = d
        self.d_i = d_i


class OccupancyGraph:
    """
    Graph covering drivable space. Nodes are given a cost associated with their
    drivability. Nodes contained in a Pandas DataFrame, indexed by x,y values
    as stored in node.x and node.y
    """

    def __init__(
        self,
        left_path,
        right_path,
        num_lat,
        num_lon,
        num_child_lr,
        init_cost_mode="Centerline",
    ):
        self.num_lat = num_lat
        self.num_lon = num_lon
        # number of children nodes, centered by lateral index, must be odd
        self.num_child_lr = num_child_lr
        self.df_nodes = self.generate_nodes(left_path, right_path)
        self.generate_edges()
        self.initialize_costs(init_cost_mode)

    def generate_nodes(self, left_path, right_path):
        nodes = []
        for s_i, s in enumerate(np.linspace(0, 1, self.num_lon)):
            # Get the segment between the two splines and add points
            segment = np.array(
                [left_path.sample_pt(s), right_path.sample_pt(s)]
            )
            dense_segment = self.expand_n_points(segment, self.num_lat)

            # Create the nodes
            d_vals = np.linspace(-1, 1, dense_segment.shape[0])
            for d_i, point in enumerate(dense_segment):
                new_node = OccupancyNode(
                    x=point[0], y=point[1], s=s, s_i=s_i, d=d_vals[d_i], d_i=d_i
                )
                nodes.append(
                    {
                        "x": new_node.x,
                        "y": new_node.y,
                        "s_i": s_i,
                        "d_i": d_i,
                        "node": new_node,
                    }
                )

        return pd.DataFrame(nodes)

    def generate_edges(self):
        for i, row in self.df_nodes.iterrows():
            row["node"].children = self.df_nodes["node"][
                self.df_nodes["s_i"] == row["node"].s_i + 1
            ][self.df_nodes["d_i"] >= row["node"].d_i - self.num_child_lr][
                self.df_nodes["d_i"] <= row["node"].d_i + self.num_child_lr
            ]

    def initialize_costs(self, mode="Centerline"):
        modes = ["Centerline"]
        if mode not in modes:
            print(f"Given mode '{mode}' not in available modes: {modes}.")
            print("Initializing nodes with default 'Centerline' mode.")
            mode = "Centerline"
        if mode == "Centerline":
            for i, row in self.df_nodes.iterrows():
                row["node"].cost = abs(row["node"].d)
                row["node"].init_cost = row["node"].cost

    def expand_n_points(self, sparse_path, num_new_points):
        # evenly space points along each segment of sparse_path to maintain a
        # minumum spacing distance between points
        dense_path = np.array(
            sparse_path[0]
        )  # initialize with first point of sparse_path
        for i, _ in enumerate(sparse_path[:-1]):
            new_points = np.linspace(
                sparse_path[i], sparse_path[i + 1], num_new_points
            )[1:]
            # create more points in segment, but don't use the first point
            # since it's a repeat of sparse_path
            dense_path = np.vstack(
                (dense_path, new_points)
            )  # append to current path

        return dense_path

    def get_nodes_in_region(self, x_lower, x_upper, y_lower, y_upper):
        return self.df_nodes["node"][self.df_nodes["x"] > x_lower][
            self.df_nodes["x"] < x_upper
        ][self.df_nodes["y"] > y_lower][self.df_nodes["y"] < y_upper]

    def place_rect_obstacle_as_circle(self, rect_obstacle):
        r_obs = max(rect_obstacle.length / 2, rect_obstacle.width / 2)
        obstacle_nodes = self.df_nodes["node"][
            (
                (self.df_nodes["x"] - rect_obstacle.x) ** 2
                + (self.df_nodes["y"] - rect_obstacle.y) ** 2
            )
            <= r_obs ** 2
        ]
        for node in obstacle_nodes:
            node.cost = 1

    def place_rect_obstacle_as_rect(
        self, rect_obstacle, obs_cost=1, use_padding=True, pad_cost=0.8
    ):
        rect_obs = mplPath.Path(
            np.array(
                [
                    rect_obstacle.fl,
                    rect_obstacle.fr,
                    rect_obstacle.rr,
                    rect_obstacle.rl,
                ]
            )
        )
        rect_pad = mplPath.Path(
            np.array(
                [
                    rect_obstacle.fl_p,
                    rect_obstacle.fr_p,
                    rect_obstacle.rr_p,
                    rect_obstacle.rl_p,
                ]
            )
        )

        for i, row in self.df_nodes.iterrows():
            if rect_obs.contains_point([row["node"].x, row["node"].y]):
                row["node"].cost = obs_cost
                rect_obstacle.nodes.append(row["node"])
            elif use_padding and rect_pad.contains_point(
                [row["node"].x, row["node"].y]
            ):
                row["node"].cost = max(row["node"].cost, pad_cost)
                rect_obstacle.nodes.append(row["node"])

    def plot(self, ax=None, edges=False):
        if ax is None:
            fig, ax = plt.subplots()
        df_plot = self.df_nodes.loc[:, ("x", "y")]
        costs = []

        for i, row in self.df_nodes.iterrows():
            costs.append(row["node"].cost)

            if edges:
                for j, child in enumerate(row["node"].children):
                    delta_d_i = row["node"].d_i - child.d_i
                    if delta_d_i == 0:
                        scale_alpha = 1
                    else:
                        scale_alpha = 1 / (abs(delta_d_i) + 1)
                    ax.plot(
                        [row["node"].x, child.x],
                        [row["node"].y, child.y],
                        color="black",
                        alpha=scale_alpha,
                        zorder=0,
                        label="Edge",
                    )
        df_plot["costs"] = costs
        scatter = ax.scatter(
            df_plot["x"],
            df_plot["y"],
            c=df_plot["costs"],
            cmap="RdYlGn_r",
            s=20,
            zorder=1,
        )
        cbar = plt.colorbar(scatter)
        cbar.set_label("Node Cost")
