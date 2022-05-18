from xml.etree.ElementPath import get_parent_map
import wa_simulator as wa
import matplotlib.pyplot as plt

# Constant - 
NUM_POINTS = 20

class CenterlinePlanner():
    def __init__(self):
        self.track_left = []
        self.track_right = []
        # self.pos = None

    # Return the midpoint between the last given left and right points
    def get_waypoint(self):

        # midpoints = [] # list of midpoints along track

        # left = wa.WASplinePath(track_left, num_points=NUM_POINTS, is_closed=False)
        # right = wa.WASplinePath(track_right, num_points=NUM_POINTS, is_closed=False)
        # for i in NUM_POINTS:
        #     midpoint_x = (right.get_points()[i][0] + left.get_points()[i][0]) / 2
        #     midpoint_y = (right.get_points()[i][1] + left.get_points()[i][1]) / 2
        #     midpoints.append([midpoint_x, midpoint_y, 0])
        # return midpoints[-1]

        midpoint_x = (self.track_right[0][0] + self.track_left[0][0]) / 2
        midpoint_y = (self.track_right[0][1] + self.track_left[0][1]) / 2
        midpoint_z = (self.track_right[0][2] + self.track_left[0][2]) / 2
        return [midpoint_x, midpoint_y, midpoint_z]