from xml.etree.ElementPath import get_parent_map
import wa_simulator as wa
import matplotlib.pyplot as plt

class TestingPlanner():
    def __init__(self):
        self.track_left = []
        self.track_right = []
        self.pos = None

    # Return the midpoint between the last given left and right points
    def get_waypoint(self):

        # for i in range(len(self.track_left)):
        #     if self.track_left[i][1] < self.pos.y:
        #         midpoint_x = (self.track_right[i][0] + self.track_left[i][0]) / 2
        #         midpoint_y = (self.track_right[i][1] + self.track_left[i][1]) / 2
        #         midpoint_z = (self.track_right[i][2] + self.track_left[i][2]) / 2
        #         return [midpoint_x, midpoint_y, midpoint_z]
        return [42.968643333274024, -89.50509922551976, 0.0]