from xml.etree.ElementPath import get_parent_map
import wa_simulator as wa
import matplotlib.pyplot as plt

# Constant - 
NUM_POINTS = 20

class CenterlinePlanner():
    def __init__(self):
        self.track = None
        self.pos = None

    def plan_path(self):
        left = wa.WASplinePath([track.left_visible_points], num_points=NUM_POINTS, is_closed=False)
        right = wa.WASplinePath([track.right_visible_points], num_points=NUM_POINTS, is_closed=False)
        midpoints = [] # list of midpoints along track

        for i in NUM_POINTS:
            midpoint_x = (right.get_points()[i][0] + left.get_points()[i][0]) / 2
            midpoint_y = (right.get_points()[i][1] + left.get_points()[i][1]) / 2
            midpoints.append([midpoint_x, midpoint_y, 0])
        return midpoints

    def get_waypoint(self, horizon):
        path = self.plan_path()

        dist = cdist(self.path, [self.pos])
        idx, = np.argmin(dist, axis=0)

        pos = wa.WAVector([path[idx][0], path[idx][1], path[idx][2]])
        return pos


# Will call the main function when 'python custom_controller_demo.py' is run
if __name__ == "__main__":
    # Load track data points from a csv file
    wa.set_wa_data_directory('../../../../../sim/wasim/data')
    filename = wa.get_wa_data_file("paths/sample_medium_loop.csv")
    waypoints = wa.load_waypoints_from_csv(filename, delimiter=",")

    # Create the path
    path = wa.WASplinePath(waypoints, num_points=1000, is_closed=True)
    # Create the track with a constant width
    track = wa.create_constant_width_track(path, width=10)

    cp = CenterlinePlanner()
    path = cp.get_path(track)

    # Plot track boundaries with our new path
    plt.axis('equal')
    path.plot("red", show=False)
    track.left.plot("black", show=False)
    track.right.plot("black")