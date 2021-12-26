# Instantaneous detection of a track
# Output of the road_detector node
# Will be accumulated to map out the track

std_msgs/Header header

# List of 3D points that define the left and right boundaries of the track
# perceived at one timestep
geometry_msgs/Point[] left_points
geometry_msgs/Point[] right_points

