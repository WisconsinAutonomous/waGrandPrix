# Represents a waypoint on a track

# Absolute GPS coordinate
GPSCoordinate coordinate

# The 3D point that defines this waypoint
# In reference to the start line of the track
Point point

# distance to left edge of the track
float64 left_width

# distance to right edge of the track
float64 right_width

# Index of the point in the centerline list
uint32[] idx
