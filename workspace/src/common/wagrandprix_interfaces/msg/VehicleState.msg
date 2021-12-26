# Instantaneous state information of the ego vehicle
# Output by the vehicle_state_estimator node

# Position/orientation, velocity, and acceleration informatio
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Accel accel

# Easy access state values
float64 speed
float64 heading
