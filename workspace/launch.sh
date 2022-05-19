#!/bin/bash

ros2 launch wagp_vehicle_integration_launch vehicle_integration.launch.py
sleep 5
ros2 launch wagp_launch main.launch.py 