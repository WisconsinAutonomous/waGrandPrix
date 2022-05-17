"""
This is the launch file for the vehicle_integration_py package
"""

import rclpy

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

logger = rclpy.logging.get_logger('logger')

def generate_launch_description():
    launch_description = LaunchDescription()

    # ----------------
    # Launch Arguments
    # ----------------
    # mapped_track_topic_arg = DeclareLaunchArgument(
    #     "mapped_track_topic", default_value=TextSubstitution(text="/perception/mapped_track/mapped")
    # )

    # -----
    # Nodes
    # -----

    power_relay = Node(
        package='vehicle_integration_py',
        namespace='vehicle_integration',
        executable='power_relay',
        name='power_relay',
        parameters=[{
        }],
        output="screen",
    )

    power_relay_publisher = Node(
        package='vehicle_integration_py',
        namespace='vehicle_integration',
        executable='power_relay_publisher',
        name='power_relay_publisher',
        parameters=[{
        }],
        output="screen",
    )

    brake_actuation = Node(
        package='vehicle_integration_py',
        namespace='vehicle_integration',
        executable='brake_actuation',
        name='brake_actuation',
        parameters=[{
        }],
        output="screen",
    )

    steering_actuation = Node(
        package='vehicle_integration_py',
        namespace='vehicle_integration',
        executable='steering_actuation',
        name='steering_actuation',
        parameters=[{
        }],
        output="screen",
    )

    throttle_actuation = Node(
        package='vehicle_integration_py',
        namespace='vehicle_integration',
        executable='throttle_actuation',
        name='throttle_actuation',
        parameters=[{
        }],
        output="screen",
    )

    launch_description.add_action(power_relay)
    launch_description.add_action(power_relay_publisher)
    launch_description.add_action(brake_actuation)
    launch_description.add_action(steering_actuation)
    launch_description.add_action(throttle_actuation) 

    return launch_description
