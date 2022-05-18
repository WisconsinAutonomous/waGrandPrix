"""
This is the launch file for the localization_py package

It will create and start all the nodes specific to localization
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
    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic", default_value=TextSubstitution(text="/sensor/imu/data")
    )

    gps_topic_arg = DeclareLaunchArgument(
        "gps_topic", default_value=TextSubstitution(text="/sensor/gps/data")
    )

    wheel_encoder_topic_arg = DeclareLaunchArgument(
        "wheel_encoder_topic", default_value=TextSubstitution(text="/sensor/wheel_encoder/data")
    )

    steering_feedback_topic_arg = DeclareLaunchArgument(
        "steering_feedback_topic", default_value=TextSubstitution(text="/sensor/steering_feedback/data")
    )

    detected_track_topic_arg = DeclareLaunchArgument(
        "detected_track_topic", default_value=TextSubstitution(text="/perception/detected_track/detected")
    )

    vehicle_state_topic_arg = DeclareLaunchArgument(
        "vehicle_state_topic", default_value=TextSubstitution(text="vehicle/state")
    )

    fake_with_sim_arg = DeclareLaunchArgument(
        "fake_with_sim", default_value=TextSubstitution(text="False")
    )

    sim_vehicle_state_topic_arg = DeclareLaunchArgument(
        "sim_vehicle_state_topic", default_value=TextSubstitution(text="/sim/vehicle/state")
    )

    sim_track_topic_arg = DeclareLaunchArgument(
        "sim_track_topic", default_value=TextSubstitution(text="/sim/track/mapped")
    )

    launch_description.add_action(imu_topic_arg)
    launch_description.add_action(gps_topic_arg)
    launch_description.add_action(wheel_encoder_topic_arg)
    launch_description.add_action(steering_feedback_topic_arg)
    launch_description.add_action(vehicle_state_topic_arg)
    launch_description.add_action(detected_track_topic_arg)
    launch_description.add_action(fake_with_sim_arg)
    launch_description.add_action(sim_vehicle_state_topic_arg)
    launch_description.add_action(sim_track_topic_arg)

    # -----
    # Nodes
    # -----
    vehicle_state_estimator_node = Node(
        package='localization_py',
        namespace='localization',
        executable='vehicle_state_estimator',
        name='vehicle_state_estimator',
        parameters=[{
            # "imu_topic": LaunchConfiguration("imu_topic"),
            # "gps_topic": LaunchConfiguration("gps_topic"),
            "wheel_encoder_topic": LaunchConfiguration("wheel_encoder_topic"),
            "steering_feedback_topic": LaunchConfiguration("steering_feedback_topic"),
            "fake_with_sim": LaunchConfiguration("fake_with_sim"),
            "sim_vehicle_state_topic": LaunchConfiguration("sim_vehicle_state_topic"),
        }],
        output="screen",
    )

    track_mapper_node = Node(
        package='localization_py',
        namespace='localization',
        executable='track_mapper',
        name='track_mapper',
        parameters=[{
            "fake_with_sim": LaunchConfiguration("fake_with_sim"),
        }],
        output="screen",
    )

    launch_description.add_action(vehicle_state_estimator_node)
    launch_description.add_action(track_mapper_node)

    return launch_description