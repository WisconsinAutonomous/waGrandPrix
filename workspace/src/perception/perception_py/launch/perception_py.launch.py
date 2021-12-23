"""
This is the launch file for the perception_py package

It will create and start all the nodes specific to perception
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
    lidar_topic_arg = DeclareLaunchArgument(
        "lidar_topic", default_value=TextSubstitution(text="/sensor/lidar/pointcloud")
    )

    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic", default_value=TextSubstitution(text="/sensor/camera/front/image")
    )

    fake_with_sim_arg = DeclareLaunchArgument(
        "fake_with_sim", default_value=TextSubstitution(text="False")
    )

    launch_description.add_action(lidar_topic_arg)
    launch_description.add_action(camera_topic_arg)
    launch_description.add_action(fake_with_sim_arg)

    # -----
    # Nodes
    # -----
    track_detector_node = Node(
        package='perception_py',
        namespace='perception',
        executable='track_detector',
        name='track_detector',
        parameters=[{
            "lidar_topic": LaunchConfiguration("lidar_topic"),
            "camera_topic": LaunchConfiguration("camera_topic"),
            "fake_with_sim": LaunchConfiguration("fake_with_sim"),
        }],
        output="screen",
    )

    launch_description.add_action(track_detector_node)

    return launch_description
