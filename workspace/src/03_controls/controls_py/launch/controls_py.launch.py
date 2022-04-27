"""
This is the launch file for the controls_py package

It will create and start all the nodes specific to controls
"""

import rclpy

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

logger = rclpy.logging.get_logger('logger')

def generate_launch_description():
    logger.info("test")
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
    # forward_searcher_node = Node(
    #     package='controls_py',
    #     namespace='controls',
    #     executable='forward_searcher',
    #     name='forward_searcher',
    #     parameters=[{
    #         # "mapped_track_topic": LaunchConfiguration("mapped_track_topic"),
    #     }],
    #     output="screen",
    # )

    # trajectory_tracker_node = Node(
    #     package='controls_py',
    #     namespace='controls',
    #     executable='trajectory_tracker',
    #     name='trajectory_tracker',
    #     parameters=[{
    #     }],
    #     output="screen",
    # )

    planning_node = Node(
        package='controls_py',
        namespace='controls',
        executable='planning_node',
        name='planning_node',
        parameters=[{
        }],
        output="screen",
    )

    controller_node = Node(
        package='controls_py',
        namespace='controls',
        executable='controller_node',
        name='controller_node',
        parameters=[{
        }],
        output="screen",
    )

    # launch_description.add_action(forward_searcher_node)
    # launch_description.add_action(trajectory_tracker_node)
    launch_description.add_action(planning_node)
    launch_description.add_action(controller_node)

    return launch_description
