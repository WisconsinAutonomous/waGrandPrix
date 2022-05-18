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

    control_algorithm_arg = DeclareLaunchArgument(
        "control_algorithm", default_value=TextSubstitution(text="StanleyController")
    )

    sim_vehicle_state_topic_arg = DeclareLaunchArgument(
        "sim_vehicle_state_topic", default_value=TextSubstitution(text="/vehicle/state")
    )

    launch_description.add_action(control_algorithm_arg)
    launch_description.add_action(sim_vehicle_state_topic_arg)


    # mapped_track_topic_arg = DeclareLaunchArgument(
    #     "mapped_track_topic", default_value=TextSubstitution(text="/perception/mapped_track/mapped")
    # )
    # launch_description.add_action(mapped_track_topic_arg)

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
        executable='global_planning_node',
        name='global_planning_node',
        parameters=[{
            # "vehicle_state_topic": LaunchConfiguration("sim_vehicle_state_topic"),
        }],
        output="screen",
    )

    controller_node = Node(
        package='controls_py',
        namespace='controls',
        executable='controller_node',
        name='controller_node',
        parameters=[{
            "control_algorithm": LaunchConfiguration("control_algorithm"),
            # "vehicle_state_topic": LaunchConfiguration("sim_vehicle_state_topic"),
        }],
        output="screen",
    )

    # launch_description.add_action(forward_searcher_node)
    # launch_description.add_action(trajectory_tracker_node)
    launch_description.add_action(planning_node)
    launch_description.add_action(controller_node)

    return launch_description
