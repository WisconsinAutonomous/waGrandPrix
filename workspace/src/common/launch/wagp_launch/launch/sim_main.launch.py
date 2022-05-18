#!/usr/bin/env python3

__author = "Victor Freire"
__email__ = "freiremelgiz@wisc.edu"


from multiprocessing import Condition
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition


def generate_launch_description():
    ld = LaunchDescription()

    # record argument
    ld.add_action(DeclareLaunchArgument("record", default_value="False"))
    record = LaunchConfiguration("record")


    # WA Simulator-ROS bridge
    wa_simulator_ros_bridge = Node(
        package='wa_simulator_ros_bridge',
        namespace='sim',
        executable='bridge',
        name='bridges'
    )
    ld.add_action(wa_simulator_ros_bridge)

    # localization
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wagp_localization_launch'),
                'launch',
                'localization.launch.py'
            ])
        ]),
        launch_arguments={
            "fake_with_sim": 'True'
        }.items()
    )
    ld.add_action(localization_launch)

    # perception
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wagp_perception_launch'),
                'launch',
                'perception.launch.py'
            ])
        ]),
        launch_arguments={
            "fake_with_sim": 'True'
        }.items()
    )
    ld.add_action(perception_launch)

    # controls
    controls_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wagp_controls_launch'),
                'launch',
                'controls.launch.py'
            ])
        ]),
    )
    ld.add_action(controls_launch)

    # bag record
    bag_record_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wagp_launch'),
                'launch',
                'wagp_bag_record.launch.py'
            ])
        ]),
        condition = IfCondition(record)
    )
    ld.add_action(bag_record_launch)

    return ld