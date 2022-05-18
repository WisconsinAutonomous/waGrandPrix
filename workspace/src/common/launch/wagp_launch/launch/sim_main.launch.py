#!/usr/bin/env python3

__author = "Victor Freire"
__email__ = "freiremelgiz@wisc.edu"


from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution


def generate_launch_description():
    return LaunchDescription([

        # WA Simulator-ROS bridge
        Node(
            package='wa_simulator_ros_bridge',
            namespace='sim',
            executable='bridge',
            name='bridges'
        ),

        # localization
        IncludeLaunchDescription(
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
        ),

        # perception
        IncludeLaunchDescription(
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
        ),

        # controls
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('wagp_controls_launch'),
                    'launch',
                    'controls.launch.py'
                ])
            ]),
        ),

        # bag record
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('wagp_launch'),
                    'launch',
                    'wagp_bag_record.launch.py'
                ])
            ])
        )
    ])
