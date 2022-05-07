#!/usr/bin/env python3

__author = "Victor Freire"
__email__ = "freiremelgiz@wisc.edu"


from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution


def generate_launch_description():
    return LaunchDescription([

        # vehicle integration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('wagp_vehicle_integration_launch'),
                    'launch',
                    'vehicle_integration.launch.py'
                ])
            ])
        ),

        # perception
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('wagp_perception_launch'),
                    'launch',
                    'perception.launch.py'
                ])
            ])
        ),

        # SBG IMU/GPS
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sbg_driver'),
                    'launch',
                    'sbg_device_launch.py'
                ])
            ])
        ),

        # localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('wagp_localization_launch'),
                    'launch',
                    'localization.launch.py'
                ])
            ])
        ),

        # controls
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('wagp_controls_launch'),
                    'launch',
                    'controls.launch.py'
                ])
            ])
        )
    ])
