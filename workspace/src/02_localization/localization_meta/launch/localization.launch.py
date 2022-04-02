"""
This is the launch file for localization nodes

It will include and start up any localization related launch files
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    launch_description = LaunchDescription()

    # ---------------
    # Launch Includes
    # ---------------
    localization_py_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('localization_py'), 'launch'),
        '/localization_py.launch.py'])
    )

    launch_description.add_action(localization_py_launch)

    return launch_description

