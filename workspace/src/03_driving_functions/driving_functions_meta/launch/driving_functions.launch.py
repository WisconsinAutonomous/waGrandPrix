"""
This is the launch file for driving_functions nodes

It will include and start up any driving_functions related launch files
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
    driving_functions_py_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('driving_functions_py'), 'launch'),
        '/driving_functions_py.launch.py'])
    )

    launch_description.add_action(driving_functions_py_launch)

    return launch_description

