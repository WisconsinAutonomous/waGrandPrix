# ROS related imports
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    ld = LaunchDescription()

    # TODO: Add support for specifying what topics to record

    # --------
    # Commands
    # --------

    ros2_bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        output='screen',
    )
    ld.add_action(ros2_bag_record)

    return ld
