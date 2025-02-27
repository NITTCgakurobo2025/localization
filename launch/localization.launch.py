import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    robots = [
        "R1",
        "R2"
    ]

    launch_elements = []

    for robot in robots:
        launch_elements.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('localization'),
                        'launch', 'scan_matcher.launch.py'
                    ),
                ),
                launch_arguments={'robot_name': robot}.items()
            )
        )

    return LaunchDescription(launch_elements)
