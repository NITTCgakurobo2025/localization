import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    name_parameter = DeclareLaunchArgument(
        'robot_name',
        default_value='R1')
    name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        name_parameter,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('localization'),
                    'launch', 'scan_matcher.launch.py'
                ),
            ),
            launch_arguments={'robot_name': name}.items()
        ),
        Node(
            package='localization',
            executable='twist_odom',
            name=[name, '_twist_odom'],
            output='screen',
            parameters=[{
                'parant_frame': [name, '/odom'],
                'frame_id': [name, '/base_footprint'],
                        'twist_topic': [name, '/cmd_vel'],
                        'odom_topic': [name, '/pose_odom'],
                        'reset_service': [name, '/odom_reset'],
                        'x_initial': 0.0,
                        'y_initial': 0.0,
                        'theta_initial': 0.0
                        }]
        )
    ])
