from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    name_parameter = DeclareLaunchArgument(
        'robot_name',
        default_value='RobotName')

    name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        name_parameter,
        Node(
            package='localization',
            executable='scan_matcher',
            name=[name, '_scan_matcher'],
            output='screen',
            parameters=[{
                'output_frame': [name, '/odom'],
                'parent_frame': 'map',
                'input_topic': ['/', name, '/downsampled_front_scan'],
                'imu_topic': ['/', name, '/imu'],
                'pose_topic': ['/', name, '/pose'],
                'odom_reset_service': ['/', name, '/odom_reset'],
                'odom_reset_topic': ['/', name, '/odom_reset'],
            }]
        )
    ])
