#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_navigation_ros2')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    params_file = LaunchConfiguration('params_file', default=default_params)

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to robot navigation parameters',
        ),
        Node(
            package='robot_navigation_ros2',
            executable='lidar_follow_web',
            name='lidar_follow_web',
            output='screen',
            parameters=[params_file, {'system_control_enabled': True}],
        ),
    ])
