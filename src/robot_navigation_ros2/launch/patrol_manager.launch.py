#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('robot_navigation_ros2')
    default_route = os.path.join(package_share, 'config', 'patrol_route.yaml')

    route_file = LaunchConfiguration('route_file')
    loop_count = LaunchConfiguration('loop_count')
    patrol_autostart = LaunchConfiguration('patrol_autostart')

    return LaunchDescription([
        DeclareLaunchArgument(
            'route_file',
            default_value=default_route,
            description='Full path to the patrol route YAML file',
        ),
        DeclareLaunchArgument(
            'loop_count',
            default_value='-1',
            description='Override route loop_count; -1 uses the YAML value',
        ),
        DeclareLaunchArgument(
            'patrol_autostart',
            default_value='false',
            description='Start patrol when Nav2 and TF become ready',
        ),
        Node(
            package='robot_navigation_ros2',
            executable='patrol_manager',
            name='patrol_manager',
            output='screen',
            parameters=[{
                'route_file': route_file,
                'loop_count': ParameterValue(loop_count, value_type=int),
                'autostart': ParameterValue(
                    patrol_autostart, value_type=bool
                ),
            }],
        ),
    ])
