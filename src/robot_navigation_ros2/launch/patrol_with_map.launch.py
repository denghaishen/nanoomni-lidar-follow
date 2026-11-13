#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('robot_navigation_ros2')
    default_map = os.path.expanduser('~/map/my_map.yaml')
    default_route = os.path.join(package_share, 'config', 'patrol_route.yaml')

    map_yaml = LaunchConfiguration('map')
    route_file = LaunchConfiguration('route_file')
    loop_count = LaunchConfiguration('loop_count')
    use_rviz = LaunchConfiguration('use_rviz')
    patrol_autostart = LaunchConfiguration('patrol_autostart')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the map YAML file',
        ),
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
            'use_rviz',
            default_value='true',
            description='Launch the Nav2 RViz configuration',
        ),
        DeclareLaunchArgument(
            'patrol_autostart',
            default_value='false',
            description='Start patrol when Nav2 and TF become ready',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, 'launch', 'nav_with_map.launch.py')
            ),
            launch_arguments={
                'map': map_yaml,
                'use_rviz': use_rviz,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    package_share, 'launch', 'patrol_manager.launch.py'
                )
            ),
            launch_arguments={
                'route_file': route_file,
                'loop_count': loop_count,
                'patrol_autostart': patrol_autostart,
            }.items(),
        ),
    ])
