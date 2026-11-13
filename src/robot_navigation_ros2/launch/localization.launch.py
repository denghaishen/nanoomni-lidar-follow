#!/usr/bin/env python3
"""
localization.launch.py
======================
启动 "有地图导航" 模式的定位栈:

    1. map_server                   (加载 .pgm/.yaml 离线地图, 发布 /map latched)
    2. amcl                         (粒子滤波定位, 发布 map->odom TF)
    3. lifecycle_manager_localization (统一 configure/activate 上述节点)

注意:
    * 与 cartographer.launch.py 互斥! 二者都会发 map->odom TF + /map.
      - "边走边建图": cartographer.launch.py
      - "有图导航":   localization.launch.py
    * map_server 用 yaml_filename 参数注入地图路径, 默认 ~/map/my_map.yaml.
    * AMCL TF 链: map -> odom (本节点发) -> base_footprint (base_control_ros2 发).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_navigation_ros2')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_map = os.path.expanduser('~/map/my_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=default_params)
    map_yaml = LaunchConfiguration('map', default=default_map)
    autostart = LaunchConfiguration('autostart', default='true')

    lifecycle_nodes = ['map_server', 'amcl']

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use Gazebo /clock if true'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Full path to nav2_params.yaml'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Full path to map yaml file'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Auto configure+activate lifecycle nodes'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                params_file,
                # 用命令行参数把 map 路径注入到 yaml_filename, 覆盖 nav2_params.yaml 中的占位
                {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml},
            ],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            }],
        ),
    ])
