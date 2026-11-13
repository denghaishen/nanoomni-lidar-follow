#!/usr/bin/env python3
"""
slam_nav.launch.py
==================
一键启动 NanoOmni 在线 SLAM + 导航全流程:

    1. base_control + lidar + URDF        (use_urdf=true 走 robot_state_publisher)
    2. cartographer_node + occupancy_grid (实时建图, 发布 map TF + /map topic)
    3. Nav2 stack                          (controller / planner / bt / behavior 等)
    4. RViz (可选, 默认开启)

不需要预先保存的地图. 一边走一边建图一边导航 ("无地图导航").
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_navigation_ros2')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    start_web = LaunchConfiguration('start_web', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use Gazebo /clock if true'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz nav visualisation'),
        DeclareLaunchArgument(
            'start_web',
            default_value='false',
            description='Start embedded web server instead of persistent controller',
        ),

        # 1) 硬件层: 底盘 + 雷达 + URDF (use_urdf=true 必开, 否则 TF 会重复)
        #    scoped 隔离: 否则 lidar/ydlidar.launch.py 先声明的 params_file(=S4.yaml)
        #    会泄漏到顶层, 使 Nav2 节点全部误加载 S4.yaml (critics 缺失, 无法激活)
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_share, 'launch', 'robot_lidar.launch.py')),
                    launch_arguments={'use_urdf': 'true'}.items(),
                ),
            ],
        ),

        # 2) Cartographer 在线建图
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'cartographer.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # 3) Nav2 stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'nav2.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'start_web': start_web,
            }.items(),
        ),

        # 4) RViz (可选)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'rviz_nav.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(use_rviz),
        ),
    ])
