#!/usr/bin/env python3
"""
nav_with_map.launch.py
======================
"有地图导航" 一键启动脚本 (与 slam_nav.launch.py 互斥):

    1. base_control + lidar + URDF       (硬件层, use_urdf=true)
    2. map_server + amcl + lifecycle      (定位层, 替代 cartographer)
    3. Nav2 stack                          (规划/控制/BT/恢复)
    4. RViz nav (可选)

使用前提: 已用 SLAM 流程建好地图并保存为 .pgm/.yaml
        默认地图路径: ~/map/my_map.yaml
        覆盖方式:    ros2 launch ... nav_with_map.launch.py map:=/path/to/my.yaml

启动后:
    * 必须先在 RViz 使用 "2D Pose Estimate" 标出机器人真实位置与车头方向
    * 确认红色激光点与地图墙体重合后，才能开始地图导航
    * 网页跟随圈选使用 base_link 局部坐标，不依赖 AMCL 初始位姿
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_navigation_ros2')
    default_map = os.path.expanduser('~/map/my_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    start_web = LaunchConfiguration('start_web', default='false')
    map_yaml = LaunchConfiguration('map', default=default_map)

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
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Full path to map yaml file'),
        LogInfo(msg=(
            '[必须操作] RViz 地图出现后，先使用 2D Pose Estimate 标出机器人'
            '真实位置，并把箭头指向真实车头；完成定位后才能导航或点选跟随目标。'
        )),

        # 1) 硬件层: 底盘 + 雷达 + URDF (use_urdf=true 必开)
        #    scoped 隔离: 否则 lidar/ydlidar.launch.py 先声明的 params_file(=S4.yaml)
        #    会泄漏到顶层, 使 map_server/amcl/Nav2 节点全部误加载 S4.yaml
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

        # 2) 定位层: map_server + amcl (替代 cartographer)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'localization.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml,
            }.items(),
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
