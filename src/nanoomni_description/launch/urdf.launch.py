#!/usr/bin/env python3
"""
nanoomni_description / urdf.launch.py
=====================================
启动 robot_state_publisher 加载 NanoOmni URDF 模型, 发布 /robot_description
和静态 TF (base_footprint -> base_link, base_link -> {imu, base_laser_link,
camera_link, base_camera_link, 4 wheels}).

使用前请确保: base_control.launch.py / lidar.launch.py 通过 use_urdf:=true
禁用各自的 static_transform_publisher, 避免 TF 重复发布.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nanoomni_description')
    default_urdf = os.path.join(pkg_share, 'urdf', 'nanoomni_description.urdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # NanoOmni URDF 中所有 joint 均为 fixed, robot_state_publisher 单独
    # 即可发布全部静态 TF, 无需 joint_state_publisher.
    with open(default_urdf, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use Gazebo /clock if true'),
        DeclareLaunchArgument('urdf_file', default_value=default_urdf,
                              description='Full path to NanoOmni URDF'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
            }],
        ),
    ])
