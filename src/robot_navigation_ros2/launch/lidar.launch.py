#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os



robot_navigation_pkg_dir = LaunchConfiguration(
    'robot_navigation_pkg_dir',
    default=os.path.join(get_package_share_directory('robot_navigation_ros2'))
    )


def generate_launch_description():

    # use_urdf=true 时由 nanoomni_description 的 robot_state_publisher 发布
    # base_link->base_laser_link, 此处不再发静态 TF, 避免重复广播.
    use_urdf = LaunchConfiguration('use_urdf', default='false')

    # NanoOmni 全向轮机器人 + YDLidar 专用
    static_transform_args = ['0.05', '0', '0.16', '3.14159', '0', '0', 'base_link', 'base_laser_link']

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_urdf', default_value='false',
            description='If true, skip base_link->base_laser_link static TF '
                        '(provided by nanoomni_description robot_state_publisher).'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_navigation_pkg_dir,'/launch', '/ydlidar.launch.py']),
            launch_arguments={'serial_port': '/dev/lidar'}.items(),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=static_transform_args,  # set tf transform data
            output='screen',
            condition=UnlessCondition(use_urdf),
        ),
    ])


