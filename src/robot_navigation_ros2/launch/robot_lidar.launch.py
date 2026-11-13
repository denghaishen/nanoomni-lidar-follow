
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

base_control_pkg_dir = LaunchConfiguration(
    'base_control_pkg_dir',
    default=os.path.join(get_package_share_directory('base_control_ros2'))
    )

robot_navigation_pkg_dir = LaunchConfiguration(
    'robot_navigation_pkg_dir',
    default=os.path.join(get_package_share_directory('robot_navigation_ros2'))
    )


def generate_launch_description():

    # use_urdf=true 时:
    #   - base_control.launch.py / lidar.launch.py 内部跳过各自静态 TF
    #   - 额外 include nanoomni_description 的 urdf.launch.py 发布 robot_state_publisher
    use_urdf = LaunchConfiguration('use_urdf', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_urdf', default_value='false',
            description='If true, load NanoOmni URDF via robot_state_publisher '
                        'and skip overlapping static_transform_publisher nodes.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([base_control_pkg_dir, '/launch', '/base_control.launch.py']),
            launch_arguments={'use_urdf': use_urdf}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_navigation_pkg_dir, '/launch', '/lidar.launch.py']),
            launch_arguments={'use_urdf': use_urdf}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_navigation_pkg_dir, '/launch', '/urdf.launch.py']),
            condition=IfCondition(use_urdf),
        ),
    ])


