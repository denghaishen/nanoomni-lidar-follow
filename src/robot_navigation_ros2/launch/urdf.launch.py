#!/usr/bin/env python3
"""
robot_navigation_ros2 / urdf.launch.py
======================================
Thin wrapper that includes ``nanoomni_description/launch/urdf.launch.py``.
Keeps the public command line consistent (``ros2 launch
robot_navigation_ros2 urdf.launch.py``) regardless of where the URDF
package lives.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    description_pkg = get_package_share_directory('nanoomni_description')
    urdf_launch = os.path.join(description_pkg, 'launch', 'urdf.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(urdf_launch),
        ),
    ])
