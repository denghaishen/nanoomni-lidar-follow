#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # use_urdf=true 时由 nanoomni_description 的 robot_state_publisher 发布
    # base_footprint->base_link / base_link->imu, 此处不再发对应 static TF,
    # 避免 TF 重复广播.
    use_urdf = LaunchConfiguration('use_urdf', default='false')

    # NanoOmni 全向轮机器人专用
    # [P2-2] IMU 在 base_link 原点, X 轴朝前, 无需偏移和旋转
    imu_static_transform_args = ['0', '0', '0', '0', '0', '0', 'base_link', 'imu']
    # [P2-2] 麦轮直径 64mm, 轮轴中心离地 32mm, base_link 定义在轮轴中心高度
    base_footprint_static_transform_args = ['0.0', '0.0', '0.032', '0.0', '0.0', '0.0', 'base_footprint', 'base_link']
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_urdf', default_value='false',
            description='If true, skip static TF for imu / base_footprint '
                        '(provided by nanoomni_description robot_state_publisher).'),
        LifecycleNode(
            package='base_control_ros2',
            executable='base_control_node',
            name='base_control_ros2',
            output='screen',
            namespace=''
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=imu_static_transform_args,  # set tf transform data
            output='log',
            condition=UnlessCondition(use_urdf),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_footprint',
            arguments=base_footprint_static_transform_args,  # set tf transform data
            output='log',
            condition=UnlessCondition(use_urdf),
        ),
    ])
