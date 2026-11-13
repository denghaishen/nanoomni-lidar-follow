#!/usr/bin/env python3
"""
nav2.launch.py
==============
启动 Nav2 stack (无 AMCL / 无 map_server, 配合 Cartographer online SLAM).

启动节点:
    1. controller_server     (MPPI Omni controller)
    2. planner_server        (NavfnPlanner A*)
    3. behavior_server       (spin / backup / wait / drive_on_heading)
    4. bt_navigator          (Behavior Tree)
    5. waypoint_follower     (多点)
    6. motion_command_arbiter (Nav2 / 跟随速度源仲裁; 按模式切换前方停止区)
    7. lidar_target_follower  (激光点云双向距离保持)
    8. lidar_follow_web       (网页雷达显示与圆形区域选择)
    9. velocity_smoother     (cmd_vel_selected -> cmd_vel_smoothed)
    10. scan_safety_watchdog (激光/TF 超时保护)
    11. collision_monitor    (正前方停止区仅在 NAV 模式生效, 最终输出 cmd_vel)
    12. lifecycle_manager_navigation (统一管理 Nav2 Lifecycle 节点)

速度链: controller/behavior -> cmd_vel_nav ─┐
       lidar_target_follower -> cmd_vel_follow ┴-> motion_command_arbiter
       -> cmd_vel_selected -> velocity_smoother -> cmd_vel_smoothed
       -> scan_safety_watchdog -> cmd_vel_collision_input
       -> collision_monitor -> cmd_vel -> 底盘.

跟随模式 (FOLLOW): motion_command_arbiter 会关闭 collision_monitor 的
    FrontStopZone，使接近目标时的倒车命令不被前方停止区拦截；
    NAV/STOP 模式重新启用前方停止区。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_navigation_ros2')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=default_params)
    autostart = LaunchConfiguration('autostart', default='true')
    start_web = LaunchConfiguration('start_web', default='true')

    # Nav2 节点列表 (lifecycle_manager 必须 activate 这些节点)
    lifecycle_nodes = [
        'collision_monitor',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    # Controller Server -> remap cmd_vel 到 cmd_vel_nav, 让 velocity_smoother 接收
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use Gazebo /clock if true'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Full path to Nav2 yaml'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Auto configure+activate Nav2 lifecycle nodes',
        ),
        DeclareLaunchArgument(
            'start_web',
            default_value='true',
            description='Start the embedded lidar web server',
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            # Recovery 行为也走平滑器
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='robot_navigation_ros2',
            executable='motion_command_arbiter',
            name='motion_command_arbiter',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='robot_navigation_ros2',
            executable='lidar_target_follower',
            name='lidar_target_follower',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='robot_navigation_ros2',
            executable='lidar_follow_web',
            name='lidar_follow_web',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            condition=IfCondition(start_web),
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('cmd_vel', 'cmd_vel_selected'),
                ('cmd_vel_smoothed', 'cmd_vel_smoothed'),
            ],
        ),
        Node(
            package='robot_navigation_ros2',
            executable='scan_safety_watchdog',
            name='scan_safety_watchdog',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            }],
        ),
    ])
