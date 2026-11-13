from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_params():
    path = PACKAGE_ROOT / 'config' / 'nav2_params.yaml'
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def test_front_stop_zone_geometry_and_threshold():
    params = load_params()['collision_monitor']['ros__parameters']
    zone = params['FrontStopZone']

    assert params['base_frame_id'] == 'base_link'
    assert params['polygons'] == ['FrontStopZone']
    assert zone['action_type'] == 'stop'
    assert zone['points'] == [
        0.41, 0.25,
        0.41, -0.25,
        0.10, -0.25,
        0.10, 0.25,
    ]
    assert zone['max_points'] == 2
    assert zone['visualize'] is True
    assert zone['polygon_pub_topic'] == 'front_stop_zone'


def test_velocity_chain_and_scan_fail_safe_configuration():
    params = load_params()
    arbiter = params['motion_command_arbiter']['ros__parameters']
    follower = params['lidar_target_follower']['ros__parameters']
    web = params['lidar_follow_web']['ros__parameters']
    watchdog = params['scan_safety_watchdog']['ros__parameters']
    monitor = params['collision_monitor']['ros__parameters']

    assert arbiter['nav_cmd_vel_topic'] == '/cmd_vel_nav'
    assert arbiter['follow_cmd_vel_topic'] == '/cmd_vel_follow'
    assert arbiter['output_cmd_vel_topic'] == '/cmd_vel_selected'
    assert arbiter['initial_mode'] == 'NAV'
    assert arbiter['gate_collision_monitor'] is True
    assert arbiter['collision_monitor_param_service'] == '/collision_monitor/set_parameters'
    assert arbiter['collision_front_stop_param'] == 'FrontStopZone.enabled'
    assert follower['cmd_vel_out_topic'] == '/cmd_vel_follow'
    assert follower['selection_topic'] == '/follow/selection'
    assert follower['start_on_target_selection'] is True
    assert follower['min_target_points'] == 1
    assert follower['allow_reverse'] is True
    assert follower['max_forward_speed'] == 0.30
    assert follower['max_reverse_speed'] == 0.20
    assert follower['use_lateral_velocity'] is False
    assert follower['rotate_first_angle'] == 0.30
    assert follower['angular_deadband'] == 0.10
    assert follower['distance_deadband'] == 0.05
    assert web['selection_topic'] == '/follow/selection'
    assert web['battery_topic'] == '/battery'
    assert web['broadcast_frequency'] == 5.0
    assert web['max_points'] == 160
    assert web['web_bind_address'] == '0.0.0.0'
    assert web['web_port'] == 8080
    assert 'safety_status_topic' not in web
    assert web['follow_distance_param_service'] == '/lidar_target_follower/set_parameters'
    assert web['follow_distance_param_name'] == 'follow_distance'
    assert web['follow_cmd_topic'] == '/cmd_vel_follow'
    assert web['distance_min'] == 0.40
    assert web['distance_max'] == 2.00
    assert web['initial_follow_distance'] == 1.0
    assert web['system_control_enabled'] is False
    assert web['system_ros2_executable'] == '/opt/ros/humble/bin/ros2'
    assert web['system_launch_package'] == 'robot_navigation_ros2'
    assert web['system_launch_file'] == 'slam_nav.launch.py'
    assert web['system_launch_use_rviz'] is False
    assert web['system_launch_start_web'] is False
    assert watchdog['cmd_vel_in_topic'] == '/cmd_vel_smoothed'
    assert watchdog['cmd_vel_out_topic'] == '/cmd_vel_collision_input'
    assert watchdog['scan_topic'] == '/scan'
    assert watchdog['scan_timeout'] == 0.5
    assert monitor['cmd_vel_in_topic'] == '/cmd_vel_collision_input'
    assert monitor['cmd_vel_out_topic'] == '/cmd_vel'
    assert monitor['source_timeout'] == 0.5
    assert monitor['observation_sources'] == ['scan']
    assert monitor['scan']['topic'] == '/scan'


def test_rviz_highlights_scan_and_hides_costmaps_for_target_selection():
    def all_displays(displays):
        for display in displays:
            yield display
            yield from all_displays(display.get('Displays', []))

    for filename in ('nav.rviz', 'slam.rviz'):
        data = yaml.safe_load(
            (PACKAGE_ROOT / 'rviz' / filename).read_text(encoding='utf-8')
        )
        tools = data['Visualization Manager']['Tools']
        tool_topics = [
            tool.get('Topic', {}).get('Value') for tool in tools
        ]
        if filename == 'nav.rviz':
            assert '/clicked_point' not in tool_topics
            assert tool_topics.count('/follow/target') == 1
        else:
            assert '/clicked_point' in tool_topics
            assert '/follow/target' in tool_topics

        displays = list(all_displays(
            data['Visualization Manager']['Displays']
        ))
        displays_by_topic = {
            display.get('Topic', {}).get('Value'): display
            for display in displays
            if display.get('Topic', {}).get('Value')
        }
        scan = displays_by_topic['/scan']
        assert scan['Color'] == '255; 0; 0'
        assert scan['Color Transformer'] == 'FlatColor'
        assert scan['Size (Pixels)'] == 7
        assert displays_by_topic['/follow/tracked_target']['Enabled'] is True
        assert displays_by_topic['/follow/selection']['Enabled'] is True
        assert displays_by_topic['/global_costmap/costmap']['Enabled'] is False
        assert displays_by_topic['/local_costmap/costmap']['Enabled'] is False


def test_map_launch_warns_that_initial_pose_is_required():
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'nav_with_map.launch.py'
    ).read_text(encoding='utf-8')

    assert '2D Pose Estimate' in launch_source
    assert '真实车头' in launch_source


def test_amcl_requires_explicit_pose_instead_of_assuming_map_origin():
    amcl = load_params()['amcl']['ros__parameters']

    assert amcl['set_initial_pose'] is False
    assert amcl['always_reset_initial_pose'] is False


def test_progress_timeout_covers_sixty_second_obstruction():
    params = load_params()
    progress = params['controller_server']['ros__parameters'][
        'progress_checker'
    ]

    assert progress['movement_time_allowance'] >= 120.0


def test_launch_routes_all_velocity_sources_through_safety_chain():
    launch_path = PACKAGE_ROOT / 'launch' / 'nav2.launch.py'
    launch_source = launch_path.read_text(encoding='utf-8')

    assert "'collision_monitor'," in launch_source
    assert "executable='motion_command_arbiter'" in launch_source
    assert "executable='lidar_target_follower'" in launch_source
    assert "executable='lidar_follow_web'" in launch_source
    assert 'condition=IfCondition(start_web)' in launch_source
    assert "executable='scan_safety_watchdog'" in launch_source
    assert "executable='reverse_safety_filter'" not in launch_source
    assert "executable='collision_monitor'" in launch_source
    assert "('cmd_vel_smoothed', 'cmd_vel')" not in launch_source
    assert "('cmd_vel', 'cmd_vel_nav')" in launch_source
    assert "('cmd_vel', 'cmd_vel_selected')" in launch_source


def test_persistent_web_control_launch_and_service_configuration():
    slam_source = (
        PACKAGE_ROOT / 'launch' / 'slam_nav.launch.py'
    ).read_text(encoding='utf-8')
    mapped_source = (
        PACKAGE_ROOT / 'launch' / 'nav_with_map.launch.py'
    ).read_text(encoding='utf-8')
    web_source = (
        PACKAGE_ROOT / 'launch' / 'web_control.launch.py'
    ).read_text(encoding='utf-8')
    service_source = (
        PACKAGE_ROOT / 'systemd' / 'robot-web-control.service'
    ).read_text(encoding='utf-8')

    assert "start_web = LaunchConfiguration('start_web', default='false')" in slam_source
    assert "'start_web': start_web" in slam_source
    assert "start_web = LaunchConfiguration('start_web', default='false')" in mapped_source
    assert "'system_control_enabled': True" in web_source
    assert 'web_control.launch.py' in service_source
    assert 'KillSignal=SIGINT' in service_source
    assert 'KillMode=control-group' in service_source
    assert 'ROS_LOCALHOST_ONLY=1' in service_source
