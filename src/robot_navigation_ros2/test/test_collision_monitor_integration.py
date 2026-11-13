import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from robot_navigation_ros2.scan_safety_watchdog import ScanSafetyWatchdog


SCAN_TOPIC = '/collision_test/scan'
SMOOTHED_TOPIC = '/collision_test/cmd_vel_smoothed'
MONITOR_INPUT_TOPIC = '/collision_test/cmd_vel_collision_input'
OUTPUT_TOPIC = '/collision_test/cmd_vel'
BASE_FRAME = 'collision_test_base_link'
LASER_FRAME = 'collision_test_laser'


def spin_until(executor, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def make_scan(node, finite_ranges, stamp_age=0.0):
    scan = LaserScan()
    stamp = node.get_clock().now() - Duration(seconds=stamp_age)
    scan.header.stamp = stamp.to_msg()
    scan.header.frame_id = LASER_FRAME
    scan.angle_min = -0.10
    scan.angle_max = 0.10
    scan.angle_increment = 0.05
    scan.range_min = 0.05
    scan.range_max = 10.0
    scan.ranges = list(finite_ranges)
    return scan


def wait_for_valid_scan(
    executor,
    node,
    watchdog,
    scan_publisher,
    ranges,
    timeout=5.0,
):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        scan_publisher.publish(make_scan(node, ranges))
        executor.spin_once(timeout_sec=0.05)
        if watchdog._scan_is_valid():
            return True
    return False


def publish_case(
    executor,
    node,
    scan_publisher,
    cmd_publisher,
    ranges,
    stamp_age=0.0,
):
    for _ in range(8):
        scan_publisher.publish(make_scan(node, ranges, stamp_age))
        command = Twist()
        command.linear.x = 0.20
        command.linear.y = 0.03
        command.angular.z = 0.40
        cmd_publisher.publish(command)
        executor.spin_once(timeout_sec=0.05)


def latest_velocity(outputs):
    assert outputs
    message = outputs[-1]
    return message.linear.x, message.linear.y, message.angular.z


def assert_velocity(executor, outputs, expected):
    def expected_velocity_received():
        return (
            bool(outputs)
            and latest_velocity(outputs) == pytest.approx(expected)
        )

    assert spin_until(executor, expected_velocity_received)


def transition(executor, client, transition_id):
    request = ChangeState.Request()
    request.transition.id = transition_id
    future = client.call_async(request)
    assert spin_until(executor, future.done)
    assert future.result().success


def test_synthetic_person_stop_clear_and_scan_timeout():
    if rclpy.ok():
        rclpy.shutdown()
    os.environ['ROS_DOMAIN_ID'] = '191'
    rclpy.init()

    params = {
        'collision_monitor': {
            'ros__parameters': {
                'use_sim_time': False,
                'base_frame_id': BASE_FRAME,
                'odom_frame_id': 'collision_test_odom',
                'cmd_vel_in_topic': MONITOR_INPUT_TOPIC,
                'cmd_vel_out_topic': OUTPUT_TOPIC,
                'transform_tolerance': 0.2,
                'source_timeout': 0.5,
                'base_shift_correction': False,
                'stop_pub_timeout': 2.0,
                'polygons': ['FrontStopZone'],
                'FrontStopZone': {
                    'type': 'polygon',
                    'points': [
                        0.41, 0.25,
                        0.41, -0.25,
                        0.10, -0.25,
                        0.10, 0.25,
                    ],
                    'action_type': 'stop',
                    'max_points': 2,
                    'visualize': True,
                    'polygon_pub_topic': 'front_stop_zone',
                    'enabled': True,
                },
                'observation_sources': ['scan'],
                'scan': {
                    'type': 'scan',
                    'topic': SCAN_TOPIC,
                    'enabled': True,
                },
            }
        }
    }

    with tempfile.TemporaryDirectory() as temporary_directory:
        params_path = Path(temporary_directory) / 'params.yaml'
        params_path.write_text(yaml_dump(params), encoding='utf-8')
        environment = os.environ.copy()
        static_tf = subprocess.Popen(
            [
                '/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', BASE_FRAME,
                '--child-frame-id', LASER_FRAME,
            ],
            env=environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        monitor = subprocess.Popen(
            [
                '/opt/ros/humble/lib/nav2_collision_monitor/collision_monitor',
                '--ros-args',
                '--params-file', str(params_path),
            ],
            env=environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        test_node = Node('collision_monitor_integration_test')
        watchdog = ScanSafetyWatchdog(parameter_overrides=[
            Parameter('scan_topic', value=SCAN_TOPIC),
            Parameter('cmd_vel_in_topic', value=SMOOTHED_TOPIC),
            Parameter('cmd_vel_out_topic', value=MONITOR_INPUT_TOPIC),
            Parameter('base_frame_id', value=BASE_FRAME),
            Parameter('scan_timeout', value=0.5),
            Parameter('transform_timeout', value=0.1),
        ])
        executor = SingleThreadedExecutor()
        executor.add_node(test_node)
        executor.add_node(watchdog)
        outputs = []
        test_node.create_subscription(
            Twist, OUTPUT_TOPIC, outputs.append, 10
        )
        scan_publisher = test_node.create_publisher(
            LaserScan, SCAN_TOPIC, qos_profile_sensor_data
        )
        cmd_publisher = test_node.create_publisher(
            Twist, SMOOTHED_TOPIC, 10
        )
        lifecycle_client = test_node.create_client(
            ChangeState, '/collision_monitor/change_state'
        )

        try:
            assert lifecycle_client.wait_for_service(timeout_sec=8.0)
            transition(
                executor,
                lifecycle_client,
                Transition.TRANSITION_CONFIGURE,
            )
            transition(
                executor,
                lifecycle_client,
                Transition.TRANSITION_ACTIVATE,
            )
            assert spin_until(
                executor,
                lambda: cmd_publisher.get_subscription_count() >= 1,
            )

            clear = [math.inf] * 5
            outside = [0.60] * 5
            two_inside = [0.40, 0.40, math.inf, math.inf, math.inf]
            three_inside = [0.40, 0.40, 0.40, math.inf, math.inf]
            assert wait_for_valid_scan(
                executor,
                test_node,
                watchdog,
                scan_publisher,
                clear,
            )

            publish_case(
                executor, test_node, scan_publisher, cmd_publisher, clear
            )
            assert spin_until(executor, lambda: bool(outputs))
            assert_velocity(executor, outputs, (0.20, 0.03, 0.40))

            outputs.clear()
            publish_case(
                executor, test_node, scan_publisher, cmd_publisher, outside
            )
            assert_velocity(executor, outputs, (0.20, 0.03, 0.40))

            outputs.clear()
            publish_case(
                executor, test_node, scan_publisher, cmd_publisher, two_inside
            )
            assert_velocity(executor, outputs, (0.20, 0.03, 0.40))

            outputs.clear()
            publish_case(
                executor,
                test_node,
                scan_publisher,
                cmd_publisher,
                three_inside,
            )
            assert_velocity(executor, outputs, (0.0, 0.0, 0.0))

            outputs.clear()
            publish_case(
                executor, test_node, scan_publisher, cmd_publisher, clear
            )
            assert_velocity(executor, outputs, (0.20, 0.03, 0.40))

            outputs.clear()
            deadline = time.monotonic() + 0.7
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            command = Twist()
            command.linear.x = 0.20
            cmd_publisher.publish(command)
            assert spin_until(executor, lambda: bool(outputs))
            assert_velocity(executor, outputs, (0.0, 0.0, 0.0))

            outputs.clear()
            publish_case(
                executor,
                test_node,
                scan_publisher,
                cmd_publisher,
                clear,
                stamp_age=2.0,
            )
            assert_velocity(executor, outputs, (0.0, 0.0, 0.0))
        finally:
            executor.remove_node(watchdog)
            executor.remove_node(test_node)
            watchdog.destroy_node()
            test_node.destroy_node()
            executor.shutdown(timeout_sec=1.0)
            for process in (monitor, static_tf):
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3.0)
            if rclpy.ok():
                rclpy.shutdown()


def yaml_dump(data):
    import yaml
    return yaml.safe_dump(data, sort_keys=False)
