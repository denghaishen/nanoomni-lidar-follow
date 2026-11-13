import math
import os
import time

from geometry_msgs.msg import (
    Point32,
    PointStamped,
    PolygonStamped,
    Twist,
)
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters

from robot_navigation_ros2.lidar_target_follower import (
    FollowState,
    LidarTargetFollower,
)
from robot_navigation_ros2.motion_command_arbiter import MotionCommandArbiter


PREFIX = '/target_follow_test'
SCAN_TOPIC = f'{PREFIX}/scan'
TARGET_TOPIC = f'{PREFIX}/target'
SELECTION_TOPIC = f'{PREFIX}/selection'
NAV_TOPIC = f'{PREFIX}/cmd_vel_nav'
FOLLOW_TOPIC = f'{PREFIX}/cmd_vel_follow'
SELECTED_TOPIC = f'{PREFIX}/cmd_vel_selected'
MODE_REQUEST_TOPIC = f'{PREFIX}/motion_mode/request'
MODE_STATUS_TOPIC = f'{PREFIX}/motion_mode/status'
BASE_FRAME = 'target_follow_test_base_link'


def spin_until(executor, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def spin_until_with_scan(
    executor,
    predicate,
    scan_publisher,
    node,
    distance,
    bearing=0.0,
    timeout=5.0,
):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        scan_publisher.publish(make_scan(node, distance, bearing))
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return True
    return False


def call_trigger(executor, client):
    request = Trigger.Request()
    future = client.call_async(request)
    assert spin_until(executor, future.done)
    return future.result()


def make_scan(node, distance=None, bearing=0.0):
    message = LaserScan()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = BASE_FRAME
    message.angle_min = -math.pi
    message.angle_increment = math.pi / 180.0
    message.angle_max = math.pi
    message.range_min = 0.05
    message.range_max = 10.0
    ranges = [math.inf] * 361
    if distance is not None:
        index = int(round((bearing + math.pi) / (math.pi / 180.0)))
        ranges[index] = distance
    message.ranges = ranges
    return message


def make_selection(node, center_x, center_y, radius):
    message = PolygonStamped()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = BASE_FRAME
    for index in range(16):
        angle = 2.0 * math.pi * index / 16
        point = Point32()
        point.x = center_x + radius * math.cos(angle)
        point.y = center_y + radius * math.sin(angle)
        message.polygon.points.append(point)
    return message


def publish_scans(executor, publisher, node, distance, bearing=0.0, count=6):
    for _ in range(count):
        publisher.publish(make_scan(node, distance, bearing))
        executor.spin_once(timeout_sec=0.05)


def wait_for_velocity(
    executor,
    outputs,
    predicate,
    keep_scan_fresh=None,
    timeout=3.0,
):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if keep_scan_fresh is not None:
            keep_scan_fresh()
        executor.spin_once(timeout_sec=0.05)
        if outputs and predicate(outputs[-1]):
            return True
    return False


def build_stack():
    arbiter = MotionCommandArbiter(parameter_overrides=[
        Parameter('nav_cmd_vel_topic', value=NAV_TOPIC),
        Parameter('follow_cmd_vel_topic', value=FOLLOW_TOPIC),
        Parameter('output_cmd_vel_topic', value=SELECTED_TOPIC),
        Parameter('mode_request_topic', value=MODE_REQUEST_TOPIC),
        Parameter('mode_status_topic', value=MODE_STATUS_TOPIC),
        Parameter('nav_service', value=f'{PREFIX}/motion_mode/nav'),
        Parameter('follow_service', value=f'{PREFIX}/motion_mode/follow'),
        Parameter('stop_service', value=f'{PREFIX}/motion_mode/stop'),
        Parameter('initial_mode', value='NAV'),
        Parameter('switch_stop_duration', value=0.02),
        Parameter('gate_collision_monitor', value=False),
    ])
    follower = LidarTargetFollower(parameter_overrides=[
        Parameter('scan_topic', value=SCAN_TOPIC),
        Parameter('target_topic', value=TARGET_TOPIC),
        Parameter('selection_topic', value=SELECTION_TOPIC),
        Parameter('cmd_vel_out_topic', value=FOLLOW_TOPIC),
        Parameter('status_topic', value=f'{PREFIX}/follow/status'),
        Parameter(
            'tracked_target_topic',
            value=f'{PREFIX}/follow/tracked_target',
        ),
        Parameter('mode_request_topic', value=MODE_REQUEST_TOPIC),
        Parameter('mode_status_topic', value=MODE_STATUS_TOPIC),
        Parameter('start_service', value=f'{PREFIX}/follow/start'),
        Parameter('stop_service', value=f'{PREFIX}/follow/stop'),
        Parameter(
            'patrol_pause_service',
            value=f'{PREFIX}/missing_patrol_pause',
        ),
        Parameter('base_frame_id', value=BASE_FRAME),
        Parameter('start_on_target_selection', value=True),
        Parameter('target_loss_timeout', value=0.30),
        Parameter('control_frequency', value=20.0),
    ])
    return arbiter, follower


def _prepare(domain):
    if rclpy.ok():
        rclpy.shutdown()
    os.environ['ROS_DOMAIN_ID'] = domain
    rclpy.init()
    arbiter, follower = build_stack()
    test_node = Node('target_follow_integration_test')
    executor = SingleThreadedExecutor()
    for node in (arbiter, follower, test_node):
        executor.add_node(node)

    scan_publisher = test_node.create_publisher(
        LaserScan, SCAN_TOPIC, qos_profile_sensor_data
    )
    target_publisher = test_node.create_publisher(
        PointStamped, TARGET_TOPIC, 10
    )
    selection_publisher = test_node.create_publisher(
        PolygonStamped, SELECTION_TOPIC, 10
    )
    nav_publisher = test_node.create_publisher(Twist, NAV_TOPIC, 10)
    outputs = []
    test_node.create_subscription(Twist, SELECTED_TOPIC, outputs.append, 10)
    nav_client = test_node.create_client(
        Trigger, f'{PREFIX}/motion_mode/nav'
    )
    return {
        'arbiter': arbiter,
        'follower': follower,
        'test_node': test_node,
        'executor': executor,
        'scan_publisher': scan_publisher,
        'target_publisher': target_publisher,
        'selection_publisher': selection_publisher,
        'nav_publisher': nav_publisher,
        'outputs': outputs,
        'nav_client': nav_client,
    }


def _teardown(ctx):
    executor = ctx['executor']
    for key in ('arbiter', 'follower', 'test_node'):
        node = ctx[key]
        executor.remove_node(node)
        node.destroy_node()
    executor.shutdown(timeout_sec=1.0)
    if rclpy.ok():
        rclpy.shutdown()


def test_bidirectional_follow_and_arbitration():
    ctx = _prepare('192')
    try:
        executor = ctx['executor']
        follower = ctx['follower']
        assert ctx['nav_client'].wait_for_service(timeout_sec=3.0)
        assert spin_until(
            executor,
            lambda: follower._mode_request_publisher
            .get_subscription_count() >= 1,
        )

        publish_scans(executor, ctx['scan_publisher'], ctx['test_node'], 1.50)
        assert follower._scan_is_fresh()

        unlocalized_target = PointStamped()
        unlocalized_target.header.stamp = ctx['test_node'].get_clock().now().to_msg()
        unlocalized_target.header.frame_id = 'missing_map_frame'
        unlocalized_target.point.x = 1.50
        ctx['target_publisher'].publish(unlocalized_target)
        assert spin_until_with_scan(
            executor,
            lambda: follower._state == FollowState.WAITING_FOR_LOCALIZATION,
            ctx['scan_publisher'],
            ctx['test_node'],
            1.50,
        )

        rejected_target = PointStamped()
        rejected_target.header.stamp = ctx['test_node'].get_clock().now().to_msg()
        rejected_target.header.frame_id = BASE_FRAME
        rejected_target.point.x = 2.50
        ctx['target_publisher'].publish(rejected_target)
        assert spin_until_with_scan(
            executor,
            lambda: follower._state == FollowState.TARGET_REJECTED,
            ctx['scan_publisher'],
            ctx['test_node'],
            1.50,
        )

        ctx['selection_publisher'].publish(
            make_selection(ctx['test_node'], 1.50, 0.0, 0.25)
        )
        assert spin_until_with_scan(
            executor,
            lambda: (
                follower._state == FollowState.FOLLOWING
                and not follower._awaiting_follow_mode
            ),
            ctx['scan_publisher'],
            ctx['test_node'],
            1.50,
        )

        ctx['outputs'].clear()
        publish_scans(executor, ctx['scan_publisher'], ctx['test_node'], 1.50)
        assert wait_for_velocity(
            executor,
            ctx['outputs'],
            lambda message: message.linear.x > 0.0,
            lambda: ctx['scan_publisher'].publish(
                make_scan(ctx['test_node'], 1.50)
            ),
        )
        assert ctx['outputs'][-1].linear.y == 0.0

        for distance in (1.25, 1.00, 0.80):
            ctx['outputs'].clear()
            publish_scans(
                executor, ctx['scan_publisher'], ctx['test_node'], distance
            )
        assert wait_for_velocity(
            executor,
            ctx['outputs'],
            lambda message: message.linear.x < 0.0,
            lambda: ctx['scan_publisher'].publish(
                make_scan(ctx['test_node'], 0.80)
            ),
        )
        assert ctx['outputs'][-1].linear.x >= -0.20
        assert ctx['outputs'][-1].linear.y == 0.0

        for distance in (0.55, 0.40, 0.30):
            ctx['outputs'].clear()
            publish_scans(
                executor, ctx['scan_publisher'], ctx['test_node'], distance
            )
        assert wait_for_velocity(
            executor,
            ctx['outputs'],
            lambda message: message.linear.x == pytest.approx(-0.20),
            lambda: ctx['scan_publisher'].publish(
                make_scan(ctx['test_node'], 0.30)
            ),
        )

        ctx['outputs'].clear()
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            ctx['scan_publisher'].publish(make_scan(ctx['test_node']))
            executor.spin_once(timeout_sec=0.05)
        assert follower._state == FollowState.TARGET_LOST
        assert ctx['outputs']
        assert ctx['outputs'][-1].linear.x == 0.0

        nav_command = Twist()
        nav_command.linear.x = 0.15
        response = call_trigger(executor, ctx['nav_client'])
        assert response.success, response.message
        ctx['outputs'].clear()
        for _ in range(5):
            ctx['scan_publisher'].publish(make_scan(ctx['test_node'], 0.80))
            ctx['nav_publisher'].publish(nav_command)
            executor.spin_once(timeout_sec=0.05)

        def keep_nav_case_fresh():
            ctx['scan_publisher'].publish(make_scan(ctx['test_node'], 0.80))
            ctx['nav_publisher'].publish(nav_command)

        assert wait_for_velocity(
            executor,
            ctx['outputs'],
            lambda message: message.linear.x == pytest.approx(0.15),
            keep_nav_case_fresh,
        )
    finally:
        _teardown(ctx)


def test_angular_follow_turns_toward_offset_target():
    ctx = _prepare('191')
    try:
        executor = ctx['executor']
        follower = ctx['follower']
        assert ctx['nav_client'].wait_for_service(timeout_sec=3.0)
        assert spin_until(
            executor,
            lambda: follower._mode_request_publisher
            .get_subscription_count() >= 1,
        )

        bearing = math.radians(30.0)
        target_x = 1.5 * math.cos(bearing)
        target_y = 1.5 * math.sin(bearing)
        publish_scans(
            executor,
            ctx['scan_publisher'],
            ctx['test_node'],
            1.50,
            bearing=bearing,
        )
        assert follower._scan_is_fresh()

        ctx['selection_publisher'].publish(
            make_selection(ctx['test_node'], target_x, target_y, 0.25)
        )
        assert spin_until_with_scan(
            executor,
            lambda: (
                follower._state == FollowState.FOLLOWING
                and not follower._awaiting_follow_mode
            ),
            ctx['scan_publisher'],
            ctx['test_node'],
            1.50,
            bearing=bearing,
        )

        ctx['outputs'].clear()
        assert wait_for_velocity(
            executor,
            ctx['outputs'],
            lambda message: (
                message.angular.z > 0.0
                and message.linear.x == 0.0
            ),
            lambda: ctx['scan_publisher'].publish(
                make_scan(ctx['test_node'], 1.50, bearing=bearing)
            ),
        )
    finally:
        _teardown(ctx)


def test_follow_distance_dynamic_update():
    if rclpy.ok():
        rclpy.shutdown()
    os.environ['ROS_DOMAIN_ID'] = '190'
    rclpy.init()

    follower = LidarTargetFollower(parameter_overrides=[
        Parameter('scan_topic', value=SCAN_TOPIC),
        Parameter('target_topic', value=TARGET_TOPIC),
        Parameter('selection_topic', value=SELECTION_TOPIC),
        Parameter('cmd_vel_out_topic', value=FOLLOW_TOPIC),
        Parameter('status_topic', value=f'{PREFIX}/follow/status'),
        Parameter(
            'tracked_target_topic',
            value=f'{PREFIX}/follow/tracked_target',
        ),
        Parameter('mode_request_topic', value=MODE_REQUEST_TOPIC),
        Parameter('mode_status_topic', value=MODE_STATUS_TOPIC),
        Parameter('start_service', value=f'{PREFIX}/follow/start'),
        Parameter('stop_service', value=f'{PREFIX}/follow/stop'),
        Parameter(
            'patrol_pause_service',
            value=f'{PREFIX}/missing_patrol_pause',
        ),
        Parameter('base_frame_id', value=BASE_FRAME),
    ])
    test_node = Node('follow_distance_update_test')
    executor = SingleThreadedExecutor()
    executor.add_node(follower)
    executor.add_node(test_node)

    client = test_node.create_client(
        SetParameters, '/lidar_target_follower/set_parameters'
    )

    try:
        assert follower._config.follow_distance == 1.0
        assert client.wait_for_service(timeout_sec=3.0)

        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                'follow_distance', Parameter.Type.DOUBLE, 1.5
            ).to_parameter_msg()
        ]
        future = client.call_async(request)
        assert spin_until(executor, future.done)
        result = future.result()
        assert all(item.successful for item in result.results)
        assert follower._config.follow_distance == pytest.approx(1.5)

        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                'follow_distance', Parameter.Type.DOUBLE, -1.0
            ).to_parameter_msg()
        ]
        future = client.call_async(request)
        assert spin_until(executor, future.done)
        result = future.result()
        assert not result.results[0].successful
        assert follower._config.follow_distance == pytest.approx(1.5)
    finally:
        executor.remove_node(follower)
        executor.remove_node(test_node)
        follower.destroy_node()
        test_node.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        if rclpy.ok():
            rclpy.shutdown()
