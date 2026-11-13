import base64
import json
import math
import os
from pathlib import Path
import socket
import struct
import time
from urllib.request import urlopen

from geometry_msgs.msg import PolygonStamped, Twist
import pytest
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, LaserScan

from robot_navigation_ros2.lidar_follow_web import LidarFollowWeb
from robot_navigation_ros2.process_supervisor import ProcessState


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PREFIX = '/follow_web_test'
BASE_FRAME = 'follow_web_test_base'


class FakeSystemSupervisor:

    def __init__(self):
        self.state = ProcessState.STOPPED
        self.pid = None
        self.last_error = ''
        self.start_calls = 0
        self.stop_calls = 0
        self.shutdown_calls = 0

    def start(self):
        self.start_calls += 1
        self.state = ProcessState.RUNNING
        self.pid = 12345
        return True, 'started'

    def stop(self):
        self.stop_calls += 1
        self.state = ProcessState.STOPPING
        return True, 'stopping'

    def shutdown(self):
        self.shutdown_calls += 1
        self.state = ProcessState.STOPPED
        self.pid = None


def spin_until(executor, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def read_exactly(connection, length):
    data = bytearray()
    while len(data) < length:
        chunk = connection.recv(length - len(data))
        assert chunk
        data.extend(chunk)
    return bytes(data)


def connect_websocket(port):
    connection = socket.create_connection(('127.0.0.1', port), timeout=3.0)
    key = base64.b64encode(b'follow-web-test-key').decode()
    request = (
        'GET /ws HTTP/1.1\r\n'
        f'Host: 127.0.0.1:{port}\r\n'
        'Upgrade: websocket\r\n'
        'Connection: Upgrade\r\n'
        f'Sec-WebSocket-Key: {key}\r\n'
        'Sec-WebSocket-Version: 13\r\n\r\n'
    )
    connection.sendall(request.encode())
    response = bytearray()
    while b'\r\n\r\n' not in response:
        response.extend(connection.recv(1024))
    assert b'101 Switching Protocols' in response
    return connection


def send_client_text(connection, data):
    payload = json.dumps(data).encode()
    mask = b'\x11\x22\x33\x44'
    header = bytearray((0x81,))
    if len(payload) < 126:
        header.append(0x80 | len(payload))
    else:
        header.append(0x80 | 126)
        header.extend(struct.pack('!H', len(payload)))
    masked = bytes(
        value ^ mask[index % 4]
        for index, value in enumerate(payload)
    )
    connection.sendall(header + mask + masked)


def receive_server_json(connection):
    header = read_exactly(connection, 2)
    assert header[0] & 0x0f == 0x01
    length = header[1] & 0x7f
    if length == 126:
        length = struct.unpack('!H', read_exactly(connection, 2))[0]
    elif length == 127:
        length = struct.unpack('!Q', read_exactly(connection, 8))[0]
    return json.loads(read_exactly(connection, length).decode())


def receive_message_type(connection, expected_type, limit=10):
    for _ in range(limit):
        message = receive_server_json(connection)
        if message.get('type') == expected_type:
            return message
    raise AssertionError(f'No {expected_type!r} WebSocket message received')


def make_scan(node):
    message = LaserScan()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = BASE_FRAME
    message.angle_min = -math.pi
    message.angle_max = math.pi
    message.angle_increment = math.pi / 180.0
    message.range_min = 0.05
    message.range_max = 10.0
    message.ranges = [math.inf] * 361
    message.ranges[180] = 1.50
    return message


def test_web_server_broadcasts_scan_and_publishes_circle_selection():
    if rclpy.ok():
        rclpy.shutdown()
    os.environ['ROS_DOMAIN_ID'] = '194'
    rclpy.init()

    web = LidarFollowWeb(parameter_overrides=[
        Parameter('scan_topic', value=f'{PREFIX}/scan'),
        Parameter('battery_topic', value=f'{PREFIX}/battery'),
        Parameter('follow_cmd_topic', value=f'{PREFIX}/follow_cmd'),
        Parameter('selection_topic', value=f'{PREFIX}/selection'),
        Parameter(
            'tracked_target_topic', value=f'{PREFIX}/tracked_target'
        ),
        Parameter('follow_status_topic', value=f'{PREFIX}/follow_status'),
        Parameter('motion_status_topic', value=f'{PREFIX}/motion_status'),
        Parameter('start_service', value=f'{PREFIX}/start'),
        Parameter('stop_service', value=f'{PREFIX}/stop'),
        Parameter('base_frame_id', value=BASE_FRAME),
        Parameter('web_bind_address', value='127.0.0.1'),
        Parameter('web_port', value=0),
        Parameter('web_root', value=str(PACKAGE_ROOT / 'web')),
    ])
    test_node = Node('follow_web_integration_test')
    executor = SingleThreadedExecutor()
    executor.add_node(web)
    executor.add_node(test_node)
    scans = test_node.create_publisher(
        LaserScan, f'{PREFIX}/scan', qos_profile_sensor_data
    )
    batteries = test_node.create_publisher(
        BatteryState, f'{PREFIX}/battery', qos_profile_sensor_data
    )
    follow_commands = test_node.create_publisher(
        Twist, f'{PREFIX}/follow_cmd', 10
    )
    selections = []
    test_node.create_subscription(
        PolygonStamped,
        f'{PREFIX}/selection',
        selections.append,
        10,
    )
    connection = None

    try:
        page = urlopen(
            f'http://127.0.0.1:{web.web_port}/', timeout=3.0
        ).read().decode()
        assert '激光目标圈选' in page
        connection = connect_websocket(web.web_port)
        connection.settimeout(3.0)
        assert spin_until(
            executor,
            lambda: (
                scans.get_subscription_count() >= 1
                and batteries.get_subscription_count() >= 1
            ),
        )

        battery = BatteryState()
        battery.voltage = 12.4
        battery.current = 0.8
        batteries.publish(battery)
        command = Twist()
        command.linear.x = -0.12
        command.angular.z = 0.3
        follow_commands.publish(command)
        assert spin_until(executor, lambda: web._battery is not None)
        assert spin_until(
            executor,
            lambda: web._follow_command['linear_x'] == -0.12,
        )

        scans.publish(make_scan(test_node))
        assert spin_until(executor, lambda: web._last_broadcast > 0.0)
        scan_data = receive_message_type(connection, 'scan_data')
        assert scan_data['points'] == [{'x': 1.5, 'y': 0.0}]
        assert scan_data['battery'] == {
            'voltage': 12.4,
            'current': 0.8,
        }
        assert scan_data['command'] == {
            'linear_x': -0.12,
            'angular_z': 0.3,
        }

        send_client_text(connection, {
            'type': 'select_region',
            'x': 1.50,
            'y': 0.10,
            'radius': 0.35,
        })
        assert spin_until(executor, lambda: bool(selections))
        polygon = selections[-1]
        assert polygon.header.frame_id == BASE_FRAME
        assert len(polygon.polygon.points) == 32
        center_x = sum(point.x for point in polygon.polygon.points) / 32
        center_y = sum(point.y for point in polygon.polygon.points) / 32
        assert center_x == pytest.approx(1.50)
        assert center_y == pytest.approx(0.10)
    finally:
        if connection is not None:
            connection.close()
        web.prepare_shutdown()
        executor.remove_node(web)
        executor.remove_node(test_node)
        web.destroy_node()
        test_node.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        if rclpy.ok():
            rclpy.shutdown()


def test_web_bridge_has_no_direct_velocity_output():
    source = (
        PACKAGE_ROOT / 'robot_navigation_ros2' / 'lidar_follow_web.py'
    ).read_text(encoding='utf-8')

    assert 'create_publisher(\n            Twist' not in source
    assert 'create_publisher(Twist' not in source
    assert "get_parameter('cmd_vel_out_topic')" not in source


def test_web_commands_start_and_stop_robot_system():
    if rclpy.ok():
        rclpy.shutdown()
    os.environ['ROS_DOMAIN_ID'] = '195'
    rclpy.init()

    supervisor = FakeSystemSupervisor()
    web = LidarFollowWeb(
        parameter_overrides=[
            Parameter('scan_topic', value=f'{PREFIX}/scan'),
            Parameter('selection_topic', value=f'{PREFIX}/selection'),
            Parameter(
                'tracked_target_topic', value=f'{PREFIX}/tracked_target'
            ),
            Parameter('follow_status_topic', value=f'{PREFIX}/follow_status'),
            Parameter('motion_status_topic', value=f'{PREFIX}/motion_status'),
            Parameter('start_service', value=f'{PREFIX}/start'),
            Parameter('stop_service', value=f'{PREFIX}/stop'),
            Parameter('base_frame_id', value=BASE_FRAME),
            Parameter('web_bind_address', value='127.0.0.1'),
            Parameter('web_port', value=0),
            Parameter('web_root', value=str(PACKAGE_ROOT / 'web')),
            Parameter('system_control_enabled', value=True),
        ],
        process_supervisor=supervisor,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(web)
    connection = None

    try:
        connection = connect_websocket(web.web_port)
        connection.settimeout(3.0)

        send_client_text(connection, {'type': 'system_start'})
        assert spin_until(executor, lambda: supervisor.start_calls == 1)
        assert supervisor.state == ProcessState.RUNNING

        send_client_text(connection, {'type': 'system_start'})
        assert spin_until(executor, lambda: supervisor.start_calls == 2)

        send_client_text(connection, {'type': 'system_stop'})
        assert spin_until(executor, lambda: supervisor.stop_calls == 1)
        assert supervisor.state == ProcessState.STOPPING
    finally:
        if connection is not None:
            connection.close()
        web.prepare_shutdown()
        assert supervisor.shutdown_calls == 1
        executor.remove_node(web)
        web.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        if rclpy.ok():
            rclpy.shutdown()


def test_web_set_distance_calls_follower_parameter_service():
    if rclpy.ok():
        rclpy.shutdown()
    os.environ['ROS_DOMAIN_ID'] = '196'
    rclpy.init()

    service_name = f'{PREFIX}/set_distance'
    recorded = []

    web = LidarFollowWeb(parameter_overrides=[
        Parameter('scan_topic', value=f'{PREFIX}/scan'),
        Parameter('battery_topic', value=f'{PREFIX}/battery'),
        Parameter('follow_cmd_topic', value=f'{PREFIX}/follow_cmd'),
        Parameter('selection_topic', value=f'{PREFIX}/selection'),
        Parameter(
            'tracked_target_topic', value=f'{PREFIX}/tracked_target'
        ),
        Parameter('follow_status_topic', value=f'{PREFIX}/follow_status'),
        Parameter('motion_status_topic', value=f'{PREFIX}/motion_status'),
        Parameter('start_service', value=f'{PREFIX}/start'),
        Parameter('stop_service', value=f'{PREFIX}/stop'),
        Parameter('base_frame_id', value=BASE_FRAME),
        Parameter('web_bind_address', value='127.0.0.1'),
        Parameter('web_port', value=0),
        Parameter('web_root', value=str(PACKAGE_ROOT / 'web')),
        Parameter('follow_distance_param_service', value=service_name),
        Parameter('follow_distance_param_name', value='follow_distance'),
    ])
    test_node = Node('follow_web_distance_test')

    def handle(request, response):
        for parameter in request.parameters:
            recorded.append(
                (parameter.name, parameter.value.double_value)
            )
        response.results = []
        for parameter in request.parameters:
            result = SetParametersResult()
            result.successful = True
            result.reason = ''
            response.results.append(result)
        return response

    test_node.create_service(SetParameters, service_name, handle)

    executor = SingleThreadedExecutor()
    executor.add_node(web)
    executor.add_node(test_node)

    connection = None
    try:
        assert spin_until(
            executor, lambda: web._distance_client.service_is_ready()
        )
        connection = connect_websocket(web.web_port)
        connection.settimeout(3.0)
        send_client_text(connection, {
            'type': 'set_distance',
            'distance': 1.6,
        })
        assert spin_until(executor, lambda: bool(recorded))
        assert recorded[-1] == ('follow_distance', 1.6)
    finally:
        if connection is not None:
            connection.close()
        web.prepare_shutdown()
        executor.remove_node(web)
        executor.remove_node(test_node)
        web.destroy_node()
        test_node.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        if rclpy.ok():
            rclpy.shutdown()
