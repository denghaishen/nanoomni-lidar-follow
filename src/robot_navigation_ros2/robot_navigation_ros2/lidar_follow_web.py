#!/usr/bin/env python3

import base64
from functools import partial
import hashlib
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from queue import Empty, Full, Queue
import socket
import struct
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped, Twist
import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from robot_navigation_ros2.follow_core import (
    Transform2D,
    quaternion_to_yaw,
    scan_to_points,
)
from robot_navigation_ros2.process_supervisor import ProcessSupervisor


_WEBSOCKET_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'


class WebSocketClient:

    def __init__(self, connection: socket.socket) -> None:
        self.connection = connection
        self.connection.setsockopt(
            socket.IPPROTO_TCP,
            socket.TCP_NODELAY,
            1,
        )
        self.lock = threading.Lock()
        self._queue = Queue(maxsize=8)
        self._latest_scan = None
        self._latest_scan_lock = threading.Lock()
        self._wake = threading.Event()
        self._closed = threading.Event()
        self._writer = threading.Thread(
            target=self._write_loop,
            daemon=True,
        )
        self._writer.start()

    def send_text(self, text: str, replaceable: bool = False) -> None:
        if self._closed.is_set():
            raise OSError('WebSocket client is closed')
        frame = self._text_frame(text)
        if replaceable:
            with self._latest_scan_lock:
                self._latest_scan = frame
            self._wake.set()
            return
        try:
            self._queue.put_nowait(frame)
        except Full as exc:
            raise OSError('WebSocket client is too slow') from exc
        self._wake.set()

    def send_pong(self, payload: bytes) -> None:
        if len(payload) > 125 or self._closed.is_set():
            return
        with self.lock:
            self.connection.sendall(bytes((0x8a, len(payload))) + payload)

    def close(self) -> None:
        self._closed.set()
        self._wake.set()
        try:
            self.connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.connection.close()
        except OSError:
            pass
        if threading.current_thread() is not self._writer:
            self._writer.join(timeout=1.0)

    @staticmethod
    def _text_frame(text: str) -> bytes:
        payload = text.encode('utf-8')
        header = bytearray((0x81,))
        if len(payload) < 126:
            header.append(len(payload))
        elif len(payload) <= 0xffff:
            header.append(126)
            header.extend(struct.pack('!H', len(payload)))
        else:
            header.append(127)
            header.extend(struct.pack('!Q', len(payload)))
        return bytes(header) + payload

    def _write_loop(self) -> None:
        while not self._closed.is_set():
            frame = None
            try:
                frame = self._queue.get_nowait()
            except Empty:
                with self._latest_scan_lock:
                    frame = self._latest_scan
                    self._latest_scan = None
            if frame is None:
                self._wake.wait(timeout=0.5)
                self._wake.clear()
                continue
            try:
                with self.lock:
                    self.connection.sendall(frame)
            except OSError:
                self.close()
                return


class FollowWebHandler(SimpleHTTPRequestHandler):

    def __init__(self, *args, bridge, **kwargs) -> None:
        self._bridge = bridge
        super().__init__(*args, **kwargs)

    def do_GET(self) -> None:
        if (
            self.path.split('?', 1)[0] == '/ws'
            and self.headers.get('Upgrade', '').lower() == 'websocket'
        ):
            self._serve_websocket()
            return
        super().do_GET()

    def end_headers(self) -> None:
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; connect-src 'self' ws:")
        super().end_headers()

    def log_message(self, format_string, *args) -> None:
        del format_string, args

    def _serve_websocket(self) -> None:
        key = self.headers.get('Sec-WebSocket-Key', '')
        if not key or self.headers.get('Sec-WebSocket-Version') != '13':
            self.send_error(400, 'Invalid WebSocket handshake')
            return
        accept = base64.b64encode(
            hashlib.sha1((key + _WEBSOCKET_GUID).encode()).digest()
        ).decode()
        self.send_response(101, 'Switching Protocols')
        self.send_header('Upgrade', 'websocket')
        self.send_header('Connection', 'Upgrade')
        self.send_header('Sec-WebSocket-Accept', accept)
        super().end_headers()

        client = WebSocketClient(self.connection)
        self._bridge.register_client(client)
        try:
            while self._bridge.server_is_running:
                opcode, payload = self._read_frame()
                if opcode is None or opcode == 0x8:
                    break
                if opcode == 0x9:
                    client.send_pong(payload)
                elif opcode == 0x1:
                    self._bridge.enqueue_command(payload.decode('utf-8'))
        except (ConnectionError, OSError, UnicodeDecodeError):
            pass
        finally:
            self._bridge.unregister_client(client)

    def _read_frame(self) -> tuple[int | None, bytes]:
        header = self._read_exactly(2)
        if header is None:
            return None, b''
        final = bool(header[0] & 0x80)
        opcode = header[0] & 0x0f
        masked = bool(header[1] & 0x80)
        length = header[1] & 0x7f
        if not final or not masked:
            raise ConnectionError('Unsupported WebSocket frame')
        if length == 126:
            extended = self._read_exactly(2)
            if extended is None:
                return None, b''
            length = struct.unpack('!H', extended)[0]
        elif length == 127:
            extended = self._read_exactly(8)
            if extended is None:
                return None, b''
            length = struct.unpack('!Q', extended)[0]
        if length > 65536:
            raise ConnectionError('WebSocket message is too large')
        mask = self._read_exactly(4)
        payload = self._read_exactly(length)
        if mask is None or payload is None:
            return None, b''
        return opcode, bytes(
            value ^ mask[index % 4]
            for index, value in enumerate(payload)
        )

    def _read_exactly(self, length: int) -> bytes | None:
        data = bytearray()
        while len(data) < length:
            chunk = self.connection.recv(length - len(data))
            if not chunk:
                return None
            data.extend(chunk)
        return bytes(data)


class LidarFollowWeb(Node):

    def __init__(
        self,
        parameter_overrides=None,
        process_supervisor=None,
    ) -> None:
        super().__init__(
            'lidar_follow_web',
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('battery_topic', '/battery')
        self.declare_parameter('selection_topic', '/follow/selection')
        self.declare_parameter(
            'tracked_target_topic', '/follow/tracked_target'
        )
        self.declare_parameter('follow_command_topic', '/cmd_vel_follow')
        self.declare_parameter('follow_status_topic', '/follow/status')
        self.declare_parameter('motion_status_topic', '/motion_mode/status')
        self.declare_parameter('follow_cmd_topic', '/cmd_vel_follow')
        self.declare_parameter('start_service', '/follow/start')
        self.declare_parameter('stop_service', '/follow/stop')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('transform_timeout', 0.1)
        self.declare_parameter('broadcast_frequency', 8.0)
        self.declare_parameter('max_points', 240)
        self.declare_parameter('selection_min_radius', 0.10)
        self.declare_parameter('selection_max_radius', 0.80)
        self.declare_parameter(
            'follow_distance_param_service',
            '/lidar_target_follower/set_parameters',
        )
        self.declare_parameter('follow_distance_param_name', 'follow_distance')
        self.declare_parameter('distance_min', 0.40)
        self.declare_parameter('distance_max', 2.00)
        self.declare_parameter('initial_follow_distance', 1.0)
        self.declare_parameter('system_control_enabled', False)
        self.declare_parameter('system_ros2_executable', '/opt/ros/humble/bin/ros2')
        self.declare_parameter('system_launch_package', 'robot_navigation_ros2')
        self.declare_parameter('system_launch_file', 'slam_nav.launch.py')
        self.declare_parameter('system_launch_use_rviz', False)
        self.declare_parameter('system_launch_start_web', False)
        self.declare_parameter('system_stop_grace_seconds', 20.0)
        self.declare_parameter('web_bind_address', '0.0.0.0')
        self.declare_parameter('web_port', 8080)
        self.declare_parameter('web_root', '')

        self._base_frame_id = str(
            self.get_parameter('base_frame_id').value
        )
        self._transform_timeout = float(
            self.get_parameter('transform_timeout').value
        )
        broadcast_frequency = float(
            self.get_parameter('broadcast_frequency').value
        )
        self._max_points = int(self.get_parameter('max_points').value)
        self._selection_min_radius = float(
            self.get_parameter('selection_min_radius').value
        )
        self._selection_max_radius = float(
            self.get_parameter('selection_max_radius').value
        )
        if self._transform_timeout < 0.0:
            raise ValueError('transform_timeout must be nonnegative')
        if broadcast_frequency <= 0.0:
            raise ValueError('broadcast_frequency must be positive')
        if self._max_points < 1:
            raise ValueError('max_points must be positive')
        if self._selection_min_radius <= 0.0:
            raise ValueError('selection_min_radius must be positive')
        if self._selection_max_radius < self._selection_min_radius:
            raise ValueError('selection radius limits are invalid')
        self._follow_distance_param_service = str(
            self.get_parameter('follow_distance_param_service').value
        )
        self._follow_distance_param_name = str(
            self.get_parameter('follow_distance_param_name').value
        )
        self._distance_min = float(
            self.get_parameter('distance_min').value
        )
        self._distance_max = float(
            self.get_parameter('distance_max').value
        )
        self._follow_distance = float(
            self.get_parameter('initial_follow_distance').value
        )
        if self._distance_min <= 0.0:
            raise ValueError('distance_min must be positive')
        if self._distance_max < self._distance_min:
            raise ValueError('distance_max must not be smaller than distance_min')
        self._system_control_enabled = bool(
            self.get_parameter('system_control_enabled').value
        )
        self._system_supervisor = process_supervisor
        if self._system_control_enabled and self._system_supervisor is None:
            ros2_executable = str(
                self.get_parameter('system_ros2_executable').value
            )
            if not os.path.isfile(ros2_executable):
                raise ValueError(
                    f'system_ros2_executable does not exist: {ros2_executable}'
                )
            launch_package = str(
                self.get_parameter('system_launch_package').value
            )
            launch_file = str(
                self.get_parameter('system_launch_file').value
            )
            use_rviz = bool(
                self.get_parameter('system_launch_use_rviz').value
            )
            start_web = bool(
                self.get_parameter('system_launch_start_web').value
            )
            stop_grace = float(
                self.get_parameter('system_stop_grace_seconds').value
            )
            self._system_supervisor = ProcessSupervisor(
                [
                    ros2_executable,
                    'launch',
                    launch_package,
                    launch_file,
                    f'use_rviz:={str(use_rviz).lower()}',
                    f'start_web:={str(start_web).lower()}',
                ],
                stop_grace_seconds=stop_grace,
            )
        self._cached_system_state = self._read_system_state()
        self._broadcast_period = 1.0 / broadcast_frequency
        self._last_broadcast = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._selection_publisher = self.create_publisher(
            PolygonStamped,
            str(self.get_parameter('selection_topic').value),
            10,
        )
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            BatteryState,
            str(self.get_parameter('battery_topic').value),
            self._battery_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('follow_cmd_topic').value),
            self._follow_command_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter('tracked_target_topic').value),
            self._target_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('follow_status_topic').value),
            lambda message: self._set_status('follow', message.data),
            status_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('motion_status_topic').value),
            lambda message: self._set_status('motion', message.data),
            status_qos,
        )
        self._start_client = self.create_client(
            Trigger, str(self.get_parameter('start_service').value)
        )
        self._stop_client = self.create_client(
            Trigger, str(self.get_parameter('stop_service').value)
        )
        self._distance_client = self.create_client(
            SetParameters, self._follow_distance_param_service
        )

        self._status = {
            'follow': 'IDLE',
            'motion': 'NAV',
        }
        self._tracked_target = None
        self._follow_command = {
            'linear_x': 0.0,
            'angular_z': 0.0,
        }
        self._battery = None
        self._websocket_clients = set()
        self._websocket_clients_lock = threading.Lock()
        self._command_queue = Queue()
        self._server_running = True

        web_root = str(self.get_parameter('web_root').value)
        if not web_root:
            web_root = os.path.join(
                get_package_share_directory('robot_navigation_ros2'),
                'web',
            )
        if not os.path.isfile(os.path.join(web_root, 'index.html')):
            raise ValueError(f'web_root has no index.html: {web_root}')
        bind_address = str(
            self.get_parameter('web_bind_address').value
        )
        port = int(self.get_parameter('web_port').value)
        handler = partial(
            FollowWebHandler,
            directory=web_root,
            bridge=self,
        )
        self._http_server = ThreadingHTTPServer(
            (bind_address, port), handler
        )
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            daemon=True,
        )
        self._http_thread.start()
        self.web_port = self._http_server.server_address[1]
        self.create_timer(0.05, self._process_commands)
        self.create_timer(0.5, self._system_tick)
        self.get_logger().info(
            f'Lidar follow selector: http://{bind_address}:{self.web_port}'
        )
        if bind_address != '127.0.0.1':
            self.get_logger().warning(
                'Web selector is reachable from the network and has no login'
            )

    @property
    def server_is_running(self) -> bool:
        return self._server_running

    def register_client(self, client: WebSocketClient) -> None:
        with self._websocket_clients_lock:
            self._websocket_clients.add(client)
        try:
            client.send_text(json.dumps(
                self._cached_system_state,
                separators=(',', ':'),
            ))
        except OSError:
            self.unregister_client(client)

    def unregister_client(self, client: WebSocketClient) -> None:
        with self._websocket_clients_lock:
            self._websocket_clients.discard(client)
        client.close()

    def enqueue_command(self, message: str) -> None:
        self._command_queue.put(message)

    def _process_commands(self) -> None:
        while True:
            try:
                raw_message = self._command_queue.get_nowait()
            except Empty:
                return
            try:
                command = json.loads(raw_message)
            except (json.JSONDecodeError, TypeError):
                self._broadcast_error('Invalid command JSON')
                continue
            command_type = command.get('type')
            if command_type == 'select_region':
                self._publish_selection(command)
            elif command_type == 'set_distance':
                self._set_follow_distance(command)
            elif command_type == 'system_start':
                self._start_robot_system()
            elif command_type == 'system_stop':
                self._stop_robot_system()
            elif command_type == 'stop':
                self._call_trigger(self._stop_client, 'stop service unavailable')
            elif command_type == 'start':
                self._call_trigger(
                    self._start_client, 'start service unavailable'
                )
            else:
                self._broadcast_error('Unsupported command')

    def _start_robot_system(self) -> None:
        if not self._system_control_enabled or self._system_supervisor is None:
            self._broadcast_error('System control is unavailable')
            return
        success, message = self._system_supervisor.start()
        if not success:
            self._broadcast_error(message)
        else:
            self.get_logger().info(message)
        self._system_tick()

    def _stop_robot_system(self) -> None:
        if not self._system_control_enabled or self._system_supervisor is None:
            self._broadcast_error('System control is unavailable')
            return
        success, message = self._system_supervisor.stop()
        if not success:
            self._broadcast_error(message)
        else:
            self.get_logger().info(message)
        self._system_tick()

    def _read_system_state(self) -> dict:
        if not self._system_control_enabled or self._system_supervisor is None:
            return {
                'type': 'system_state',
                'enabled': False,
                'state': 'UNAVAILABLE',
                'pid': None,
                'error': '',
            }
        return {
            'type': 'system_state',
            'enabled': True,
            'state': self._system_supervisor.state.value,
            'pid': self._system_supervisor.pid,
            'error': self._system_supervisor.last_error,
        }

    def _system_tick(self) -> None:
        state = self._read_system_state()
        if state == self._cached_system_state:
            return
        self._cached_system_state = state
        self._broadcast_json(state)

    def _publish_selection(self, command: dict) -> None:
        try:
            center_x = float(command['x'])
            center_y = float(command['y'])
            radius = float(command['radius'])
        except (KeyError, TypeError, ValueError):
            self._broadcast_error('Selection requires x, y and radius')
            return
        if not all(math.isfinite(value) for value in (
            center_x, center_y, radius
        )):
            self._broadcast_error('Selection values must be finite')
            return
        radius = max(
            self._selection_min_radius,
            min(radius, self._selection_max_radius),
        )
        if not 0.20 <= center_x <= 3.50 or abs(center_y) > 2.0:
            self._broadcast_error('Select a region in front of the robot')
            return

        message = PolygonStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._base_frame_id
        for index in range(32):
            angle = 2.0 * math.pi * index / 32
            point = Point32()
            point.x = center_x + radius * math.cos(angle)
            point.y = center_y + radius * math.sin(angle)
            message.polygon.points.append(point)
        self._selection_publisher.publish(message)
        self._broadcast_json({
            'type': 'selection',
            'x': center_x,
            'y': center_y,
            'radius': radius,
        })

    def _call_trigger(self, client, unavailable_message: str) -> None:
        if not client.service_is_ready():
            self._broadcast_error(unavailable_message)
            return
        client.call_async(Trigger.Request())

    def _set_follow_distance(self, command: dict) -> None:
        try:
            distance = float(command['distance'])
        except (KeyError, TypeError, ValueError):
            self._broadcast_error('Distance requires a numeric value')
            return
        if not math.isfinite(distance):
            self._broadcast_error('Distance must be finite')
            return
        distance = max(self._distance_min, min(distance, self._distance_max))
        self._follow_distance = distance
        if not self._distance_client.service_is_ready():
            self._broadcast_error('Follow distance service is unavailable')
            return
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                self._follow_distance_param_name,
                Parameter.Type.DOUBLE,
                distance,
            ).to_parameter_msg()
        ]
        self._distance_client.call_async(request)
        self._broadcast_json({'type': 'distance', 'distance': distance})

    def _scan_callback(self, message: LaserScan) -> None:
        now = time.monotonic()
        if now - self._last_broadcast < self._broadcast_period:
            return
        try:
            transform = self._scan_transform(message)
        except TransformException:
            return
        points = scan_to_points(
            message.ranges,
            message.angle_min,
            message.angle_increment,
            message.range_min,
            message.range_max,
            transform,
        )
        step = max(1, math.ceil(len(points) / self._max_points))
        sampled = points[::step][:self._max_points]
        self._last_broadcast = now
        self._broadcast_json({
            'type': 'scan_data',
            'points': [
                {'x': round(point.x, 3), 'y': round(point.y, 3)}
                for point in sampled
            ],
            'target': self._tracked_target,
            'command': self._follow_command,
            'battery': self._battery,
            'status': dict(self._status),
            'distance': self._follow_distance,
            'distance_limits': {
                'min': self._distance_min,
                'max': self._distance_max,
            },
            'selection_limits': {
                'min_radius': self._selection_min_radius,
                'max_radius': self._selection_max_radius,
            },
        })

    def _scan_transform(self, message: LaserScan) -> Transform2D:
        if not message.header.frame_id:
            raise TransformException('LaserScan frame_id is empty')
        if message.header.frame_id == self._base_frame_id:
            return Transform2D()
        transform = self._tf_buffer.lookup_transform(
            self._base_frame_id,
            message.header.frame_id,
            Time.from_msg(message.header.stamp),
            timeout=Duration(seconds=self._transform_timeout),
        ).transform
        return Transform2D(
            x=transform.translation.x,
            y=transform.translation.y,
            yaw=quaternion_to_yaw(
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ),
        )

    def _follow_command_callback(self, message: Twist) -> None:
        self._follow_command = {
            'linear_x': round(message.linear.x, 3),
            'angular_z': round(message.angular.z, 3),
        }

    def _battery_callback(self, message: BatteryState) -> None:
        if not math.isfinite(message.voltage) or message.voltage <= 0.0:
            return
        self._battery = {
            'voltage': round(message.voltage, 2),
            'current': (
                round(message.current, 2)
                if math.isfinite(message.current)
                else None
            ),
        }

    def _target_callback(self, message: PointStamped) -> None:
        self._tracked_target = {
            'x': round(message.point.x, 3),
            'y': round(message.point.y, 3),
        }

    def _set_status(self, name: str, value: str) -> None:
        self._status[name] = value

    def _broadcast_error(self, message: str) -> None:
        self._broadcast_json({'type': 'error', 'message': message})

    def _broadcast_json(self, data: dict) -> None:
        payload = json.dumps(data, separators=(',', ':'))
        dead_clients = []
        with self._websocket_clients_lock:
            clients = tuple(self._websocket_clients)
        for client in clients:
            try:
                client.send_text(
                    payload,
                    replaceable=data.get('type') == 'scan_data',
                )
            except OSError:
                dead_clients.append(client)
        for client in dead_clients:
            self.unregister_client(client)

    def prepare_shutdown(self) -> None:
        if self._system_supervisor is not None:
            self._system_supervisor.shutdown()
        self._server_running = False
        self._http_server.shutdown()
        self._http_server.server_close()
        with self._websocket_clients_lock:
            clients = tuple(self._websocket_clients)
            self._websocket_clients.clear()
        for client in clients:
            client.close()
        self._http_thread.join(timeout=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarFollowWeb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.prepare_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
