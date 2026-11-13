#!/usr/bin/env python3

from dataclasses import replace
from enum import Enum
import math
import time

from geometry_msgs.msg import PointStamped, PolygonStamped, Twist
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from robot_navigation_ros2.follow_core import (
    FollowConfig,
    Point2D,
    Transform2D,
    compute_follow_command,
    find_target_centroid,
    quaternion_to_yaw,
    scan_to_points,
    target_is_selectable,
)


class FollowState(str, Enum):
    IDLE = 'IDLE'
    WAITING_FOR_SCAN = 'WAITING_FOR_SCAN'
    WAITING_FOR_LOCALIZATION = 'WAITING_FOR_LOCALIZATION'
    TARGET_REJECTED = 'TARGET_REJECTED'
    TARGET_SELECTED = 'TARGET_SELECTED'
    FOLLOWING = 'FOLLOWING'
    TARGET_LOST = 'TARGET_LOST'
    STOPPED = 'STOPPED'
    ERROR = 'ERROR'


class LidarTargetFollower(Node):

    def __init__(self, parameter_overrides=None) -> None:
        super().__init__(
            'lidar_target_follower',
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('target_topic', '/follow/target')
        self.declare_parameter('selection_topic', '/follow/selection')
        self.declare_parameter('cmd_vel_out_topic', '/cmd_vel_follow')
        self.declare_parameter('status_topic', '/follow/status')
        self.declare_parameter(
            'tracked_target_topic', '/follow/tracked_target'
        )
        self.declare_parameter('mode_request_topic', '/motion_mode/request')
        self.declare_parameter('mode_status_topic', '/motion_mode/status')
        self.declare_parameter('start_service', '/follow/start')
        self.declare_parameter('stop_service', '/follow/stop')
        self.declare_parameter('patrol_pause_service', '/patrol/pause')
        self.declare_parameter('pause_patrol_on_start', True)
        self.declare_parameter('start_on_target_selection', False)
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('follow_distance', 1.0)
        self.declare_parameter('distance_deadband', 0.05)
        self.declare_parameter('target_radius', 0.30)
        self.declare_parameter('selection_min_radius', 0.10)
        self.declare_parameter('selection_max_radius', 0.80)
        self.declare_parameter('min_target_points', 1)
        self.declare_parameter('scan_timeout', 0.5)
        self.declare_parameter('target_loss_timeout', 0.5)
        self.declare_parameter('motion_mode_timeout', 0.5)
        self.declare_parameter('transform_timeout', 0.1)
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('angular_gain', 1.0)
        self.declare_parameter('max_forward_speed', 0.30)
        self.declare_parameter('max_reverse_speed', 0.20)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('allow_reverse', True)
        self.declare_parameter('use_lateral_velocity', False)
        self.declare_parameter('rotate_first_angle', 0.30)
        self.declare_parameter('angular_deadband', 0.10)
        self.declare_parameter('target_min_x', 0.30)
        self.declare_parameter('target_max_x', 3.00)
        self.declare_parameter('target_max_abs_y', 1.50)

        self._config = FollowConfig(
            follow_distance=float(
                self.get_parameter('follow_distance').value
            ),
            distance_deadband=float(
                self.get_parameter('distance_deadband').value
            ),
            target_radius=float(
                self.get_parameter('target_radius').value
            ),
            min_target_points=int(
                self.get_parameter('min_target_points').value
            ),
            linear_gain=float(self.get_parameter('linear_gain').value),
            angular_gain=float(self.get_parameter('angular_gain').value),
            max_forward_speed=float(
                self.get_parameter('max_forward_speed').value
            ),
            max_reverse_speed=float(
                self.get_parameter('max_reverse_speed').value
            ),
            max_angular_speed=float(
                self.get_parameter('max_angular_speed').value
            ),
            allow_reverse=bool(
                self.get_parameter('allow_reverse').value
            ),
            use_lateral_velocity=bool(
                self.get_parameter('use_lateral_velocity').value
            ),
            rotate_first_angle=float(
                self.get_parameter('rotate_first_angle').value
            ),
            angular_deadband=float(
                self.get_parameter('angular_deadband').value
            ),
            target_min_x=float(
                self.get_parameter('target_min_x').value
            ),
            target_max_x=float(
                self.get_parameter('target_max_x').value
            ),
            target_max_abs_y=float(
                self.get_parameter('target_max_abs_y').value
            ),
        )
        self._base_frame_id = str(
            self.get_parameter('base_frame_id').value
        )
        self._selection_min_radius = float(
            self.get_parameter('selection_min_radius').value
        )
        self._selection_max_radius = float(
            self.get_parameter('selection_max_radius').value
        )
        self._scan_timeout = float(
            self.get_parameter('scan_timeout').value
        )
        self._target_loss_timeout = float(
            self.get_parameter('target_loss_timeout').value
        )
        self._motion_mode_timeout = float(
            self.get_parameter('motion_mode_timeout').value
        )
        self._transform_timeout = float(
            self.get_parameter('transform_timeout').value
        )
        control_frequency = float(
            self.get_parameter('control_frequency').value
        )
        if self._selection_min_radius <= 0.0:
            raise ValueError('selection_min_radius must be positive')
        if self._selection_max_radius < self._selection_min_radius:
            raise ValueError(
                'selection_max_radius must not be smaller than minimum'
            )
        if self._scan_timeout <= 0.0:
            raise ValueError('scan_timeout must be positive')
        if self._target_loss_timeout <= 0.0:
            raise ValueError('target_loss_timeout must be positive')
        if self._motion_mode_timeout <= 0.0:
            raise ValueError('motion_mode_timeout must be positive')
        if self._transform_timeout < 0.0:
            raise ValueError('transform_timeout must be nonnegative')
        if control_frequency <= 0.0:
            raise ValueError('control_frequency must be positive')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._state = FollowState.IDLE
        self._target: Point2D | None = None
        self._latest_points: tuple[Point2D, ...] = ()
        self._last_valid_scan: float | None = None
        self._loss_started_at: float | None = None
        self._moving = False
        self._awaiting_follow_mode = False
        self._follow_mode_requested_at: float | None = None
        self._last_output_was_nonzero = False

        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._command_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter('cmd_vel_out_topic').value),
            10,
        )
        self._status_publisher = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            status_qos,
        )
        self._tracked_target_publisher = self.create_publisher(
            PointStamped,
            str(self.get_parameter('tracked_target_topic').value),
            10,
        )
        self._mode_request_publisher = self.create_publisher(
            String,
            str(self.get_parameter('mode_request_topic').value),
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter('target_topic').value),
            self._target_callback,
            10,
        )
        self.create_subscription(
            PolygonStamped,
            str(self.get_parameter('selection_topic').value),
            self._selection_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('mode_status_topic').value),
            self._mode_status_callback,
            status_qos,
        )
        self.create_service(
            Trigger,
            str(self.get_parameter('start_service').value),
            self._start_callback,
        )
        self.create_service(
            Trigger,
            str(self.get_parameter('stop_service').value),
            self._stop_callback,
        )
        self._patrol_pause_client = self.create_client(
            Trigger,
            str(self.get_parameter('patrol_pause_service').value),
        )
        self._pause_patrol_on_start = bool(
            self.get_parameter('pause_patrol_on_start').value
        )
        self._start_on_target_selection = bool(
            self.get_parameter('start_on_target_selection').value
        )
        self.add_on_set_parameters_callback(self._on_parameter_update)
        self.create_timer(1.0 / control_frequency, self._control_tick)
        self._publish_state()
        self._publish_stop()

    def _on_parameter_update(self, parameters):
        for parameter in parameters:
            if parameter.name != 'follow_distance':
                continue
            value = parameter.value
            if (
                not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0.0
            ):
                return SetParametersResult(
                    successful=False,
                    reason='follow_distance must be a positive number',
                )
            self._config = replace(
                self._config, follow_distance=float(value)
            )
            self.get_logger().info(
                f'Follow distance set to {float(value):.2f} m'
            )
        return SetParametersResult(successful=True)

    def _scan_callback(self, message: LaserScan) -> None:
        if not self._stamp_is_fresh(message):
            self._tracking_missed('LaserScan timestamp is stale')
            return
        try:
            transform = self._scan_transform(message)
        except TransformException as exc:
            self._tracking_missed(f'LaserScan TF unavailable: {exc}')
            return

        points = scan_to_points(
            message.ranges,
            message.angle_min,
            message.angle_increment,
            message.range_min,
            message.range_max,
            transform,
        )
        self._latest_points = points
        self._last_valid_scan = time.monotonic()
        if self._target is None:
            return

        target, count = find_target_centroid(
            points,
            self._target,
            self._config.target_radius,
            self._config.min_target_points,
        )
        if target is None or target.x <= 0.0:
            self._tracking_missed(
                f'only {count} target points found in current scan'
            )
            return

        self._target = target
        self._loss_started_at = None
        self._publish_tracked_target()

    def _target_callback(self, message: PointStamped) -> None:
        if not self._selection_is_ready():
            return
        try:
            selected_point = self._point_in_base_frame(message)
        except TransformException as exc:
            self._reject_for_missing_transform(exc)
            return
        self._select_target(selected_point, self._config.target_radius)

    def _selection_callback(self, message: PolygonStamped) -> None:
        if not self._selection_is_ready():
            return
        if len(message.polygon.points) < 3:
            self._reject_target(
                'Rejected follow region: circle requires at least 3 points'
            )
            return

        center_x = sum(
            point.x for point in message.polygon.points
        ) / len(message.polygon.points)
        center_y = sum(
            point.y for point in message.polygon.points
        ) / len(message.polygon.points)
        radius = max(
            ((point.x - center_x) ** 2 + (point.y - center_y) ** 2)
            ** 0.5
            for point in message.polygon.points
        )
        radius = max(
            self._selection_min_radius,
            min(radius, self._selection_max_radius),
        )

        center = PointStamped()
        center.header = message.header
        center.point.x = center_x
        center.point.y = center_y
        try:
            selected_point = self._point_in_base_frame(center)
        except TransformException as exc:
            self._reject_for_missing_transform(exc)
            return
        self._select_target(selected_point, radius)

    def _selection_is_ready(self) -> bool:
        if self._scan_is_fresh():
            return True
        if self._target is None and not self._moving:
            self._state = FollowState.WAITING_FOR_SCAN
            self._publish_state()
        self.get_logger().warning(
            'Rejected follow target because LaserScan is not fresh'
        )
        return False

    def _reject_for_missing_transform(self, error: Exception) -> None:
        if self._target is None and not self._moving:
            self._state = FollowState.WAITING_FOR_LOCALIZATION
            self._publish_state()
        self.get_logger().warning(
            'Rejected follow target because localization TF is missing; '
            f'use RViz 2D Pose Estimate first: {error}'
        )

    def _reject_target(self, reason: str) -> None:
        if self._target is None and not self._moving:
            self._state = FollowState.TARGET_REJECTED
            self._publish_state()
        self.get_logger().warning(reason)

    def _select_target(self, selected_point: Point2D, radius: float) -> None:
        if not target_is_selectable(selected_point, self._config):
            self._reject_target(
                'Rejected follow target outside the selectable front area'
            )
            return

        target, count = find_target_centroid(
            self._latest_points,
            selected_point,
            radius,
            self._config.min_target_points,
        )
        if target is None:
            self._reject_target(
                f'Rejected follow target: only {count} nearby scan points; '
                f'{self._config.min_target_points} required'
            )
            return

        self._moving = False
        self._awaiting_follow_mode = False
        self._follow_mode_requested_at = None
        self._publish_stop()
        self._request_mode('STOP')
        self._target = target
        self._loss_started_at = None
        self._state = FollowState.TARGET_SELECTED
        self._publish_state()
        self._publish_tracked_target()
        self.get_logger().info(
            f'Follow target selected at x={target.x:.2f}, y={target.y:.2f}, '
            f'selection_radius={radius:.2f}'
        )
        if self._start_on_target_selection:
            success, message = self._start_following()
            if success:
                self.get_logger().info(message)
            else:
                self.get_logger().warning(
                    f'Target selected but automatic start failed: {message}'
                )

    def _start_callback(self, request, response):
        del request
        response.success, response.message = self._start_following()
        return response

    def _start_following(self) -> tuple[bool, str]:
        if self._moving:
            return False, 'target follower is already running'
        if self._target is None:
            return False, 'select a valid target before starting'
        if not self._scan_is_fresh():
            return False, 'LaserScan is not fresh'
        if self._loss_started_at is not None:
            return False, 'target is not currently tracked'
        if self._mode_request_publisher.get_subscription_count() < 1:
            return False, 'motion command arbiter is not ready'

        if (
            self._pause_patrol_on_start
            and self._patrol_pause_client.service_is_ready()
        ):
            self._patrol_pause_client.call_async(Trigger.Request())
        self._moving = True
        self._awaiting_follow_mode = True
        self._follow_mode_requested_at = time.monotonic()
        self._state = FollowState.FOLLOWING
        self._request_mode('FOLLOW')
        self._publish_state()
        return True, 'target following started'

    def _stop_callback(self, request, response):
        del request
        self._moving = False
        self._awaiting_follow_mode = False
        self._follow_mode_requested_at = None
        self._publish_stop()
        self._request_mode('STOP')
        self._state = (
            FollowState.STOPPED
            if self._target is not None
            else FollowState.IDLE
        )
        self._publish_state()
        response.success = True
        response.message = 'target following stopped; NAV is not resumed'
        return response

    def _mode_status_callback(self, message: String) -> None:
        if message.data == 'FOLLOW':
            self._awaiting_follow_mode = False
            self._follow_mode_requested_at = None
            return
        if not self._moving or self._awaiting_follow_mode:
            return
        self._moving = False
        self._publish_stop()
        if self._target is not None:
            self._state = FollowState.STOPPED
            self._publish_state()

    def _control_tick(self) -> None:
        if self._moving and self._awaiting_follow_mode:
            request_time = self._follow_mode_requested_at
            if (
                request_time is None
                or time.monotonic() - request_time
                >= self._motion_mode_timeout
            ):
                self._moving = False
                self._awaiting_follow_mode = False
                self._follow_mode_requested_at = None
                self._publish_stop()
                self._request_mode('STOP')
                self._state = FollowState.ERROR
                self._publish_state()
                self.get_logger().error(
                    'Timed out waiting for FOLLOW mode confirmation'
                )
            else:
                self._publish_stop()
            return
        if self._target is not None and not self._scan_is_fresh():
            self._lose_target('LaserScan timed out')
            return
        if self._loss_started_at is not None:
            self._publish_stop()
            if (
                time.monotonic() - self._loss_started_at
                >= self._target_loss_timeout
            ):
                self._lose_target('target point cloud timed out')
            return
        if not self._moving or self._target is None:
            if self._last_output_was_nonzero:
                self._publish_stop()
            return

        command = compute_follow_command(self._target, self._config)
        message = Twist()
        message.linear.x = command.linear_x
        message.linear.y = command.linear_y
        message.angular.z = command.angular_z
        self._command_publisher.publish(message)
        self._last_output_was_nonzero = any((
            command.linear_x,
            command.linear_y,
            command.angular_z,
        ))

    def _tracking_missed(self, reason: str) -> None:
        if self._target is None:
            return
        if self._loss_started_at is None:
            self._loss_started_at = time.monotonic()
            self.get_logger().warning(reason)
        self._publish_stop()

    def _lose_target(self, reason: str) -> None:
        if self._target is None and self._state == FollowState.TARGET_LOST:
            return
        self._moving = False
        self._awaiting_follow_mode = False
        self._follow_mode_requested_at = None
        self._target = None
        self._loss_started_at = None
        self._publish_stop()
        self._request_mode('STOP')
        self._state = FollowState.TARGET_LOST
        self._publish_state()
        self.get_logger().error(reason)

    def _scan_is_fresh(self) -> bool:
        return (
            self._last_valid_scan is not None
            and time.monotonic() - self._last_valid_scan
            <= self._scan_timeout
        )

    def _stamp_is_fresh(self, message: LaserScan) -> bool:
        stamp_nanoseconds = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )
        if stamp_nanoseconds == 0:
            return False
        age_nanoseconds = (
            self.get_clock().now().nanoseconds - stamp_nanoseconds
        )
        return (
            age_nanoseconds <= int(self._scan_timeout * 1_000_000_000)
            and age_nanoseconds
            >= -int(self._transform_timeout * 1_000_000_000)
        )

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

    def _point_in_base_frame(self, message: PointStamped) -> Point2D:
        if not message.header.frame_id:
            raise TransformException('target frame_id is empty')
        if message.header.frame_id == self._base_frame_id:
            return Point2D(message.point.x, message.point.y)
        stamp = Time.from_msg(message.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = Time()
        transform = self._tf_buffer.lookup_transform(
            self._base_frame_id,
            message.header.frame_id,
            stamp,
            timeout=Duration(seconds=self._transform_timeout),
        )
        transformed = do_transform_point(message, transform)
        return Point2D(transformed.point.x, transformed.point.y)

    def _publish_state(self) -> None:
        message = String()
        message.data = self._state.value
        self._status_publisher.publish(message)

    def _publish_tracked_target(self) -> None:
        if self._target is None:
            return
        message = PointStamped()
        message.header.frame_id = self._base_frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.point.x = self._target.x
        message.point.y = self._target.y
        self._tracked_target_publisher.publish(message)

    def _request_mode(self, mode: str) -> None:
        message = String()
        message.data = mode
        self._mode_request_publisher.publish(message)

    def _publish_stop(self) -> None:
        self._command_publisher.publish(Twist())
        self._last_output_was_nonzero = False

    def prepare_shutdown(self) -> None:
        self._moving = False
        self._awaiting_follow_mode = False
        self._follow_mode_requested_at = None
        self._publish_stop()
        self._request_mode('STOP')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarTargetFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.prepare_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
