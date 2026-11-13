#!/usr/bin/env python3

from dataclasses import replace
import signal
import time
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from robot_navigation_ros2.patrol_core import (
    PatrolRoute,
    PatrolState,
    PatrolStateMachine,
    ProgressDecision,
    load_route,
    yaw_to_quaternion,
)


class PatrolManager(Node):

    def __init__(self) -> None:
        super().__init__('patrol_manager')
        self.declare_parameter('route_file', '')
        self.declare_parameter('loop_count', -1)
        self.declare_parameter('autostart', False)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('action_name', 'navigate_to_pose')

        self._route_file = str(self.get_parameter('route_file').value)
        self._loop_count_override = int(self.get_parameter('loop_count').value)
        self._autostart_pending = bool(self.get_parameter('autostart').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        action_name = str(self.get_parameter('action_name').value)

        self._action_client = ActionClient(self, NavigateToPose, action_name)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_publisher = self.create_publisher(
            String, '/patrol/status', status_qos
        )
        self.create_service(Trigger, '/patrol/start', self._start_callback)
        self.create_service(Trigger, '/patrol/stop', self._stop_callback)
        self.create_service(Trigger, '/patrol/pause', self._pause_callback)
        self.create_service(Trigger, '/patrol/resume', self._resume_callback)

        self._machine = PatrolStateMachine(self._empty_route())
        self._operation_epoch = 0
        self._goal_sequence = 0
        self._pending_goal_token = None
        self._active_goal_token = None
        self._active_goal_handle = None
        self._dwell_deadline: Optional[float] = None
        self._paused_dwell_remaining: Optional[float] = None
        self._stop_deadline: Optional[float] = None
        self._next_autostart_attempt = 0.0
        self._shutting_down = False

        self._load_route_at_startup()
        self._control_timer = self.create_timer(0.1, self._control_tick)
        self._publish_state()

    def _empty_route(self) -> PatrolRoute:
        return PatrolRoute(
            frame_id='map',
            loop_count=1,
            max_retries=0,
            failure_policy='stop',
            waypoints=(),
        )

    def _load_route_at_startup(self) -> None:
        try:
            self._machine = PatrolStateMachine(self._read_route())
            self.get_logger().info(
                f'Loaded {len(self._machine.route.waypoints)} patrol waypoints'
            )
        except (OSError, ValueError) as exc:
            self._machine.mark_error()
            self.get_logger().error(f'Failed to load patrol route: {exc}')

    def _read_route(self) -> PatrolRoute:
        if not self._route_file:
            raise ValueError('route_file parameter is empty')
        route = load_route(self._route_file)
        if self._loop_count_override < -1:
            raise ValueError('loop_count override must be -1 or nonnegative')
        if self._loop_count_override >= 0:
            route = replace(route, loop_count=self._loop_count_override)
        return route

    def _start_callback(self, request, response):
        del request
        response.success, response.message = self._start_patrol()
        return response

    def _start_patrol(self) -> tuple[bool, str]:
        if self._shutting_down:
            return False, 'patrol manager is shutting down'
        if self._machine.state in {
            PatrolState.RUNNING,
            PatrolState.PAUSED,
            PatrolState.STOPPING,
        }:
            return False, f'patrol is already {self._machine.state.value}'
        if (
            self._pending_goal_token is not None
            or self._active_goal_handle is not None
        ):
            return False, 'a previous navigation goal is still active'

        try:
            route = self._read_route()
        except (OSError, ValueError) as exc:
            self._machine.mark_error()
            self._publish_state()
            return False, f'route is invalid: {exc}'
        if not route.waypoints:
            self._machine = PatrolStateMachine(route)
            self._machine.mark_error()
            self._publish_state()
            return False, 'route contains no waypoints'
        if not self._action_client.server_is_ready():
            return False, 'Nav2 NavigateToPose action server is not ready'
        if not self._tf_buffer.can_transform(
            route.frame_id,
            self._base_frame,
            Time(),
            timeout=Duration(seconds=0.2),
        ):
            return (
                False,
                f'{route.frame_id} -> {self._base_frame} TF is not ready',
            )

        self._machine = PatrolStateMachine(route)
        self._machine.start()
        self._operation_epoch += 1
        self._dwell_deadline = None
        self._paused_dwell_remaining = None
        self._autostart_pending = False
        self._publish_state()
        self.get_logger().info(
            f'Patrol started with {len(route.waypoints)} waypoints, '
            f'loop_count={route.loop_count}'
        )
        return True, 'patrol started'

    def _stop_callback(self, request, response):
        del request
        if self._machine.state == PatrolState.IDLE:
            response.success = True
            response.message = 'patrol is already IDLE'
            return response

        self._machine.begin_stop()
        self._operation_epoch += 1
        self._dwell_deadline = None
        self._paused_dwell_remaining = None
        self._stop_deadline = time.monotonic() + 5.0
        self._publish_state()
        self._cancel_active_goal()
        self._finish_stop_if_quiet()
        response.success = True
        response.message = 'patrol stop requested'
        return response

    def _pause_callback(self, request, response):
        del request
        if not self._machine.pause():
            response.success = False
            response.message = (
                f'cannot pause while {self._machine.state.value}'
            )
            return response

        self._operation_epoch += 1
        if self._dwell_deadline is not None:
            self._paused_dwell_remaining = max(
                0.0, self._dwell_deadline - time.monotonic()
            )
            self._dwell_deadline = None
        else:
            self._paused_dwell_remaining = None
        self._publish_state()
        self._cancel_active_goal()
        response.success = True
        response.message = 'patrol paused'
        return response

    def _resume_callback(self, request, response):
        del request
        if not self._machine.resume():
            response.success = False
            response.message = (
                f'cannot resume while {self._machine.state.value}'
            )
            return response

        self._operation_epoch += 1
        if self._paused_dwell_remaining is not None:
            self._dwell_deadline = (
                time.monotonic() + self._paused_dwell_remaining
            )
            self._paused_dwell_remaining = None
        self._publish_state()
        response.success = True
        response.message = 'patrol resumed'
        return response

    def _control_tick(self) -> None:
        if self._shutting_down:
            return

        if (
            self._autostart_pending
            and time.monotonic() >= self._next_autostart_attempt
        ):
            success, message = self._start_patrol()
            if not success:
                self.get_logger().info(f'Autostart waiting: {message}')
                self._next_autostart_attempt = time.monotonic() + 1.0

        if self._machine.state == PatrolState.STOPPING:
            self._finish_stop_if_quiet()
            if (
                self._machine.state == PatrolState.STOPPING
                and self._stop_deadline is not None
                and time.monotonic() >= self._stop_deadline
            ):
                self._machine.mark_error()
                self._publish_state()
                self.get_logger().error(
                    'Timed out while cancelling navigation goal'
                )
            return

        if self._machine.state != PatrolState.RUNNING:
            return
        if self._dwell_deadline is not None:
            if time.monotonic() >= self._dwell_deadline:
                self._dwell_deadline = None
                self._advance_after_success()
            return
        if (
            self._pending_goal_token is None
            and self._active_goal_handle is None
        ):
            self._send_current_waypoint()

    def _send_current_waypoint(self) -> None:
        waypoint = self._machine.current_waypoint
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._machine.route.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint.x
        goal.pose.pose.position.y = waypoint.y
        qx, qy, qz, qw = yaw_to_quaternion(waypoint.yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self._goal_sequence += 1
        token = (
            self._operation_epoch,
            self._goal_sequence,
            self._machine.current_index,
        )
        self._pending_goal_token = token
        self.get_logger().info(
            f'Loop {self._machine.loop_number}, waypoint '
            f'{self._machine.current_index + 1}/'
            f'{len(self._machine.route.waypoints)}: {waypoint.name}, '
            f'retry {self._machine.retry_count}/'
            f'{self._machine.route.max_retries}'
        )
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed, goal_token=token: self._goal_response(
                completed, goal_token
            )
        )

    def _goal_response(self, future, token) -> None:
        if self._pending_goal_token == token:
            self._pending_goal_token = None
        try:
            goal_handle = future.result()
        except Exception as exc:
            if self._is_current_token(token):
                self._handle_failure(f'goal request failed: {exc}')
            self._finish_stop_if_quiet()
            return

        if not self._is_current_token(token):
            if goal_handle.accepted:
                self._active_goal_token = token
                self._active_goal_handle = goal_handle
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(
                    lambda completed, goal_token=token: self._goal_result(
                        completed, goal_token
                    )
                )
                goal_handle.cancel_goal_async()
            self._finish_stop_if_quiet()
            return
        if not goal_handle.accepted:
            self._handle_failure('goal was rejected by Nav2')
            return

        self._active_goal_token = token
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, goal_token=token: self._goal_result(
                completed, goal_token
            )
        )

    def _goal_result(self, future, token) -> None:
        if self._active_goal_token == token:
            self._active_goal_token = None
            self._active_goal_handle = None
        if not self._is_current_token(token):
            self._finish_stop_if_quiet()
            return

        try:
            status = future.result().status
        except Exception as exc:
            self._handle_failure(f'goal result failed: {exc}')
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            waypoint = self._machine.current_waypoint
            self.get_logger().info(f'Arrived at waypoint: {waypoint.name}')
            self._dwell_deadline = time.monotonic() + waypoint.wait_seconds
            return

        status_name = {
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }.get(status, str(status))
        self._handle_failure(f'navigation finished with {status_name}')

    def _advance_after_success(self) -> None:
        completed_before = self._machine.completed_loops
        decision = self._machine.waypoint_succeeded()
        if self._machine.completed_loops > completed_before:
            self.get_logger().info(
                f'Completed patrol loop {self._machine.completed_loops}'
            )
        if decision == ProgressDecision.COMPLETE:
            self._publish_state()
            self.get_logger().info('Patrol completed')

    def _handle_failure(self, reason: str) -> None:
        waypoint = self._machine.current_waypoint
        decision = self._machine.waypoint_failed()
        if decision == ProgressDecision.RETRY:
            self.get_logger().warning(
                f'{waypoint.name}: {reason}; retry '
                f'{self._machine.retry_count}/'
                f'{self._machine.route.max_retries}'
            )
        elif decision == ProgressDecision.NEXT:
            self.get_logger().warning(f'{waypoint.name}: {reason}; skipping')
        else:
            self.get_logger().error(
                f'{waypoint.name}: {reason}; patrol stopped'
            )
            self._publish_state()

    def _is_current_token(self, token) -> bool:
        return (
            token[0] == self._operation_epoch
            and self._machine.state == PatrolState.RUNNING
        )

    def _cancel_active_goal(self) -> None:
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()

    def _finish_stop_if_quiet(self) -> None:
        if self._machine.state != PatrolState.STOPPING:
            return
        if (
            self._pending_goal_token is not None
            or self._active_goal_handle is not None
        ):
            return
        self._machine.finish_stop()
        self._stop_deadline = None
        self._publish_state()
        self.get_logger().info('Patrol stopped')

    def _publish_state(self) -> None:
        message = String()
        message.data = self._machine.state.value
        self._status_publisher.publish(message)

    def prepare_shutdown(self):
        self._shutting_down = True
        self._operation_epoch += 1
        self._dwell_deadline = None
        self._paused_dwell_remaining = None
        cancel_future = None
        if self._active_goal_handle is not None and rclpy.ok():
            cancel_future = self._active_goal_handle.cancel_goal_async()
        self._machine.finish_stop()
        if rclpy.ok():
            self._publish_state()
        return cancel_future


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PatrolManager()
    shutdown_requested = False

    def request_shutdown(signum, frame):
        nonlocal shutdown_requested
        del signum, frame
        shutdown_requested = True

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    try:
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        cancel_future = node.prepare_shutdown()
        if cancel_future is not None and rclpy.ok():
            rclpy.spin_until_future_complete(
                node, cancel_future, timeout_sec=2.0
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
