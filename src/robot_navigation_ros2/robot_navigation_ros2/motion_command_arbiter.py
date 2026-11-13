#!/usr/bin/env python3

from enum import Enum
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MotionMode(str, Enum):
    NAV = 'NAV'
    FOLLOW = 'FOLLOW'
    STOP = 'STOP'


class SourceSelector:

    def __init__(
        self,
        nav_timeout: float,
        follow_timeout: float,
        switch_stop_duration: float,
        initial_mode: MotionMode = MotionMode.NAV,
    ) -> None:
        if nav_timeout <= 0.0 or follow_timeout <= 0.0:
            raise ValueError('source timeouts must be positive')
        if switch_stop_duration < 0.0:
            raise ValueError('switch_stop_duration must be nonnegative')
        self.nav_timeout = nav_timeout
        self.follow_timeout = follow_timeout
        self.switch_stop_duration = switch_stop_duration
        self.mode = initial_mode
        self._nav_updated_at: float | None = None
        self._follow_updated_at: float | None = None
        self._hold_until = 0.0

    def update_nav(self, now: float) -> None:
        self._nav_updated_at = now

    def update_follow(self, now: float) -> None:
        self._follow_updated_at = now

    def set_mode(self, mode: MotionMode, now: float) -> None:
        self.mode = mode
        self._hold_until = now + self.switch_stop_duration
        if mode == MotionMode.NAV:
            self._nav_updated_at = None
        elif mode == MotionMode.FOLLOW:
            self._follow_updated_at = None

    def selected_source(self, now: float) -> str | None:
        if self.mode == MotionMode.STOP or now < self._hold_until:
            return None
        if self.mode == MotionMode.NAV:
            if self._is_fresh(self._nav_updated_at, self.nav_timeout, now):
                return 'nav'
            return None
        if self._is_fresh(
            self._follow_updated_at, self.follow_timeout, now
        ):
            return 'follow'
        return None

    @staticmethod
    def _is_fresh(
        updated_at: float | None,
        timeout: float,
        now: float,
    ) -> bool:
        return updated_at is not None and now - updated_at <= timeout


class MotionCommandArbiter(Node):

    def __init__(self, parameter_overrides=None) -> None:
        super().__init__(
            'motion_command_arbiter',
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter('nav_cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('follow_cmd_vel_topic', '/cmd_vel_follow')
        self.declare_parameter('output_cmd_vel_topic', '/cmd_vel_selected')
        self.declare_parameter('mode_request_topic', '/motion_mode/request')
        self.declare_parameter('mode_status_topic', '/motion_mode/status')
        self.declare_parameter('nav_service', '/motion_mode/nav')
        self.declare_parameter('follow_service', '/motion_mode/follow')
        self.declare_parameter('stop_service', '/motion_mode/stop')
        self.declare_parameter('initial_mode', 'NAV')
        self.declare_parameter('nav_source_timeout', 0.5)
        self.declare_parameter('follow_source_timeout', 0.5)
        self.declare_parameter('switch_stop_duration', 0.05)
        self.declare_parameter('publish_frequency', 30.0)
        self.declare_parameter('gate_collision_monitor', True)
        self.declare_parameter(
            'collision_monitor_param_service', '/collision_monitor/set_parameters'
        )
        self.declare_parameter(
            'collision_front_stop_param', 'FrontStopZone.enabled'
        )

        initial_mode = self._parse_mode(
            str(self.get_parameter('initial_mode').value)
        )
        nav_timeout = float(
            self.get_parameter('nav_source_timeout').value
        )
        follow_timeout = float(
            self.get_parameter('follow_source_timeout').value
        )
        switch_stop_duration = float(
            self.get_parameter('switch_stop_duration').value
        )
        publish_frequency = float(
            self.get_parameter('publish_frequency').value
        )
        if publish_frequency <= 0.0:
            raise ValueError('publish_frequency must be positive')

        self._gate_collision_monitor = bool(
            self.get_parameter('gate_collision_monitor').value
        )
        self._collision_monitor_param_service = str(
            self.get_parameter('collision_monitor_param_service').value
        )
        self._collision_front_stop_param = str(
            self.get_parameter('collision_front_stop_param').value
        )

        self._selector = SourceSelector(
            nav_timeout,
            follow_timeout,
            switch_stop_duration,
            initial_mode,
        )
        self._nav_command = Twist()
        self._follow_command = Twist()

        output_topic = str(
            self.get_parameter('output_cmd_vel_topic').value
        )
        self._command_publisher = self.create_publisher(
            Twist, output_topic, 10
        )
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_publisher = self.create_publisher(
            String,
            str(self.get_parameter('mode_status_topic').value),
            status_qos,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('nav_cmd_vel_topic').value),
            self._nav_callback,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('follow_cmd_vel_topic').value),
            self._follow_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('mode_request_topic').value),
            self._mode_request_callback,
            10,
        )
        self.create_service(
            Trigger,
            str(self.get_parameter('nav_service').value),
            self._nav_service_callback,
        )
        self.create_service(
            Trigger,
            str(self.get_parameter('follow_service').value),
            self._follow_service_callback,
        )
        self.create_service(
            Trigger,
            str(self.get_parameter('stop_service').value),
            self._stop_service_callback,
        )
        self._collision_param_client = None
        if self._gate_collision_monitor:
            self._collision_param_client = self.create_client(
                SetParameters,
                self._collision_monitor_param_service,
            )
        self.create_timer(1.0 / publish_frequency, self._publish_selected)
        self._publish_status()
        self._publish_stop()

    def _nav_callback(self, message: Twist) -> None:
        self._nav_command = self._copy_twist(message)
        self._selector.update_nav(time.monotonic())

    def _follow_callback(self, message: Twist) -> None:
        self._follow_command = self._copy_twist(message)
        self._selector.update_follow(time.monotonic())

    def _mode_request_callback(self, message: String) -> None:
        try:
            mode = self._parse_mode(message.data)
        except ValueError:
            self.get_logger().warning(
                f'Ignoring invalid motion mode request: {message.data!r}'
            )
            return
        self._set_mode(mode)

    def _nav_service_callback(self, request, response):
        del request
        self._set_mode(MotionMode.NAV)
        response.success = True
        response.message = 'motion mode set to NAV; waiting for fresh input'
        return response

    def _follow_service_callback(self, request, response):
        del request
        self._set_mode(MotionMode.FOLLOW)
        response.success = True
        response.message = 'motion mode set to FOLLOW; waiting for fresh input'
        return response

    def _stop_service_callback(self, request, response):
        del request
        self._set_mode(MotionMode.STOP)
        response.success = True
        response.message = 'motion mode set to STOP'
        return response

    def _set_mode(self, mode: MotionMode) -> None:
        self._publish_stop()
        self._selector.set_mode(mode, time.monotonic())
        self._publish_status()
        self._update_collision_gate(mode)
        self.get_logger().info(f'Motion mode: {mode.value}')

    def _update_collision_gate(self, mode: MotionMode) -> None:
        if self._collision_param_client is None:
            return
        enabled = mode != MotionMode.FOLLOW
        if not self._collision_param_client.service_is_ready():
            self.get_logger().warning(
                'Collision monitor parameter service is not ready; '
                f'{self._collision_front_stop_param} left unchanged'
            )
            return
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                self._collision_front_stop_param,
                Parameter.Type.BOOL,
                enabled,
            ).to_parameter_msg()
        ]
        self._collision_param_client.call_async(request)
        self.get_logger().info(
            f'Collision monitor {self._collision_front_stop_param} '
            f'= {enabled}'
        )

    def _publish_selected(self) -> None:
        source = self._selector.selected_source(time.monotonic())
        if source == 'nav':
            self._command_publisher.publish(self._nav_command)
        elif source == 'follow':
            self._command_publisher.publish(self._follow_command)
        else:
            self._publish_stop()

    def _publish_status(self) -> None:
        message = String()
        message.data = self._selector.mode.value
        self._status_publisher.publish(message)

    def _publish_stop(self) -> None:
        self._command_publisher.publish(Twist())

    @staticmethod
    def _copy_twist(message: Twist) -> Twist:
        copied = Twist()
        copied.linear.x = message.linear.x
        copied.linear.y = message.linear.y
        copied.linear.z = message.linear.z
        copied.angular.x = message.angular.x
        copied.angular.y = message.angular.y
        copied.angular.z = message.angular.z
        return copied

    @staticmethod
    def _parse_mode(value: str) -> MotionMode:
        try:
            return MotionMode(value.strip().upper())
        except ValueError as exc:
            raise ValueError(f'unknown motion mode: {value}') from exc

    def prepare_shutdown(self) -> None:
        self._selector.set_mode(MotionMode.STOP, time.monotonic())
        self._publish_stop()
        self._publish_status()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionCommandArbiter()
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
