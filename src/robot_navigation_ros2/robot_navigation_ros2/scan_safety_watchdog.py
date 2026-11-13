#!/usr/bin/env python3

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class ScanSafetyWatchdog(Node):

    def __init__(self, parameter_overrides=None) -> None:
        super().__init__(
            'scan_safety_watchdog',
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_in_topic', '/cmd_vel_smoothed')
        self.declare_parameter(
            'cmd_vel_out_topic', '/cmd_vel_collision_input'
        )
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('scan_timeout', 0.5)
        self.declare_parameter('transform_timeout', 0.1)

        scan_topic = str(self.get_parameter('scan_topic').value)
        cmd_vel_in_topic = str(
            self.get_parameter('cmd_vel_in_topic').value
        )
        cmd_vel_out_topic = str(
            self.get_parameter('cmd_vel_out_topic').value
        )
        self._base_frame_id = str(
            self.get_parameter('base_frame_id').value
        )
        self._scan_timeout = float(
            self.get_parameter('scan_timeout').value
        )
        self._transform_timeout = float(
            self.get_parameter('transform_timeout').value
        )
        if self._scan_timeout <= 0.0:
            raise ValueError('scan_timeout must be positive')
        if self._transform_timeout < 0.0:
            raise ValueError('transform_timeout must be nonnegative')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_valid_scan: float | None = None
        self._last_output_was_nonzero = False
        self._fault_active = True

        self._cmd_publisher = self.create_publisher(
            Twist, cmd_vel_out_topic, 10
        )
        self.create_subscription(
            LaserScan,
            scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            cmd_vel_in_topic,
            self._cmd_vel_callback,
            10,
        )
        self.create_timer(0.05, self._watchdog_tick)

    def _scan_callback(self, message: LaserScan) -> None:
        if not message.header.frame_id or not self._stamp_is_fresh(message):
            return
        if not self._tf_buffer.can_transform(
            self._base_frame_id,
            message.header.frame_id,
            Time(),
            timeout=Duration(seconds=self._transform_timeout),
        ):
            return
        self._last_valid_scan = time.monotonic()
        if self._fault_active:
            self._fault_active = False
            self.get_logger().info('LaserScan and TF are valid')

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

    def _cmd_vel_callback(self, message: Twist) -> None:
        if self._scan_is_valid():
            self._cmd_publisher.publish(message)
            self._last_output_was_nonzero = self._is_nonzero(message)
            return
        self._publish_stop()

    def _watchdog_tick(self) -> None:
        if self._scan_is_valid():
            return
        if not self._fault_active:
            self._fault_active = True
            self.get_logger().error(
                'LaserScan or TF timed out; forcing zero velocity'
            )
        if self._last_output_was_nonzero:
            self._publish_stop()

    def _scan_is_valid(self) -> bool:
        if self._last_valid_scan is None:
            return False
        return time.monotonic() - self._last_valid_scan <= self._scan_timeout

    def _publish_stop(self) -> None:
        self._cmd_publisher.publish(Twist())
        self._last_output_was_nonzero = False

    @staticmethod
    def _is_nonzero(message: Twist) -> bool:
        return any((
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        ))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanSafetyWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
