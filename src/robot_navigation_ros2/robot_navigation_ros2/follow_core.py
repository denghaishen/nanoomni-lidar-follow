#!/usr/bin/env python3

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclass(frozen=True)
class Transform2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def apply(self, point: Point2D) -> Point2D:
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        return Point2D(
            self.x + cosine * point.x - sine * point.y,
            self.y + sine * point.x + cosine * point.y,
        )

    def rotate(self, point: Point2D) -> Point2D:
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        return Point2D(
            cosine * point.x - sine * point.y,
            sine * point.x + cosine * point.y,
        )


@dataclass(frozen=True)
class FollowConfig:
    follow_distance: float = 1.0
    distance_deadband: float = 0.05
    target_radius: float = 0.30
    min_target_points: int = 1
    linear_gain: float = 0.5
    angular_gain: float = 1.0
    max_forward_speed: float = 0.30
    max_reverse_speed: float = 0.20
    max_angular_speed: float = 1.0
    allow_reverse: bool = True
    use_lateral_velocity: bool = False
    rotate_first_angle: float = 0.30
    angular_deadband: float = 0.10
    target_min_x: float = 0.30
    target_max_x: float = 3.00
    target_max_abs_y: float = 1.50

    def __post_init__(self) -> None:
        positive_values = {
            'follow_distance': self.follow_distance,
            'distance_deadband': self.distance_deadband,
            'target_radius': self.target_radius,
            'linear_gain': self.linear_gain,
            'angular_gain': self.angular_gain,
            'max_forward_speed': self.max_forward_speed,
            'max_reverse_speed': self.max_reverse_speed,
            'max_angular_speed': self.max_angular_speed,
            'rotate_first_angle': self.rotate_first_angle,
            'target_max_x': self.target_max_x,
            'target_max_abs_y': self.target_max_abs_y,
        }
        for name, value in positive_values.items():
            if value <= 0.0:
                raise ValueError(f'{name} must be positive')
        if self.angular_deadband < 0.0:
            raise ValueError('angular_deadband must be nonnegative')
        if self.min_target_points < 1:
            raise ValueError('min_target_points must be positive')
        if self.target_min_x < 0.0:
            raise ValueError('target_min_x must be nonnegative')
        if self.target_min_x >= self.target_max_x:
            raise ValueError('target_min_x must be less than target_max_x')
        if self.use_lateral_velocity:
            raise ValueError(
                'lateral following requires left and right safety zones'
            )


@dataclass(frozen=True)
class FollowCommand:
    linear_x: float
    linear_y: float
    angular_z: float


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def find_target_centroid(
    points: Sequence[Point2D],
    center: Point2D,
    radius: float,
    min_points: int,
) -> tuple[Point2D | None, int]:
    radius_squared = radius * radius
    candidates = [
        point
        for point in points
        if (
            (point.x - center.x) ** 2
            + (point.y - center.y) ** 2
        ) <= radius_squared
    ]
    if len(candidates) < min_points:
        return None, len(candidates)
    return Point2D(
        sum(point.x for point in candidates) / len(candidates),
        sum(point.y for point in candidates) / len(candidates),
    ), len(candidates)


def target_is_selectable(point: Point2D, config: FollowConfig) -> bool:
    return (
        config.target_min_x <= point.x <= config.target_max_x
        and abs(point.y) <= config.target_max_abs_y
    )


def compute_follow_command(
    target: Point2D,
    config: FollowConfig,
) -> FollowCommand:
    target_distance = math.hypot(target.x, target.y)
    distance_error = target_distance - config.follow_distance
    angle_error = math.atan2(target.y, target.x)

    if abs(distance_error) <= config.distance_deadband:
        linear_x = 0.0
    else:
        minimum_speed = (
            -config.max_reverse_speed if config.allow_reverse else 0.0
        )
        linear_x = clamp(
            config.linear_gain * distance_error,
            minimum_speed,
            config.max_forward_speed,
        )

    if abs(angle_error) > config.rotate_first_angle:
        linear_x = 0.0

    if abs(angle_error) <= config.angular_deadband:
        angular_z = 0.0
    else:
        angular_z = clamp(
            config.angular_gain * angle_error,
            -config.max_angular_speed,
            config.max_angular_speed,
        )
    return FollowCommand(
        linear_x=linear_x,
        linear_y=0.0,
        angular_z=angular_z,
    )


def scan_to_points(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    transform: Transform2D = Transform2D(),
) -> tuple[Point2D, ...]:
    points = []
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance):
            continue
        if distance < range_min or distance > range_max:
            continue
        angle = angle_min + index * angle_increment
        scan_point = Point2D(
            distance * math.cos(angle),
            distance * math.sin(angle),
        )
        points.append(transform.apply(scan_point))
    return tuple(points)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
