import math

import pytest

from robot_navigation_ros2.follow_core import (
    FollowConfig,
    Point2D,
    Transform2D,
    compute_follow_command,
    find_target_centroid,
    scan_to_points,
    target_is_selectable,
)


def make_config(**changes):
    values = {
        'follow_distance': 1.0,
        'distance_deadband': 0.05,
        'target_radius': 0.30,
        'min_target_points': 3,
        'linear_gain': 0.5,
        'angular_gain': 1.0,
        'max_forward_speed': 0.30,
        'max_reverse_speed': 0.20,
        'max_angular_speed': 1.0,
        'allow_reverse': True,
        'use_lateral_velocity': False,
        'rotate_first_angle': 0.30,
        'angular_deadband': 0.10,
        'target_min_x': 0.30,
        'target_max_x': 3.00,
        'target_max_abs_y': 1.50,
    }
    values.update(changes)
    return FollowConfig(**values)


def test_forward_deadband_and_reverse_distance_control():
    config = make_config()

    forward = compute_follow_command(Point2D(1.50, 0.0), config)
    at_distance = compute_follow_command(Point2D(1.00, 0.0), config)
    in_deadband = compute_follow_command(Point2D(1.02, 0.0), config)
    reverse = compute_follow_command(Point2D(0.80, 0.0), config)

    assert forward.linear_x == pytest.approx(0.25)
    assert at_distance.linear_x == 0.0
    assert in_deadband.linear_x == 0.0
    assert reverse.linear_x == pytest.approx(-0.10)
    assert all(command.linear_y == 0.0 for command in (
        forward, at_distance, in_deadband, reverse
    ))


def test_speed_limits_apply_in_both_directions():
    config = make_config()

    forward = compute_follow_command(Point2D(3.0, 0.0), config)
    reverse = compute_follow_command(Point2D(0.30, 0.0), config)

    assert forward.linear_x == 0.30
    assert reverse.linear_x == -0.20


def test_target_bearing_controls_rotation_in_both_directions():
    config = make_config()

    left = compute_follow_command(Point2D(0.80, 0.20), config)
    right = compute_follow_command(Point2D(1.50, -0.20), config)

    assert left.linear_x < 0.0
    assert left.angular_z > 0.0
    assert right.linear_x > 0.0
    assert right.angular_z < 0.0


def test_large_bearing_rotates_before_translation():
    config = make_config()

    command = compute_follow_command(Point2D(0.60, 0.60), config)

    assert abs(math.atan2(0.60, 0.60)) > config.rotate_first_angle
    assert command.linear_x == 0.0
    assert command.angular_z > 0.0


def test_target_selection_and_centroid_tracking():
    config = make_config()
    points = (
        Point2D(1.48, -0.02),
        Point2D(1.50, 0.00),
        Point2D(1.52, 0.02),
        Point2D(2.20, 0.00),
    )

    target, count = find_target_centroid(
        points,
        Point2D(1.50, 0.00),
        config.target_radius,
        config.min_target_points,
    )

    assert count == 3
    assert target == Point2D(1.50, 0.00)
    assert target_is_selectable(target, config)


def test_target_selection_rejects_too_few_points_and_invalid_area():
    config = make_config()
    target, count = find_target_centroid(
        (Point2D(1.0, 0.0), Point2D(1.1, 0.0)),
        Point2D(1.0, 0.0),
        config.target_radius,
        config.min_target_points,
    )

    assert target is None
    assert count == 2
    assert not target_is_selectable(Point2D(-0.5, 0.0), config)
    assert not target_is_selectable(Point2D(1.0, 2.0), config)


def test_scan_points_are_transformed_to_base_frame():
    points = scan_to_points(
        [1.0],
        angle_min=0.0,
        angle_increment=1.0,
        range_min=0.05,
        range_max=10.0,
        transform=Transform2D(x=0.10, y=0.20, yaw=math.pi / 2),
    )

    assert points[0].x == pytest.approx(0.10)
    assert points[0].y == pytest.approx(1.20)


def test_offset_target_rotates_before_translation():
    config = make_config()
    bearing = math.radians(30.0)
    target = Point2D(1.5 * math.cos(bearing), 1.5 * math.sin(bearing))
    command = compute_follow_command(target, config)

    assert abs(math.atan2(target.y, target.x)) > config.rotate_first_angle
    assert command.linear_x == 0.0
    assert command.angular_z == pytest.approx(math.radians(30.0))


def test_angular_deadband_suppresses_small_heading_noise():
    config = make_config()
    command = compute_follow_command(Point2D(1.0, 0.02), config)

    assert command.angular_z == 0.0
