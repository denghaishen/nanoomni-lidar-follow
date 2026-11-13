import math

import pytest

from robot_navigation_ros2.patrol_core import (
    PatrolRoute,
    PatrolState,
    PatrolStateMachine,
    PatrolWaypoint,
    ProgressDecision,
    parse_route,
    yaw_to_quaternion,
)


def make_route(
    waypoint_count=2,
    loop_count=1,
    max_retries=0,
    failure_policy='stop',
):
    waypoints = tuple(
        PatrolWaypoint(
            name=f'point_{index}',
            x=float(index),
            y=0.0,
            yaw=0.0,
            wait_seconds=0.0,
        )
        for index in range(waypoint_count)
    )
    return PatrolRoute(
        frame_id='map',
        loop_count=loop_count,
        max_retries=max_retries,
        failure_policy=failure_policy,
        waypoints=waypoints,
    )


def test_route_parsing():
    route = parse_route({
        'frame_id': 'map',
        'loop_count': 2,
        'max_retries': 1,
        'failure_policy': 'skip',
        'waypoints': [{
            'name': 'door',
            'x': 1.2,
            'y': -0.4,
            'yaw': 1.57,
            'wait_seconds': 3.0,
        }],
    })

    assert route.frame_id == 'map'
    assert route.loop_count == 2
    assert route.max_retries == 1
    assert route.failure_policy == 'skip'
    assert route.waypoints[0].name == 'door'
    assert route.waypoints[0].wait_seconds == 3.0


def test_yaw_to_quaternion():
    qx, qy, qz, qw = yaw_to_quaternion(math.pi)

    assert qx == 0.0
    assert qy == 0.0
    assert qz == pytest.approx(1.0)
    assert qw == pytest.approx(0.0, abs=1e-12)


def test_empty_route_rejects_start():
    machine = PatrolStateMachine(make_route(waypoint_count=0))

    with pytest.raises(ValueError, match='no waypoints'):
        machine.start()

    assert machine.state == PatrolState.IDLE


def test_finite_loop_count():
    machine = PatrolStateMachine(make_route(loop_count=2))
    machine.start()

    assert machine.waypoint_succeeded() == ProgressDecision.NEXT
    assert machine.current_index == 1
    assert machine.waypoint_succeeded() == ProgressDecision.NEXT
    assert machine.completed_loops == 1
    assert machine.current_index == 0
    assert machine.waypoint_succeeded() == ProgressDecision.NEXT
    assert machine.waypoint_succeeded() == ProgressDecision.COMPLETE
    assert machine.completed_loops == 2
    assert machine.state == PatrolState.COMPLETED


def test_zero_loop_count_runs_forever():
    machine = PatrolStateMachine(
        make_route(waypoint_count=1, loop_count=0)
    )
    machine.start()

    for expected_loops in range(1, 4):
        assert machine.waypoint_succeeded() == ProgressDecision.NEXT
        assert machine.completed_loops == expected_loops
        assert machine.state == PatrolState.RUNNING


def test_pause_resume_and_stop_transitions():
    machine = PatrolStateMachine(make_route())
    machine.start()

    assert machine.pause()
    assert machine.state == PatrolState.PAUSED
    assert machine.resume()
    assert machine.state == PatrolState.RUNNING
    assert machine.begin_stop()
    assert machine.state == PatrolState.STOPPING
    machine.finish_stop()
    assert machine.state == PatrolState.IDLE


def test_stop_prevents_waypoint_continuation():
    machine = PatrolStateMachine(make_route())
    machine.start()
    machine.begin_stop()
    machine.finish_stop()

    with pytest.raises(RuntimeError, match='not RUNNING'):
        machine.waypoint_succeeded()

    assert machine.current_index == 0
    assert machine.state == PatrolState.IDLE


def test_failure_retries_then_stops():
    machine = PatrolStateMachine(make_route(max_retries=1))
    machine.start()

    assert machine.waypoint_failed() == ProgressDecision.RETRY
    assert machine.retry_count == 1
    assert machine.waypoint_failed() == ProgressDecision.ERROR
    assert machine.state == PatrolState.ERROR


def test_failure_policy_skip_advances():
    machine = PatrolStateMachine(
        make_route(max_retries=0, failure_policy='skip')
    )
    machine.start()

    assert machine.waypoint_failed() == ProgressDecision.NEXT
    assert machine.current_index == 1
