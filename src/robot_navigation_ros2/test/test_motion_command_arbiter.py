import os
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters

from robot_navigation_ros2.motion_command_arbiter import (
    MotionCommandArbiter,
    MotionMode,
    SourceSelector,
)


def make_selector(initial_mode=MotionMode.NAV):
    return SourceSelector(
        nav_timeout=0.5,
        follow_timeout=0.5,
        switch_stop_duration=0.05,
        initial_mode=initial_mode,
    )


def test_default_nav_mode_requires_fresh_nav_command():
    selector = make_selector()

    assert selector.selected_source(1.0) is None
    selector.update_nav(1.0)
    assert selector.selected_source(1.0) == 'nav'
    assert selector.selected_source(1.51) is None


def test_follow_mode_ignores_fresh_nav_source():
    selector = make_selector()
    selector.update_nav(1.0)
    selector.set_mode(MotionMode.FOLLOW, 1.0)
    selector.update_follow(1.01)

    assert selector.selected_source(1.02) is None
    assert selector.selected_source(1.06) == 'follow'


def test_follow_timeout_never_falls_back_to_nav():
    selector = make_selector()
    selector.update_nav(1.0)
    selector.set_mode(MotionMode.FOLLOW, 1.0)
    selector.update_follow(1.06)
    selector.update_nav(1.50)

    assert selector.selected_source(1.57) is None


def test_stop_mode_never_selects_a_source():
    selector = make_selector()
    selector.update_nav(1.0)
    selector.update_follow(1.0)
    selector.set_mode(MotionMode.STOP, 1.0)

    assert selector.selected_source(2.0) is None


def test_mode_switch_requires_a_new_source_command():
    selector = make_selector()
    selector.update_nav(1.0)
    selector.set_mode(MotionMode.FOLLOW, 1.0)
    selector.update_follow(1.1)
    assert selector.selected_source(1.1) == 'follow'

    selector.set_mode(MotionMode.NAV, 1.2)
    assert selector.selected_source(1.3) is None
    selector.update_nav(1.3)
    assert selector.selected_source(1.3) == 'nav'


def test_collision_monitor_front_stop_gates_on_mode():
    if rclpy.ok():
        rclpy.shutdown()
    os.environ['ROS_DOMAIN_ID'] = '193'
    rclpy.init()

    prefix = '/arbiter_gate_test'
    recorded = []

    service_node = Node('fake_collision_monitor')

    def handle(request, response):
        for parameter in request.parameters:
            recorded.append(
                (parameter.name, parameter.value.bool_value)
            )
        response.results = []
        for parameter in request.parameters:
            result = SetParametersResult()
            result.successful = True
            result.reason = ''
            response.results.append(result)
        return response

    service_node.create_service(
        SetParameters, f'{prefix}/set_parameters', handle
    )

    arbiter = MotionCommandArbiter(parameter_overrides=[
        Parameter('gate_collision_monitor', value=True),
        Parameter(
            'collision_monitor_param_service',
            value=f'{prefix}/set_parameters',
        ),
        Parameter(
            'collision_front_stop_param', value='FrontStopZone.enabled'
        ),
        Parameter('nav_service', value=f'{prefix}/nav'),
        Parameter('follow_service', value=f'{prefix}/follow'),
        Parameter('stop_service', value=f'{prefix}/stop'),
        Parameter('mode_request_topic', value=f'{prefix}/request'),
        Parameter('mode_status_topic', value=f'{prefix}/status'),
    ])

    executor = SingleThreadedExecutor()
    executor.add_node(service_node)
    executor.add_node(arbiter)

    try:
        deadline = time.monotonic() + 3.0
        while (
            not arbiter._collision_param_client.service_is_ready()
            and time.monotonic() < deadline
        ):
            executor.spin_once(timeout_sec=0.05)
        assert arbiter._collision_param_client.service_is_ready()

        arbiter._set_mode(MotionMode.FOLLOW)
        deadline = time.monotonic() + 3.0
        while not recorded and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert recorded
        assert recorded[-1] == ('FrontStopZone.enabled', False)

        arbiter._set_mode(MotionMode.NAV)
        deadline = time.monotonic() + 3.0
        while len(recorded) < 2 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert recorded[-1] == ('FrontStopZone.enabled', True)
    finally:
        executor.remove_node(arbiter)
        executor.remove_node(service_node)
        arbiter.destroy_node()
        service_node.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        if rclpy.ok():
            rclpy.shutdown()
