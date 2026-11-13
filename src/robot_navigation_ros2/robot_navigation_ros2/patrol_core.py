from dataclasses import dataclass
from enum import Enum
import math
from pathlib import Path
from typing import Any, Mapping

import yaml


class PatrolState(str, Enum):
    IDLE = 'IDLE'
    RUNNING = 'RUNNING'
    PAUSED = 'PAUSED'
    STOPPING = 'STOPPING'
    COMPLETED = 'COMPLETED'
    ERROR = 'ERROR'


class ProgressDecision(str, Enum):
    NEXT = 'NEXT'
    RETRY = 'RETRY'
    COMPLETE = 'COMPLETE'
    ERROR = 'ERROR'


@dataclass(frozen=True)
class PatrolWaypoint:
    name: str
    x: float
    y: float
    yaw: float
    wait_seconds: float


@dataclass(frozen=True)
class PatrolRoute:
    frame_id: str
    loop_count: int
    max_retries: int
    failure_policy: str
    waypoints: tuple[PatrolWaypoint, ...]


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def _number(value: Any, field: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{field} must be a number')
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{field} must be a number') from exc
    if not math.isfinite(parsed):
        raise ValueError(f'{field} must be finite')
    return parsed


def _nonnegative_integer(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f'{field} must be an integer')
    if value < 0:
        raise ValueError(f'{field} must be nonnegative')
    return value


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f'{field} must be a mapping')
    return value


def parse_route(data: Any) -> PatrolRoute:
    root = _mapping({} if data is None else data, 'route')
    frame_id = str(root.get('frame_id', 'map')).strip()
    if not frame_id:
        raise ValueError('frame_id must not be empty')

    loop_count = _nonnegative_integer(root.get('loop_count', 1), 'loop_count')
    max_retries = _nonnegative_integer(
        root.get('max_retries', 0), 'max_retries'
    )
    failure_policy = str(root.get('failure_policy', 'stop')).strip().lower()
    if failure_policy not in {'stop', 'skip'}:
        raise ValueError('failure_policy must be stop or skip')

    raw_waypoints = root.get('waypoints', [])
    if not isinstance(raw_waypoints, list):
        raise ValueError('waypoints must be a list')

    waypoints = []
    for index, raw_waypoint in enumerate(raw_waypoints):
        waypoint = _mapping(raw_waypoint, f'waypoints[{index}]')
        name = str(waypoint.get('name', '')).strip()
        if not name:
            raise ValueError(f'waypoints[{index}].name must not be empty')
        wait_seconds = _number(
            waypoint.get('wait_seconds', 0.0),
            f'waypoints[{index}].wait_seconds',
        )
        if wait_seconds < 0.0:
            raise ValueError(
                f'waypoints[{index}].wait_seconds must be nonnegative'
            )
        waypoints.append(
            PatrolWaypoint(
                name=name,
                x=_number(waypoint.get('x'), f'waypoints[{index}].x'),
                y=_number(waypoint.get('y'), f'waypoints[{index}].y'),
                yaw=_number(waypoint.get('yaw'), f'waypoints[{index}].yaw'),
                wait_seconds=wait_seconds,
            )
        )

    return PatrolRoute(
        frame_id=frame_id,
        loop_count=loop_count,
        max_retries=max_retries,
        failure_policy=failure_policy,
        waypoints=tuple(waypoints),
    )


def load_route(path: str) -> PatrolRoute:
    route_path = Path(path).expanduser()
    with route_path.open('r', encoding='utf-8') as route_file:
        return parse_route(yaml.safe_load(route_file))


class PatrolStateMachine:

    def __init__(self, route: PatrolRoute) -> None:
        self.route = route
        self.state = PatrolState.IDLE
        self.current_index = 0
        self.completed_loops = 0
        self.retry_count = 0

    @property
    def current_waypoint(self) -> PatrolWaypoint:
        if not self.route.waypoints:
            raise ValueError('route has no waypoints')
        return self.route.waypoints[self.current_index]

    @property
    def loop_number(self) -> int:
        return self.completed_loops + 1

    def start(self) -> None:
        if not self.route.waypoints:
            raise ValueError('route has no waypoints')
        if self.state not in {
            PatrolState.IDLE,
            PatrolState.COMPLETED,
            PatrolState.ERROR,
        }:
            raise RuntimeError(f'cannot start from {self.state.value}')
        self.current_index = 0
        self.completed_loops = 0
        self.retry_count = 0
        self.state = PatrolState.RUNNING

    def pause(self) -> bool:
        if self.state != PatrolState.RUNNING:
            return False
        self.state = PatrolState.PAUSED
        return True

    def resume(self) -> bool:
        if self.state != PatrolState.PAUSED:
            return False
        self.state = PatrolState.RUNNING
        return True

    def begin_stop(self) -> bool:
        if self.state == PatrolState.IDLE:
            return False
        self.state = PatrolState.STOPPING
        return True

    def finish_stop(self) -> None:
        self.state = PatrolState.IDLE
        self.retry_count = 0

    def mark_error(self) -> None:
        self.state = PatrolState.ERROR

    def waypoint_succeeded(self) -> ProgressDecision:
        self._require_running()
        self.retry_count = 0
        return self._advance()

    def waypoint_failed(self) -> ProgressDecision:
        self._require_running()
        if self.retry_count < self.route.max_retries:
            self.retry_count += 1
            return ProgressDecision.RETRY
        if self.route.failure_policy == 'skip':
            self.retry_count = 0
            return self._advance()
        self.state = PatrolState.ERROR
        return ProgressDecision.ERROR

    def _advance(self) -> ProgressDecision:
        if self.current_index + 1 < len(self.route.waypoints):
            self.current_index += 1
            return ProgressDecision.NEXT

        self.completed_loops += 1
        if (
            self.route.loop_count > 0
            and self.completed_loops >= self.route.loop_count
        ):
            self.state = PatrolState.COMPLETED
            return ProgressDecision.COMPLETE

        self.current_index = 0
        return ProgressDecision.NEXT

    def _require_running(self) -> None:
        if self.state != PatrolState.RUNNING:
            raise RuntimeError(f'patrol is {self.state.value}, not RUNNING')
