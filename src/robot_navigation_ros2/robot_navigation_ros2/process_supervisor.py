#!/usr/bin/env python3

from enum import Enum
import os
import signal
import subprocess
import time
from typing import Callable, Sequence


class ProcessState(str, Enum):
    STOPPED = 'STOPPED'
    RUNNING = 'RUNNING'
    STOPPING = 'STOPPING'
    FAILED = 'FAILED'


class ProcessSupervisor:

    def __init__(
        self,
        command: Sequence[str],
        stop_grace_seconds: float = 20.0,
        terminate_grace_seconds: float = 5.0,
        process_factory: Callable = subprocess.Popen,
        group_signal: Callable[[int, int], None] = os.killpg,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not command:
            raise ValueError('command must not be empty')
        if stop_grace_seconds <= 0.0:
            raise ValueError('stop_grace_seconds must be positive')
        if terminate_grace_seconds <= 0.0:
            raise ValueError('terminate_grace_seconds must be positive')
        self.command = tuple(str(value) for value in command)
        self.stop_grace_seconds = stop_grace_seconds
        self.terminate_grace_seconds = terminate_grace_seconds
        self._process_factory = process_factory
        self._group_signal = group_signal
        self._clock = clock
        self._process = None
        self._stopping = False
        self._terminate_sent = False
        self._deadline = 0.0
        self._failed = False
        self._last_returncode: int | None = None
        self._last_error = ''

    @property
    def state(self) -> ProcessState:
        self.poll()
        if self._process is None:
            return ProcessState.FAILED if self._failed else ProcessState.STOPPED
        if self._stopping:
            return ProcessState.STOPPING
        return ProcessState.RUNNING

    @property
    def pid(self) -> int | None:
        self.poll()
        return None if self._process is None else int(self._process.pid)

    @property
    def last_returncode(self) -> int | None:
        return self._last_returncode

    @property
    def last_error(self) -> str:
        return self._last_error

    def start(self) -> tuple[bool, str]:
        self.poll()
        if self._process is not None:
            return False, 'robot system is already running or stopping'
        try:
            self._process = self._process_factory(
                list(self.command),
                start_new_session=True,
            )
        except OSError as exc:
            self._failed = True
            self._last_error = str(exc)
            return False, f'failed to start robot system: {exc}'
        self._stopping = False
        self._terminate_sent = False
        self._failed = False
        self._last_returncode = None
        self._last_error = ''
        return True, 'robot system start requested'

    def stop(self) -> tuple[bool, str]:
        self.poll()
        if self._process is None:
            return False, 'robot system is already stopped'
        if self._stopping:
            return False, 'robot system is already stopping'
        try:
            self._group_signal(int(self._process.pid), signal.SIGINT)
        except ProcessLookupError:
            self.poll()
            return False, 'robot system already exited'
        self._stopping = True
        self._terminate_sent = False
        self._deadline = self._clock() + self.stop_grace_seconds
        return True, 'robot system stop requested'

    def poll(self) -> None:
        if self._process is None:
            return
        returncode = self._process.poll()
        if returncode is not None:
            requested_stop = self._stopping
            self._last_returncode = int(returncode)
            self._process = None
            self._stopping = False
            self._terminate_sent = False
            self._failed = not requested_stop and returncode != 0
            if self._failed:
                self._last_error = (
                    f'robot system exited with code {returncode}'
                )
            return
        if not self._stopping or self._clock() < self._deadline:
            return
        next_signal = signal.SIGTERM
        next_delay = self.terminate_grace_seconds
        if self._terminate_sent:
            next_signal = signal.SIGKILL
            next_delay = 1.0
        try:
            self._group_signal(int(self._process.pid), next_signal)
        except ProcessLookupError:
            return
        self._terminate_sent = True
        self._deadline = self._clock() + next_delay

    def shutdown(self) -> None:
        self.poll()
        if self._process is None:
            return
        if not self._stopping:
            self.stop()
        deadline = (
            self._clock()
            + self.stop_grace_seconds
            + self.terminate_grace_seconds
            + 2.0
        )
        while self._process is not None and self._clock() < deadline:
            self.poll()
            time.sleep(0.05)
        if self._process is not None:
            try:
                self._group_signal(int(self._process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass
            self.poll()
