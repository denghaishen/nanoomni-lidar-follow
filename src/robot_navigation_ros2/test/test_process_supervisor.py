import signal

from robot_navigation_ros2.process_supervisor import (
    ProcessState,
    ProcessSupervisor,
)


class FakeProcess:

    def __init__(self, pid=4321):
        self.pid = pid
        self.returncode = None
        self.wait_calls = []

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.wait_calls.append(timeout)
        return self.returncode


class FakeClock:

    def __init__(self):
        self.value = 100.0

    def __call__(self):
        return self.value


def test_start_and_graceful_stop_use_process_group_without_shell():
    created = []
    signals = []
    process = FakeProcess()

    def factory(command, **kwargs):
        created.append((command, kwargs))
        return process

    supervisor = ProcessSupervisor(
        ['/opt/ros/humble/bin/ros2', 'launch', 'pkg', 'file.launch.py'],
        process_factory=factory,
        group_signal=lambda pid, sent: signals.append((pid, sent)),
    )

    success, _ = supervisor.start()
    assert success
    assert supervisor.state == ProcessState.RUNNING
    assert created == [(
        ['/opt/ros/humble/bin/ros2', 'launch', 'pkg', 'file.launch.py'],
        {'start_new_session': True},
    )]

    success, _ = supervisor.start()
    assert not success

    success, _ = supervisor.stop()
    assert success
    assert supervisor.state == ProcessState.STOPPING
    assert signals == [(4321, signal.SIGINT)]

    process.returncode = -signal.SIGINT
    supervisor.poll()
    assert supervisor.state == ProcessState.STOPPED
    assert supervisor.last_returncode == -signal.SIGINT


def test_stop_escalates_only_after_grace_periods():
    process = FakeProcess()
    clock = FakeClock()
    signals = []
    supervisor = ProcessSupervisor(
        ['robot'],
        stop_grace_seconds=10.0,
        terminate_grace_seconds=3.0,
        process_factory=lambda *_args, **_kwargs: process,
        group_signal=lambda pid, sent: signals.append((pid, sent)),
        clock=clock,
    )

    supervisor.start()
    supervisor.stop()
    assert signals[-1][1] == signal.SIGINT

    clock.value = 109.9
    supervisor.poll()
    assert signals[-1][1] == signal.SIGINT

    clock.value = 110.0
    supervisor.poll()
    assert signals[-1][1] == signal.SIGTERM

    clock.value = 113.0
    supervisor.poll()
    assert signals[-1][1] == signal.SIGKILL


def test_unexpected_nonzero_exit_is_failed():
    process = FakeProcess()
    supervisor = ProcessSupervisor(
        ['robot'],
        process_factory=lambda *_args, **_kwargs: process,
    )

    supervisor.start()
    process.returncode = 7

    assert supervisor.state == ProcessState.FAILED
    assert supervisor.last_returncode == 7
    assert 'code 7' in supervisor.last_error
