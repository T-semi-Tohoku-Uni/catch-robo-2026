#!/usr/bin/env python3
import importlib.util
from pathlib import Path

from launch.actions import TimerAction


def load_coordinator():
    path = Path(__file__).parents[1] / 'launch/runtime_restart.py'
    spec = importlib.util.spec_from_file_location('runtime_restart', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.RuntimeRestartCoordinator


class Event:
    def __init__(self, action=None, text=b''):
        self.action = action
        self.text = text


class Context:
    def __init__(self, shutdown=False):
        self.is_shutdown = shutdown


def test_restart_waits_for_every_exit_and_rejects_startup_failure():
    factory_calls = []

    def factory():
        factory_calls.append(True)
        return [object(), object()]

    coordinator = load_coordinator()(factory, ack_socket='', expected_starts=2)
    old = [object(), object()]
    for action in old:
        coordinator._on_start(Event(action), Context())
    output = coordinator._on_supervisor_output(Event(
        text=b'CATCHROBO_RUNTIME_RESET_REQUESTED 1\n'))
    assert len(output) == 4
    assert coordinator._on_exit(Event(old[0]), Context()) == []
    finished = coordinator._on_exit(Event(old[1]), Context())
    assert len(finished) == 1 and isinstance(finished[0], TimerAction)
    fresh = object()
    coordinator._on_start(Event(fresh), Context())
    failure = coordinator._on_exit(Event(fresh), Context())
    assert failure and not coordinator._restarting


def test_shutdown_never_respawns_and_markers_may_be_split():
    coordinator = load_coordinator()(lambda: [object()], ack_socket='')
    action = object()
    coordinator._on_start(Event(action), Context())
    assert coordinator._on_supervisor_output(Event(
        text=b'CATCHROBO_RUNTIME_RESET_')) == []
    output = coordinator._on_supervisor_output(Event(text=b'REQUESTED 2\n'))
    assert output
    assert coordinator._on_exit(Event(action), Context(shutdown=True)) == []
