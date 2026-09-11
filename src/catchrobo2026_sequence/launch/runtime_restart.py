"""Launch-side coordination for restarting a set of ROS processes."""

import signal
import socket
import shutil

from launch.actions import EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown
from launch.events import Shutdown, matches_action
from launch.events.process import SignalProcess
from launch_ros.actions import Node


REQUEST_MARKER = 'CATCHROBO_RUNTIME_RESET_REQUESTED '
READY_MARKER = 'CATCHROBO_RUNTIME_SUPERVISOR_READY'


class RuntimeRestartCoordinator:
    """Keep the launch service alive while replacing every managed process."""

    def __init__(self, factory, enabled=True, restart_delay=0.2,
                 stop_timeout=5.0, kill_timeout=2.0, ack_socket='', expected_starts=0,
                 supervisor_arguments=None, session_directory=''):
        self._factory = factory
        self._enabled = enabled
        self._restart_delay = restart_delay
        self._stop_timeout = stop_timeout
        self._kill_timeout = kill_timeout
        self._ack_socket = ack_socket
        self._session_directory = session_directory
        self._managed = set()
        self._stopping = set()
        self._restarting = False
        self._waiting_for_starts = False
        self._expected_starts = 0
        self._observed_starts = 0
        self._generation = 0
        self._initial_expected_starts = expected_starts
        self._initial_observed_starts = 0
        self._stdout = ''
        self.supervisor = Node(
            package='catchrobo2026_ui', executable='reset_supervisor.py',
            name='runtime_reset_supervisor', output='screen',
            arguments=supervisor_arguments or [])

    def actions(self):
        runtime = list(self._factory())
        if not self._enabled:
            return runtime
        return [
            RegisterEventHandler(OnProcessStart(on_start=self._on_start)),
            RegisterEventHandler(OnProcessExit(on_exit=self._on_exit)),
            RegisterEventHandler(OnProcessIO(
                target_action=self.supervisor, on_stdout=self._on_supervisor_output)),
            RegisterEventHandler(OnShutdown(
                on_shutdown=[OpaqueFunction(function=self._cleanup)])),
            self.supervisor,
        ]

    def _on_start(self, event, _context):
        if event.action is not self.supervisor:
            self._managed.add(event.action)
            if not self._restarting and self._initial_expected_starts:
                self._initial_observed_starts += 1
                if self._initial_observed_starts >= self._initial_expected_starts:
                    self._initial_expected_starts = 0
                    self._send_message('ready 0')
            if self._restarting and not self._waiting_for_starts:
                self._stopping.add(event.action)
                return [self._signal(event.action, signal.SIGINT)]
            if self._waiting_for_starts:
                self._observed_starts += 1
                if self._observed_starts >= self._expected_starts:
                    self._waiting_for_starts = False
                    self._restarting = False
                    self._send_message(f'complete {self._generation}')
        return []

    def _on_supervisor_output(self, event):
        self._stdout += event.text.decode(errors='replace')
        actions = []
        while '\n' in self._stdout:
            line, self._stdout = self._stdout.split('\n', 1)
            if line == READY_MARKER and not self._managed and not self._restarting:
                actions.extend(self._factory())
            elif line.startswith(REQUEST_MARKER) and not self._restarting:
                self._generation = int(line[len(REQUEST_MARKER):])
                self._restarting = True
                self._stopping = set(self._managed)
                self._expected_starts = len(self._stopping)
                actions.append(LogInfo(msg='Runtime reset accepted; stopping ROS nodes'))
                actions.extend(self._signal(action, signal.SIGINT)
                               for action in self._stopping)
                actions.append(TimerAction(
                    period=self._stop_timeout,
                    actions=[OpaqueFunction(
                        function=self._escalate_term, args=[self._generation])]))
                if not self._stopping:
                    actions.extend(self._finish_restart())
        return actions

    def _on_exit(self, event, context):
        if event.action is self.supervisor:
            if not context.is_shutdown:
                return [EmitEvent(event=Shutdown(reason='runtime reset supervisor exited'))]
            return []
        self._managed.discard(event.action)
        if self._restarting:
            if self._waiting_for_starts:
                self._restarting = False
                self._waiting_for_starts = False
                if not context.is_shutdown:
                    return [EmitEvent(event=Shutdown(
                        reason='ROS node failed while runtime was restarting'))]
                return []
            self._stopping.discard(event.action)
            if not self._stopping:
                if context.is_shutdown:
                    self._restarting = False
                    return []
                return self._finish_restart()
            return []
        if not context.is_shutdown:
            return [EmitEvent(event=Shutdown(reason='managed ROS node exited'))]
        return []

    def _finish_restart(self):
        self._waiting_for_starts = True
        self._observed_starts = 0
        return [TimerAction(
            period=self._restart_delay,
            actions=[LogInfo(msg='All ROS nodes stopped; starting fresh processes'),
                     *self._factory()])]

    @staticmethod
    def _signal(action, signal_number):
        return EmitEvent(event=SignalProcess(
            signal_number=signal_number, process_matcher=matches_action(action)))

    def _escalate_term(self, context, generation):
        if (context.is_shutdown or generation != self._generation or
                not self._restarting or self._waiting_for_starts):
            return []
        actions = [self._signal(action, signal.SIGTERM) for action in self._stopping]
        actions.append(TimerAction(
            period=self._kill_timeout,
            actions=[OpaqueFunction(
                function=self._escalate_kill, args=[generation])]))
        return actions

    def _escalate_kill(self, context, generation):
        if (context.is_shutdown or generation != self._generation or
                not self._restarting or self._waiting_for_starts):
            return []
        return [self._signal(action, signal.SIGKILL) for action in self._stopping]

    def _send_message(self, message):
        if not self._ack_socket:
            return
        channel = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            channel.sendto(message.encode('ascii'), self._ack_socket)
        finally:
            channel.close()

    def _cleanup(self, _context):
        if self._session_directory:
            shutil.rmtree(self._session_directory, ignore_errors=True)
        return []
