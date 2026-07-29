#!/usr/bin/env python3
"""Fault scenario runner node (docs sections 6.3, 11).

Loads a YAML scenario, then drives the target emulator's fault services
on the scenario timeline:

    events:
      - at_sec: 5.0
        action: inject_fault
        fault: {fault_type: drop, target: data, parameters: {...}}
      - at_sec: 15.0
        action: clear_all_faults

Services:
    ~/load_scenario      (hils_bridge_interfaces/srv/LoadScenario)
    ~/start_scenario     (std_srvs/srv/Trigger)
    ~/stop_scenario      (std_srvs/srv/Trigger)
    ~/get_scenario_state (hils_bridge_interfaces/srv/GetScenarioState)

Parameters:
    scenario_file: YAML scenario to load at startup ('' = none).
    autostart: start the scenario as soon as the target services appear.
    tick_hz: event scheduler tick rate (default 200 Hz -> 5 ms).

Every event is logged with planned and actual firing time so reports
can verify injection timing accuracy (docs sections 17.2, 17.6).
"""

import yaml

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Trigger

from hils_bridge_interfaces.srv import (
    ClearFault, GetScenarioState, InjectFault, LoadScenario, SetDeviceState,
)

from hils_bridge_base.scenario.scenario_loader import (
    Scenario, ScenarioError, load_scenario_file, load_scenario_yaml,
)
from hils_bridge_base.scenario.event_scheduler import (
    EventSchedule, FAILED, SUCCEEDED,
)

IDLE = 'idle'
LOADED = 'loaded'
WAITING = 'waiting_for_target'
RUNNING = 'running'
FINISHED = 'finished'
STOPPED = 'stopped'


class ScenarioRunnerNode(Node):
    """Executes fault scenarios against an emulator's fault services."""

    def __init__(self):
        super().__init__('hils_scenario_runner')

        self.declare_parameter('scenario_file', '',
            ParameterDescriptor(description='YAML scenario to load at startup.'))
        self.declare_parameter('autostart', True,
            ParameterDescriptor(description='Start once target services appear.'))
        self.declare_parameter('tick_hz', 200.0,
            ParameterDescriptor(description='Event scheduler tick rate [Hz].'))

        self._scenario: Scenario = None
        self._schedule: EventSchedule = None
        self._phase = IDLE
        self._start_time = None
        self._inject_client = None
        self._clear_client = None
        self._tick_timer = None
        self._wait_timer = None

        self.create_service(LoadScenario, '~/load_scenario',
                            self._handle_load)
        self.create_service(Trigger, '~/start_scenario', self._handle_start)
        self.create_service(Trigger, '~/stop_scenario', self._handle_stop)
        self.create_service(GetScenarioState, '~/get_scenario_state',
                            self._handle_get_state)

        scenario_file = self.get_parameter('scenario_file').value
        if scenario_file:
            try:
                self._load(load_scenario_file(scenario_file))
            except (ScenarioError, OSError) as e:
                self.get_logger().error(f'Cannot load {scenario_file}: {e}')
                raise
            if self.get_parameter('autostart').value:
                self._start_when_target_ready()

    # -- loading --

    def _load(self, scenario: Scenario):
        if self._phase == RUNNING:
            raise ScenarioError('scenario is running; stop it first')
        self._scenario = scenario
        self._schedule = EventSchedule.from_events(scenario.events)
        self._phase = LOADED
        self._inject_client = self.create_client(
            InjectFault, f'{scenario.target}/inject_fault')
        self._clear_client = self.create_client(
            ClearFault, f'{scenario.target}/clear_fault')
        self._state_client = self.create_client(
            SetDeviceState, f'{scenario.target}/set_device_state')
        self._needs_state_client = any(
            e.action == 'set_device_state' for e in scenario.events)
        if scenario.expectations:
            self.get_logger().warning(
                f'{len(scenario.expectations)} expectation(s) parsed but not '
                f'evaluated (test oracle is a later phase)')
        self.get_logger().info(
            f'Scenario loaded: id={scenario.scenario_id} '
            f'target={scenario.target} seed={scenario.seed} '
            f'events={len(scenario.events)} '
            f'duration={scenario.duration_sec:.1f}s')

    def _handle_load(self, request, response):
        if bool(request.scenario_path) == bool(request.scenario_yaml):
            response.success = False
            response.message = \
                'set exactly one of scenario_path / scenario_yaml'
            return response
        try:
            if request.scenario_path:
                scenario = load_scenario_file(request.scenario_path)
            else:
                scenario = load_scenario_yaml(request.scenario_yaml)
            self._load(scenario)
        except (ScenarioError, OSError) as e:
            self.get_logger().warning(f'scenario rejected: {e}')
            response.success = False
            response.message = str(e)
            return response
        response.success = True
        response.message = 'ok'
        response.scenario_id = scenario.scenario_id
        return response

    # -- start / stop --

    def _start_when_target_ready(self):
        self._phase = WAITING
        self.get_logger().info(
            f'Waiting for fault services of {self._scenario.target} ...')
        self._wait_timer = self.create_timer(0.2, self._check_target_ready)

    def _target_ready(self) -> bool:
        ready = (self._inject_client.service_is_ready()
                 and self._clear_client.service_is_ready())
        if self._needs_state_client:
            ready = ready and self._state_client.service_is_ready()
        return ready

    def _check_target_ready(self):
        if self._target_ready():
            self._cancel_wait_timer()
            self._start()

    def _cancel_wait_timer(self):
        if self._wait_timer is not None:
            self._wait_timer.cancel()
            self.destroy_timer(self._wait_timer)
            self._wait_timer = None

    def _start(self):
        self._schedule = EventSchedule.from_events(self._scenario.events)
        self._start_time = self.get_clock().now()
        self._phase = RUNNING
        tick_hz = self.get_parameter('tick_hz').value
        self._tick_timer = self.create_timer(1.0 / tick_hz, self._tick)
        self.get_logger().info(
            f'[scenario_event] started: id={self._scenario.scenario_id}')

    def _handle_start(self, request, response):
        if self._scenario is None:
            response.success = False
            response.message = 'no scenario loaded'
            return response
        if self._phase == RUNNING:
            response.success = False
            response.message = 'already running'
            return response
        self._cancel_wait_timer()
        if not self._target_ready():
            response.success = False
            response.message = \
                f'fault services of {self._scenario.target} not available'
            return response
        self._start()
        response.success = True
        response.message = f'started {self._scenario.scenario_id}'
        return response

    def _handle_stop(self, request, response):
        if self._phase not in (RUNNING, WAITING):
            response.success = False
            response.message = f'not running (phase={self._phase})'
            return response
        self._teardown(STOPPED)
        # Leave the target in a clean state (docs section 17.5).
        if self._clear_client.service_is_ready():
            self._clear_client.call_async(ClearFault.Request())
            self.get_logger().info(
                '[scenario_event] stop: clear_all sent to target')
        response.success = True
        response.message = 'stopped'
        return response

    def _teardown(self, phase: str):
        if self._tick_timer is not None:
            self._tick_timer.cancel()
            self.destroy_timer(self._tick_timer)
            self._tick_timer = None
        self._cancel_wait_timer()
        self._phase = phase

    # -- event execution --

    def _elapsed_sec(self) -> float:
        return (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

    def _tick(self):
        elapsed = self._elapsed_sec()
        for tracked in self._schedule.due(elapsed):
            self._dispatch(tracked, elapsed)
        if self._phase == RUNNING and self._schedule.pending_count == 0 \
                and self._schedule.in_flight_count == 0:
            self._teardown(FINISHED)
            failed = sum(1 for t in self._schedule.tracked
                         if t.status == FAILED)
            self.get_logger().info(
                f'[scenario_event] finished: id={self._scenario.scenario_id} '
                f'events={len(self._schedule.tracked)} failed={failed}')

    def _dispatch(self, tracked, elapsed: float):
        event = tracked.event
        self.get_logger().info(
            f'[scenario_event] dispatch: index={event.index} '
            f'action={event.action} planned={event.at_sec:.3f}s '
            f'actual={elapsed:.3f}s')

        if event.action == 'inject_fault':
            request = InjectFault.Request()
            request.fault_yaml = yaml.safe_dump(event.fault, sort_keys=False)
            future = self._inject_client.call_async(request)
        elif event.action == 'clear_fault':
            request = ClearFault.Request()
            request.fault_id = event.fault_id
            future = self._clear_client.call_async(request)
        elif event.action == 'set_device_state':
            request = SetDeviceState.Request()
            request.state = event.state
            future = self._state_client.call_async(request)
        else:  # clear_all_faults
            future = self._clear_client.call_async(ClearFault.Request())

        future.add_done_callback(
            lambda fut, t=tracked: self._on_response(t, fut))

    def _on_response(self, tracked, future):
        try:
            result = future.result()
        except Exception as e:  # noqa: BLE001 - service call failed
            tracked.status = FAILED
            tracked.result = f'service call failed: {e}'
            self.get_logger().error(
                f'[scenario_event] event {tracked.event.index}: '
                f'{tracked.result}')
            return
        tracked.status = SUCCEEDED if result.success else FAILED
        tracked.result = result.message
        tracked.fault_id = getattr(result, 'fault_id', '')
        line = (f'[scenario_event] event {tracked.event.index} '
                f'{tracked.status}: {result.message}')
        # rclpy loggers pin the severity per call site, so keep
        # separate statements for info and error.
        if result.success:
            self.get_logger().info(line)
        else:
            self.get_logger().error(line)

    # -- state --

    def _handle_get_state(self, request, response):
        state = {'phase': self._phase}
        if self._scenario is not None:
            state['scenario_id'] = self._scenario.scenario_id
            state['target'] = self._scenario.target
            state['seed'] = self._scenario.seed
        if self._phase == RUNNING:
            state['elapsed_sec'] = round(self._elapsed_sec(), 3)
        if self._schedule is not None:
            state['events'] = self._schedule.snapshot()
        response.state_yaml = yaml.safe_dump(state, sort_keys=False)
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ScenarioRunnerNode()
    except Exception:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
