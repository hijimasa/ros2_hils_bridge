"""ROS 2 service API for runtime fault injection (docs section 11).

Exposes per-node services:
    ~/inject_fault    (hils_bridge_interfaces/srv/InjectFault)
    ~/clear_fault     (hils_bridge_interfaces/srv/ClearFault)
    ~/get_fault_state (hils_bridge_interfaces/srv/GetFaultState)

Every inject/clear event is logged with the node clock time so test
reports can correlate injection times with observations (docs
section 17.2).
"""

import threading

import yaml

from rcl_interfaces.msg import ParameterDescriptor

from hils_bridge_interfaces.srv import ClearFault, GetFaultState, InjectFault

from . import create_fault
from .fault_base import FaultSpecError
from .pipeline import FaultPipeline
from .sender import DelayedSender

_SPEC_KEYS = ('fault_type', 'fault_id', 'target', 'seed', 'duration_sec',
              'parameters')


class FaultInjectionController:
    """Wires a FaultPipeline to per-node ROS 2 services."""

    def __init__(self, node, pipeline: FaultPipeline,
                 sender: DelayedSender = None):
        self._node = node
        self._pipeline = pipeline
        self._sender = sender
        self._counter = 0
        self._expiry_timers = {}  # fault_id -> rclpy timer
        self._lock = threading.Lock()

        if not node.has_parameter('fault_seed'):
            node.declare_parameter('fault_seed', 0,
                ParameterDescriptor(
                    description='Default random seed for injected faults. '
                                'Recorded per fault for reproducibility.'))

        self._services = [
            node.create_service(InjectFault, '~/inject_fault',
                                self._handle_inject),
            node.create_service(ClearFault, '~/clear_fault',
                                self._handle_clear),
            node.create_service(GetFaultState, '~/get_fault_state',
                                self._handle_get_state),
        ]

    # -- service handlers --

    def _handle_inject(self, request, response):
        try:
            spec = yaml.safe_load(request.fault_yaml)
        except yaml.YAMLError as e:
            return self._fail(response, f'invalid YAML: {e}')
        if not isinstance(spec, dict):
            return self._fail(response, 'fault_yaml must be a YAML mapping')

        unknown = set(spec) - set(_SPEC_KEYS)
        if unknown:
            return self._fail(response, f'unknown key(s): {sorted(unknown)}')

        fault_type = spec.get('fault_type')
        if not isinstance(fault_type, str):
            return self._fail(response, 'fault_type is required')

        duration_sec = spec.get('duration_sec')
        if duration_sec is not None:
            if isinstance(duration_sec, bool) or \
                    not isinstance(duration_sec, (int, float)) or \
                    duration_sec <= 0.0:
                return self._fail(response,
                                  'duration_sec must be a positive number')

        seed = spec.get('seed', self._node.get_parameter('fault_seed').value)
        with self._lock:
            self._counter += 1
            fault_id = spec.get('fault_id') or f'{fault_type}_{self._counter}'

        try:
            fault = create_fault(
                fault_type, fault_id,
                target=spec.get('target'), seed=seed,
                parameters=spec.get('parameters'))
            self._pipeline.add_fault(fault)
        except (FaultSpecError, ValueError) as e:
            return self._fail(response, str(e))

        if duration_sec is not None:
            self._arm_expiry(fault_id, float(duration_sec))

        self._log_event(
            f'fault injected: id={fault_id} type={fault_type} '
            f'target={spec.get("target")} seed={seed} '
            f'parameters={fault.parameters()} duration_sec={duration_sec}')
        response.success = True
        response.message = 'ok'
        response.fault_id = fault_id
        return response

    def _handle_clear(self, request, response):
        if request.fault_id:
            removed = self._pipeline.remove_fault(request.fault_id)
            if not removed:
                return self._fail(
                    response, f'no active fault: {request.fault_id}')
            self._cancel_expiry(request.fault_id)
            self._log_event(f'fault cleared: id={request.fault_id}')
            response.message = 'ok'
        else:
            count = self._pipeline.clear()
            with self._lock:
                for fault_id in list(self._expiry_timers):
                    self._cancel_expiry_locked(fault_id)
            self._log_event(f'all faults cleared: count={count}')
            response.message = f'cleared {count} fault(s)'
        response.success = True
        return response

    def _handle_get_state(self, request, response):
        state = self._pipeline.snapshot()
        if self._sender is not None:
            state['pending_delayed_packets'] = self._sender.pending_count
        response.state_yaml = yaml.safe_dump(state, sort_keys=False)
        return response

    # -- expiry (duration_sec) --

    def _arm_expiry(self, fault_id: str, duration_sec: float):
        def _expire():
            self._cancel_expiry(fault_id)
            if self._pipeline.remove_fault(fault_id):
                self._log_event(
                    f'fault expired: id={fault_id} '
                    f'duration_sec={duration_sec}')

        timer = self._node.create_timer(duration_sec, _expire)
        with self._lock:
            self._expiry_timers[fault_id] = timer

    def _cancel_expiry(self, fault_id: str):
        with self._lock:
            self._cancel_expiry_locked(fault_id)

    def _cancel_expiry_locked(self, fault_id: str):
        timer = self._expiry_timers.pop(fault_id, None)
        if timer is not None:
            timer.cancel()
            self._node.destroy_timer(timer)

    # -- helpers --

    def _fail(self, response, message: str):
        self._node.get_logger().warning(f'fault request rejected: {message}')
        response.success = False
        response.message = message
        return response

    def _log_event(self, message: str):
        stamp = self._node.get_clock().now().nanoseconds * 1e-9
        self._node.get_logger().info(f'[fault_event t={stamp:.6f}] {message}')
