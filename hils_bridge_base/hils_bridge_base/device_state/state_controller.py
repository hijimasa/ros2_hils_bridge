"""ROS 2 service API for device state control (docs sections 8.1, 11).

Exposes per-node services:
    ~/set_device_state (hils_bridge_interfaces/srv/SetDeviceState)
    ~/get_device_state (hils_bridge_interfaces/srv/GetDeviceState)

Handles the "reboot" pseudo-state: enter REBOOTING, then transition to
the node's reboot_target_state after boot_duration_sec (docs section
7.8: a reboot is more than a transmission gap - the device passes
through its boot sequence and may require reconfiguration).
"""

import threading

import yaml

from rcl_interfaces.msg import ParameterDescriptor

from hils_bridge_interfaces.srv import GetDeviceState, SetDeviceState

from . import state as st
from .state_machine import DeviceStateMachine


class DeviceStateController:
    """Wires a DeviceStateMachine to per-node ROS 2 services."""

    def __init__(self, node, machine: DeviceStateMachine, *,
                 default_reboot_target: str = st.STREAMING):
        self._node = node
        self._machine = machine
        self._boot_timer = None
        self._lock = threading.Lock()

        node.declare_parameter('boot_duration_sec', 2.0,
            ParameterDescriptor(
                description='Time spent in rebooting/booting before the '
                            'device becomes available again.'))
        node.declare_parameter('reboot_target_state', default_reboot_target,
            ParameterDescriptor(
                description='State reached after a reboot completes. '
                            'Devices with discovery/configuration phases '
                            'should use "discoverable" so the driver must '
                            'reconfigure (docs section 7.8).'))

        self._services = [
            node.create_service(SetDeviceState, '~/set_device_state',
                                self._handle_set),
            node.create_service(GetDeviceState, '~/get_device_state',
                                self._handle_get),
        ]

    # -- service handlers --

    def _handle_set(self, request, response):
        requested = request.state.strip().lower()
        self._cancel_boot_timer()

        if requested == st.REBOOT_REQUEST:
            previous = self._machine.set_state(st.REBOOTING)
            duration = self._node.get_parameter('boot_duration_sec').value
            target = self._node.get_parameter('reboot_target_state').value
            if not st.is_valid_state(target):
                response.success = False
                response.message = \
                    f'invalid reboot_target_state parameter: {target!r}'
                return response
            self._arm_boot_timer(float(duration), target)
            self._log_event(
                f'reboot requested: previous={previous} '
                f'boot_duration_sec={duration} target={target}')
        else:
            if not st.is_valid_state(requested):
                response.success = False
                response.message = (
                    f'invalid state {requested!r}, valid: '
                    f'{list(st.ALL_STATES)} or "{st.REBOOT_REQUEST}"')
                return response
            previous = self._machine.set_state(requested)
            self._log_event(
                f'state set: {previous} -> {requested}')

        response.success = True
        response.message = 'ok'
        response.previous_state = previous
        return response

    def _handle_get(self, request, response):
        snapshot = self._machine.snapshot()
        response.state = snapshot['state']
        response.time_in_state_sec = float(snapshot['time_in_state_sec'])
        response.detail_yaml = yaml.safe_dump(snapshot, sort_keys=False)
        return response

    # -- boot timer --

    def _arm_boot_timer(self, duration: float, target: str):
        def _boot_done():
            self._cancel_boot_timer()
            old = self._machine.set_state(target)
            self._log_event(f'boot complete: {old} -> {target}')

        with self._lock:
            self._boot_timer = self._node.create_timer(
                max(0.001, duration), _boot_done)

    def _cancel_boot_timer(self):
        with self._lock:
            if self._boot_timer is not None:
                self._boot_timer.cancel()
                self._node.destroy_timer(self._boot_timer)
                self._boot_timer = None

    # -- helpers --

    def _log_event(self, message: str):
        stamp = self._node.get_clock().now().nanoseconds * 1e-9
        self._node.get_logger().info(f'[state_event t={stamp:.6f}] {message}')
