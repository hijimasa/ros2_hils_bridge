"""
hils_bridge_base.device_state - Emulated device state management.

rclpy-independent core:
  state          - state constants and per-state channel policy
  state_machine  - DeviceStateMachine with channel gating and listeners

ROS 2 integration (imported separately to keep the core testable
without ROS):
  state_controller - ~/set_device_state, ~/get_device_state services
"""

from . import state
from .state import (
    ALL_STATES, BOOTING, COMMUNICATION_FAULT, CONFIGURATION_FAULT,
    CONFIGURING, DEGRADED, DISCOVERABLE, INTERNAL_ERROR, POWER_OFF,
    READY, REBOOTING, REBOOT_REQUEST, STREAMING, is_valid_state,
)
from .state_machine import DeviceStateMachine

__all__ = [
    'state', 'DeviceStateMachine', 'is_valid_state',
    'ALL_STATES', 'POWER_OFF', 'BOOTING', 'DISCOVERABLE', 'CONFIGURING',
    'READY', 'STREAMING', 'DEGRADED', 'COMMUNICATION_FAULT',
    'CONFIGURATION_FAULT', 'INTERNAL_ERROR', 'REBOOTING', 'REBOOT_REQUEST',
]
