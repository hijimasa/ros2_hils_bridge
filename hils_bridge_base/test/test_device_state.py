"""Unit tests for the device state machine core (no ROS required)."""

import pytest

from hils_bridge_base.device_state import DeviceStateMachine
from hils_bridge_base.device_state import state as st
from hils_bridge_base.scenario import ScenarioError, load_scenario_dict


def test_initial_state_default_streaming_allows_everything():
    machine = DeviceStateMachine()
    assert machine.state == st.STREAMING
    for channel in ('data', 'position', 'imu', 'discovery', 'command'):
        assert machine.allows(channel)


def test_invalid_states_rejected():
    with pytest.raises(ValueError):
        DeviceStateMachine('exploded')
    machine = DeviceStateMachine()
    with pytest.raises(ValueError):
        machine.set_state('exploded')


def test_power_off_blocks_all_channels():
    machine = DeviceStateMachine()
    machine.set_state(st.POWER_OFF)
    for channel in ('data', 'position', 'discovery', 'command'):
        assert not machine.allows(channel)
    snapshot = machine.snapshot()
    assert snapshot['state'] == st.POWER_OFF
    assert snapshot['suppressed_packets'] == {
        'data': 1, 'position': 1, 'discovery': 1, 'command': 1}


def test_discoverable_allows_discovery_and_command_only():
    machine = DeviceStateMachine(st.DISCOVERABLE)
    assert machine.allows('discovery')
    assert machine.allows('command')
    assert not machine.allows('data')
    assert not machine.allows('imu')


def test_ready_blocks_data_but_allows_status():
    machine = DeviceStateMachine(st.READY)
    assert machine.allows('position')
    assert machine.allows('command')
    assert not machine.allows('data')


def test_is_open_does_not_count():
    machine = DeviceStateMachine(st.POWER_OFF)
    assert not machine.is_open('command')
    assert machine.snapshot()['suppressed_packets'] == {}


def test_listener_called_on_transition_only():
    machine = DeviceStateMachine()
    calls = []
    machine.add_listener(lambda old, new: calls.append((old, new)))
    machine.set_state(st.POWER_OFF)
    machine.set_state(st.POWER_OFF)  # no-op
    machine.set_state(st.REBOOTING)
    assert calls == [(st.STREAMING, st.POWER_OFF),
                     (st.POWER_OFF, st.REBOOTING)]
    assert machine.snapshot()['transition_count'] == 2


def test_channel_policy_override():
    machine = DeviceStateMachine(
        st.READY, channel_policy={st.READY: frozenset({'heartbeat'})})
    assert machine.allows('heartbeat')
    assert not machine.allows('command')


def test_time_in_state_uses_injected_clock():
    now = [100.0]
    machine = DeviceStateMachine(clock=lambda: now[0])
    now[0] = 103.5
    assert machine.time_in_state() == pytest.approx(3.5)
    machine.set_state(st.POWER_OFF)
    now[0] = 104.0
    assert machine.time_in_state() == pytest.approx(0.5)


# -- scenario integration --

def _scenario_with_state_event(state):
    return {
        'scenario': {'id': 's1', 'target': '/emu'},
        'events': [{'at_sec': 1.0, 'action': 'set_device_state',
                    'state': state}],
    }


def test_scenario_set_device_state_parsed():
    scenario = load_scenario_dict(_scenario_with_state_event('power_off'))
    assert scenario.events[0].state == st.POWER_OFF


def test_scenario_accepts_reboot_pseudo_state():
    scenario = load_scenario_dict(_scenario_with_state_event('reboot'))
    assert scenario.events[0].state == st.REBOOT_REQUEST


def test_scenario_rejects_unknown_state():
    with pytest.raises(ScenarioError, match='state'):
        load_scenario_dict(_scenario_with_state_event('exploded'))
