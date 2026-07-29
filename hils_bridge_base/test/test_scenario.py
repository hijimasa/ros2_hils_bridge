"""Unit tests for scenario loading and event scheduling (no ROS required)."""

import pytest

from hils_bridge_base.scenario import (
    EventSchedule, ScenarioError, load_scenario_dict, load_scenario_yaml,
)
from hils_bridge_base.scenario.event_scheduler import FAILED, SUCCEEDED

VALID = {
    'scenario': {
        'id': 'velodyne_drop_001',
        'description': 'drop test',
        'target': '/hils_velodyne_emulator',
        'seed': 1001,
    },
    'events': [
        {'at_sec': 15.0, 'action': 'clear_all_faults'},
        {'at_sec': 5.0, 'action': 'inject_fault',
         'fault': {'fault_type': 'drop', 'target': 'data',
                   'parameters': {'probability': 1.0}}},
    ],
}


def _valid():
    import copy
    return copy.deepcopy(VALID)


# -- loader --

def test_load_valid_scenario_sorts_events():
    scenario = load_scenario_dict(_valid())
    assert scenario.scenario_id == 'velodyne_drop_001'
    assert scenario.target == '/hils_velodyne_emulator'
    assert [e.at_sec for e in scenario.events] == [5.0, 15.0]
    assert scenario.duration_sec == 15.0


def test_scenario_seed_propagates_to_fault_specs():
    scenario = load_scenario_dict(_valid())
    inject = next(e for e in scenario.events if e.action == 'inject_fault')
    assert inject.fault['seed'] == 1001


def test_explicit_fault_seed_wins():
    doc = _valid()
    doc['events'][1]['fault']['seed'] = 7
    scenario = load_scenario_dict(doc)
    inject = next(e for e in scenario.events if e.action == 'inject_fault')
    assert inject.fault['seed'] == 7


def test_load_yaml_text():
    scenario = load_scenario_yaml(
        'scenario: {id: s1, target: /emu}\n'
        'events:\n'
        '  - {at_sec: 1.0, action: clear_all_faults}\n')
    assert scenario.scenario_id == 's1'


@pytest.mark.parametrize('mutate,match', [
    (lambda d: d['scenario'].pop('id'), 'scenario.id'),
    (lambda d: d['scenario'].update(target='relative'), 'scenario.target'),
    (lambda d: d.update(events=[]), 'events'),
    (lambda d: d['events'][1].update(action='explode'), 'action'),
    (lambda d: d['events'][1].update(at_sec=-1), 'at_sec'),
    (lambda d: d['events'][1].pop('fault'), 'fault'),
    (lambda d: d['events'][1]['fault']['parameters'].update(probability=9),
     'probability'),
    (lambda d: d['events'][1]['fault'].update(fault_type='explode'),
     'fault_type'),
    (lambda d: d['events'][0].update(typo=1), 'unknown key'),
    (lambda d: d.update(oracle=[]), 'unknown key'),
])
def test_invalid_scenarios_rejected(mutate, match):
    doc = _valid()
    mutate(doc)
    with pytest.raises(ScenarioError, match=match):
        load_scenario_dict(doc)


def test_clear_fault_requires_fault_id():
    doc = _valid()
    doc['events'].append({'at_sec': 20.0, 'action': 'clear_fault'})
    with pytest.raises(ScenarioError, match='fault_id'):
        load_scenario_dict(doc)


# -- scheduler --

def test_schedule_fires_events_once_in_order():
    scenario = load_scenario_dict(_valid())
    schedule = EventSchedule.from_events(scenario.events)
    assert schedule.due(1.0) == []
    fired = schedule.due(5.002)
    assert [t.event.at_sec for t in fired] == [5.0]
    assert fired[0].actual_sec == 5.002
    assert schedule.due(5.01) == []  # not fired twice
    assert schedule.pending_count == 1
    fired = schedule.due(30.0)
    assert [t.event.at_sec for t in fired] == [15.0]
    assert schedule.pending_count == 0


def test_schedule_done_tracking():
    scenario = load_scenario_dict(_valid())
    schedule = EventSchedule.from_events(scenario.events)
    for tracked in schedule.due(100.0):
        tracked.status = SUCCEEDED
    assert schedule.all_done
    snapshot = schedule.snapshot()
    assert all(item['status'] == 'succeeded' for item in snapshot)
    assert snapshot[0]['planned_sec'] == 5.0


def test_schedule_snapshot_reports_failures():
    scenario = load_scenario_dict(_valid())
    schedule = EventSchedule.from_events(scenario.events)
    fired = schedule.due(100.0)
    fired[0].status = FAILED
    fired[0].result = 'boom'
    fired[1].status = SUCCEEDED
    assert schedule.all_done
    assert schedule.snapshot()[0]['result'] == 'boom'
