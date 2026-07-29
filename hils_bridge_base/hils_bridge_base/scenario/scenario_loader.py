"""YAML scenario loading and validation (docs sections 6.3, 10).

Independent of rclpy so scenarios can be validated without a ROS 2
environment. Invalid scenarios are rejected at load time, before any
test starts (docs section 18).

Phase 1 event actions:
    inject_fault      - fault: {fault_type, target, parameters, ...}
    clear_fault       - fault_id: <id>
    clear_all_faults  - (no payload)

`expectations` blocks are parsed and preserved for the Phase 6 test
oracle but are not evaluated yet.
"""

from dataclasses import dataclass, field
from typing import List, Optional

import yaml

from hils_bridge_base.fault_injection import create_fault
from hils_bridge_base.fault_injection.fault_base import FaultSpecError

ACTIONS = ('inject_fault', 'clear_fault', 'clear_all_faults')

_EVENT_KEYS = ('at_sec', 'action', 'fault', 'fault_id')
_HEADER_KEYS = ('id', 'description', 'target', 'seed')
_TOP_KEYS = ('scenario', 'events', 'expectations')

_FAULT_SPEC_KEYS = ('fault_type', 'fault_id', 'target', 'seed',
                    'duration_sec', 'parameters')


class ScenarioError(ValueError):
    """Raised when a scenario document is invalid."""


@dataclass
class ScenarioEvent:
    index: int
    at_sec: float
    action: str
    fault: Optional[dict] = None      # inject_fault
    fault_id: Optional[str] = None    # clear_fault


@dataclass
class Scenario:
    scenario_id: str
    description: str
    target: str
    seed: int
    events: List[ScenarioEvent] = field(default_factory=list)
    expectations: List[dict] = field(default_factory=list)

    @property
    def duration_sec(self) -> float:
        return max((e.at_sec for e in self.events), default=0.0)


def load_scenario_file(path: str) -> Scenario:
    with open(path, 'r', encoding='utf-8') as f:
        try:
            doc = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise ScenarioError(f'invalid YAML in {path}: {e}')
    return load_scenario_dict(doc)


def load_scenario_yaml(text: str) -> Scenario:
    try:
        doc = yaml.safe_load(text)
    except yaml.YAMLError as e:
        raise ScenarioError(f'invalid YAML: {e}')
    return load_scenario_dict(doc)


def load_scenario_dict(doc) -> Scenario:
    if not isinstance(doc, dict):
        raise ScenarioError('scenario document must be a YAML mapping')
    _reject_unknown(doc, _TOP_KEYS, 'top level')

    header = doc.get('scenario')
    if not isinstance(header, dict):
        raise ScenarioError('"scenario" header mapping is required')
    _reject_unknown(header, _HEADER_KEYS, 'scenario header')

    scenario_id = header.get('id')
    if not isinstance(scenario_id, str) or not scenario_id:
        raise ScenarioError('scenario.id must be a non-empty string')

    target = header.get('target')
    if not isinstance(target, str) or not target.startswith('/'):
        raise ScenarioError(
            'scenario.target must be an absolute node namespace '
            "(e.g. '/hils_velodyne_emulator')")

    seed = header.get('seed', 0)
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ScenarioError('scenario.seed must be an integer')

    description = header.get('description', '')
    if not isinstance(description, str):
        raise ScenarioError('scenario.description must be a string')

    raw_events = doc.get('events')
    if not isinstance(raw_events, list) or not raw_events:
        raise ScenarioError('"events" must be a non-empty list')

    events = []
    for i, raw in enumerate(raw_events):
        events.append(_parse_event(i, raw, seed))
    events.sort(key=lambda e: (e.at_sec, e.index))

    expectations = doc.get('expectations', [])
    if not isinstance(expectations, list):
        raise ScenarioError('"expectations" must be a list')

    return Scenario(
        scenario_id=scenario_id,
        description=description,
        target=target,
        seed=seed,
        events=events,
        expectations=expectations,
    )


def _parse_event(index: int, raw, scenario_seed: int) -> ScenarioEvent:
    where = f'events[{index}]'
    if not isinstance(raw, dict):
        raise ScenarioError(f'{where} must be a mapping')
    _reject_unknown(raw, _EVENT_KEYS, where)

    at_sec = raw.get('at_sec')
    if isinstance(at_sec, bool) or not isinstance(at_sec, (int, float)) \
            or at_sec < 0.0:
        raise ScenarioError(f'{where}.at_sec must be a number >= 0')

    action = raw.get('action')
    if action not in ACTIONS:
        raise ScenarioError(
            f'{where}.action must be one of {list(ACTIONS)}, got {action!r}')

    fault = None
    fault_id = None
    if action == 'inject_fault':
        fault = raw.get('fault')
        if not isinstance(fault, dict):
            raise ScenarioError(f'{where}.fault mapping is required')
        _reject_unknown(fault, _FAULT_SPEC_KEYS, f'{where}.fault')
        fault = dict(fault)
        fault.setdefault('seed', scenario_seed)
        _validate_fault_spec(fault, where)
    elif action == 'clear_fault':
        fault_id = raw.get('fault_id')
        if not isinstance(fault_id, str) or not fault_id:
            raise ScenarioError(f'{where}.fault_id must be a non-empty string')

    return ScenarioEvent(index=index, at_sec=float(at_sec), action=action,
                         fault=fault, fault_id=fault_id)


def _validate_fault_spec(spec: dict, where: str) -> None:
    """Dry-construct the fault so bad specs fail at load time."""
    fault_type = spec.get('fault_type')
    if not isinstance(fault_type, str):
        raise ScenarioError(f'{where}.fault.fault_type is required')
    duration = spec.get('duration_sec')
    if duration is not None:
        if isinstance(duration, bool) or \
                not isinstance(duration, (int, float)) or duration <= 0.0:
            raise ScenarioError(
                f'{where}.fault.duration_sec must be a positive number')
    try:
        create_fault(
            fault_type, spec.get('fault_id') or f'{where}-dry-run',
            target=spec.get('target'), seed=spec.get('seed', 0),
            parameters=spec.get('parameters'))
    except FaultSpecError as e:
        raise ScenarioError(f'{where}.fault: {e}')


def _reject_unknown(mapping: dict, known, where: str) -> None:
    unknown = set(mapping) - set(known)
    if unknown:
        raise ScenarioError(f'{where}: unknown key(s) {sorted(unknown)}')
