"""Expectation evaluators for the test oracle (docs sections 6.4, 12).

Independent of rclpy: evaluators receive topic arrival times and node
graph observations in scenario-relative seconds and return Verdict
objects. This keeps the pass/fail semantics unit-testable without ROS.

Supported expectation types (scenario YAML `expectations:` entries):

    - type: topic_timeout        # stream goes silent after an event
      topic: /livox/lidar
      within_sec: 2.0            # silence must begin within this time
      after_event_sec: 10.0      # reference time (default: first event)
      min_gap_sec: 1.0           # gap length that counts as "silent"

    - type: topic_resume         # stream comes back after an event
      topic: /livox/lidar
      within_sec: 10.0
      after_event_sec: 20.0

    - type: topic_alive          # stream never has a long gap
      topic: /ouster/points
      max_gap_sec: 2.0
      from_sec: 0.0              # optional evaluation window
      to_sec: null

    - type: node_alive           # DDS node visible in the target graph
      node: livox_driver_node    # substring match on full node name
      expected: true
      (alias: process_alive - a vanished participant is the observable
       footprint of a dead process; a hung process keeps its node alive
       but trips topic expectations instead)

Unknown types are reported as SKIPPED, never as failures, so scenario
files may carry expectations for oracle features that ship later.
"""

from dataclasses import dataclass, field
from typing import List, Optional

PASS = 'pass'
FAIL = 'fail'
SKIP = 'skip'
ERROR = 'error'

SUPPORTED_TYPES = ('topic_timeout', 'topic_resume', 'topic_alive',
                   'node_alive', 'process_alive')


@dataclass
class Verdict:
    index: int
    exp_type: str
    status: str
    detail: str
    expectation: dict = field(default_factory=dict)

    @property
    def passed(self) -> bool:
        return self.status in (PASS, SKIP)


def _num(exp, key, default):
    value = exp.get(key, default)
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{key} must be a number, got {value!r}')
    return float(value)


def first_silence(arrivals: List[float], ref: float, min_gap: float,
                  t_end: float) -> Optional[float]:
    """Scenario time when the stream first goes silent for >= min_gap
    at or after ref, or None if it never does before t_end."""
    after = sorted(t for t in arrivals if t >= ref)
    prev = ref
    for t in after:
        if t - prev >= min_gap:
            return prev
        prev = t
    if t_end - prev >= min_gap:
        return prev
    return None


def eval_topic_timeout(exp: dict, arrivals: List[float], ref: float,
                       t_end: float) -> tuple:
    within = _num(exp, 'within_sec', 2.0)
    min_gap = _num(exp, 'min_gap_sec', 1.0)
    silent_at = first_silence(arrivals, ref, min_gap, t_end)
    if silent_at is None:
        return FAIL, (f'stream never went silent (>= {min_gap}s gap) '
                      f'after t={ref:.2f}s')
    if silent_at - ref <= within:
        return PASS, (f'silent from t={silent_at:.2f}s '
                      f'({silent_at - ref:.2f}s after reference, '
                      f'limit {within}s)')
    return FAIL, (f'silence began at t={silent_at:.2f}s, '
                  f'{silent_at - ref:.2f}s after reference '
                  f'(limit {within}s)')


def eval_topic_resume(exp: dict, arrivals: List[float], ref: float,
                      t_end: float) -> tuple:
    within = _num(exp, 'within_sec', 10.0)
    # Strictly after: a message at exactly the reference instant is the
    # tail of the pre-fault stream, not a resume.
    after = [t for t in arrivals if t > ref]
    if not after:
        return FAIL, f'no message after t={ref:.2f}s (limit {within}s)'
    first = min(after)
    if first - ref <= within:
        return PASS, (f'resumed at t={first:.2f}s '
                      f'({first - ref:.2f}s after reference, '
                      f'limit {within}s)')
    return FAIL, (f'first message at t={first:.2f}s, '
                  f'{first - ref:.2f}s after reference (limit {within}s)')


def eval_topic_alive(exp: dict, arrivals: List[float], t_start: float,
                     t_end: float) -> tuple:
    max_gap = _num(exp, 'max_gap_sec', 2.0)
    lo = _num(exp, 'from_sec', t_start)
    hi = _num(exp, 'to_sec', None)
    hi = t_end if hi is None else hi
    window = sorted(t for t in arrivals if lo <= t <= hi)
    if not window:
        return FAIL, f'no messages in window [{lo:.2f}, {hi:.2f}]s'
    points = [lo] + window + [hi]
    worst = max(b - a for a, b in zip(points, points[1:]))
    if worst <= max_gap:
        return PASS, (f'{len(window)} messages, worst gap '
                      f'{worst:.2f}s (limit {max_gap}s)')
    return FAIL, f'gap of {worst:.2f}s exceeds limit {max_gap}s'


def eval_node_alive(exp: dict, node_names: List[str]) -> tuple:
    needle = exp.get('node')
    if not isinstance(needle, str) or not needle:
        raise ValueError('node must be a non-empty string')
    expected = exp.get('expected', True)
    if not isinstance(expected, bool):
        raise ValueError('expected must be a boolean')
    found = [n for n in node_names if needle in n]
    alive = bool(found)
    if alive == expected:
        return PASS, (f'node {found[0]!r} present' if alive
                      else f'no node matching {needle!r} (as expected)')
    return FAIL, (f'node matching {needle!r} '
                  + ('present but expected absent'
                     if alive else 'not found in graph'))


def evaluate(expectations: List[dict], *, arrivals_by_topic: dict,
             node_names: List[str], default_ref: float,
             t_start: float, t_end: float) -> List[Verdict]:
    """Evaluate all expectations against recorded observations.

    Args:
        expectations: raw expectation dicts from the scenario.
        arrivals_by_topic: {topic: [scenario-relative arrival times]}.
        node_names: node names observed in the target graph near the
            end of the scenario.
        default_ref: reference time for expectations without
            after_event_sec (the first event's actual time).
        t_start, t_end: observed scenario window.
    """
    verdicts = []
    for i, exp in enumerate(expectations):
        exp_type = exp.get('type') if isinstance(exp, dict) else None
        try:
            if exp_type in ('topic_timeout', 'topic_resume', 'topic_alive'):
                topic = exp.get('topic')
                if not isinstance(topic, str) or not topic:
                    raise ValueError('topic must be a non-empty string')
                arrivals = arrivals_by_topic.get(topic, [])
                ref = _num(exp, 'after_event_sec', default_ref)
                if exp_type == 'topic_timeout':
                    status, detail = eval_topic_timeout(
                        exp, arrivals, ref, t_end)
                elif exp_type == 'topic_resume':
                    status, detail = eval_topic_resume(
                        exp, arrivals, ref, t_end)
                else:
                    status, detail = eval_topic_alive(
                        exp, arrivals, t_start, t_end)
            elif exp_type in ('node_alive', 'process_alive'):
                status, detail = eval_node_alive(exp, node_names)
            elif exp_type is None:
                status, detail = ERROR, 'expectation has no "type" key'
            else:
                status, detail = SKIP, \
                    f'type {exp_type!r} not supported by this oracle yet'
        except ValueError as e:
            status, detail = ERROR, str(e)
        verdicts.append(Verdict(index=i, exp_type=str(exp_type),
                                status=status, detail=detail,
                                expectation=dict(exp) if isinstance(exp, dict)
                                else {'raw': exp}))
    return verdicts
