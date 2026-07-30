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

    - type: maximum_message_age  # header.stamp vs arrival time
      topic: /imu/data
      threshold_ms: 200
      from_sec: 0.0              # optional evaluation window
      to_sec: null
      NOTE: meaningless when the simulation source replays old
      timestamps (e.g. rosbag) - use only with live simulation clocks.

    - type: diagnostic_level     # /diagnostics reaches a level
      node: livox_ros_driver2    # substring match on status name/hw id
      expected: error            # warn | error | stale | warn_or_error
      within_sec: 3.0
      after_event_sec: 10.0

    - type: invalid_message_not_published
      # Content inspection (docs 12.1: corrupted input must never be
      # republished as valid data): every observed value of `field`
      # must stay inside the given bounds and be finite.
      topic: /fix
      field: latitude            # dotted path into the message
      min: -90.0                 # optional lower bound
      max: 90.0                  # optional upper bound
      allow_nan: false           # NaN/inf count as invalid (default)
      from_sec: 0.0              # optional evaluation window
      to_sec: null
      NOTE: an empty window PASSES - absence of messages is absence of
      invalid data; pair with topic_alive/topic_resume to require
      traffic.

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
                   'node_alive', 'process_alive',
                   'maximum_message_age', 'diagnostic_level',
                   'invalid_message_not_published')

# diagnostic_msgs/DiagnosticStatus levels
_DIAG_LEVELS = {'ok': 0, 'warn': 1, 'warn_or_error': 1, 'error': 2,
                'stale': 3}


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


def eval_max_message_age(exp: dict, ages: List[tuple], t_start: float,
                         t_end: float) -> tuple:
    """ages: [(scenario_time, age_sec), ...] per received message."""
    threshold = _num(exp, 'threshold_ms', None)
    if threshold is None:
        raise ValueError('threshold_ms is required')
    lo = _num(exp, 'from_sec', t_start)
    hi = _num(exp, 'to_sec', None)
    hi = t_end if hi is None else hi
    window = [(t, age) for t, age in ages if lo <= t <= hi]
    if not window:
        return FAIL, f'no stamped messages in window [{lo:.2f}, {hi:.2f}]s'
    worst_t, worst = max(window, key=lambda p: p[1])
    if worst * 1000.0 <= threshold:
        return PASS, (f'{len(window)} messages, max age '
                      f'{worst * 1000.0:.1f}ms (limit {threshold}ms)')
    return FAIL, (f'age {worst * 1000.0:.1f}ms at t={worst_t:.2f}s '
                  f'exceeds limit {threshold}ms')


def eval_diagnostic_level(exp: dict, diagnostics: List[tuple], ref: float,
                          t_end: float) -> tuple:
    """diagnostics: [(scenario_time, status_name, level), ...]."""
    expected = exp.get('expected')
    if expected not in _DIAG_LEVELS or expected == 'ok':
        raise ValueError(
            f'expected must be one of warn, error, stale, warn_or_error; '
            f'got {expected!r}')
    threshold = _DIAG_LEVELS[expected]
    needle = exp.get('node') or exp.get('name') or ''
    if not isinstance(needle, str):
        raise ValueError('node must be a string')
    within = _num(exp, 'within_sec', t_end - ref)
    matched = [(t, name, level) for t, name, level in diagnostics
               if needle in name]
    if not matched:
        return FAIL, (f'no diagnostic status matching {needle!r} observed'
                      if needle else 'no diagnostics observed')
    hits = [(t, name, level) for t, name, level in matched
            if ref <= t <= ref + within and level >= threshold]
    if hits:
        t, name, level = min(hits, key=lambda h: h[0])
        return PASS, (f'{name!r} reached level {level} at t={t:.2f}s '
                      f'({t - ref:.2f}s after reference, limit {within}s)')
    worst = max(level for _, _, level in matched)
    return FAIL, (f'no matching status reached level >= {threshold} in '
                  f'[{ref:.2f}, {ref + within:.2f}]s '
                  f'(worst observed level {worst})')


def extract_field(msg, path: str):
    """Follow a dotted field path into a message-like object.

    Returns the leaf value, or None if any segment is missing (the
    evaluator reports that as a spec error). Lives here rather than in
    the recorder so it stays unit-testable without rclpy.
    """
    value = msg
    for part in path.split('.'):
        value = getattr(value, part, None)
        if value is None:
            return None
    return value


def eval_invalid_not_published(exp: dict, samples: List[tuple],
                               t_start: float, t_end: float) -> tuple:
    """samples: [(scenario_time, value), ...] for (topic, field)."""
    import math

    lo_bound = _num(exp, 'min', None)
    hi_bound = _num(exp, 'max', None)
    allow_nan = exp.get('allow_nan', False)
    if not isinstance(allow_nan, bool):
        raise ValueError('allow_nan must be a boolean')
    lo = _num(exp, 'from_sec', t_start)
    hi = _num(exp, 'to_sec', None)
    hi = t_end if hi is None else hi

    window = [(t, v) for t, v in samples if lo <= t <= hi]
    if not window:
        return PASS, (f'no messages in window [{lo:.2f}, {hi:.2f}]s - '
                      f'no invalid data published')

    missing = [t for t, v in window if v is None]
    if missing:
        raise ValueError(
            f'field {exp.get("field")!r} not found on {exp.get("topic")} '
            f'({len(missing)} of {len(window)} messages)')

    def invalid_reason(v):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            return f'non-numeric value {v!r}'
        if not math.isfinite(v):
            return None if allow_nan else f'non-finite value {v}'
        if lo_bound is not None and v < lo_bound:
            return f'value {v} < min {lo_bound}'
        if hi_bound is not None and v > hi_bound:
            return f'value {v} > max {hi_bound}'
        return None

    for t, v in window:
        reason = invalid_reason(v)
        if reason:
            return FAIL, f'invalid message at t={t:.2f}s: {reason}'
    return PASS, (f'{len(window)} messages inspected, all values valid')


def evaluate(expectations: List[dict], *, arrivals_by_topic: dict,
             node_names: List[str], default_ref: float,
             t_start: float, t_end: float,
             ages_by_topic: dict = None,
             diagnostics: List[tuple] = None,
             contents_by_field: dict = None) -> List[Verdict]:
    """Evaluate all expectations against recorded observations.

    Args:
        expectations: raw expectation dicts from the scenario.
        arrivals_by_topic: {topic: [scenario-relative arrival times]}.
        node_names: node names observed in the target graph near the
            end of the scenario.
        default_ref: reference time for expectations without
            after_event_sec (the first event's actual time).
        t_start, t_end: observed scenario window.
        contents_by_field: {(topic, field): [(scenario_time, value)]}
            for invalid_message_not_published expectations.
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
            elif exp_type == 'maximum_message_age':
                topic = exp.get('topic')
                if not isinstance(topic, str) or not topic:
                    raise ValueError('topic must be a non-empty string')
                ages = (ages_by_topic or {}).get(topic, [])
                status, detail = eval_max_message_age(
                    exp, ages, t_start, t_end)
            elif exp_type == 'diagnostic_level':
                ref = _num(exp, 'after_event_sec', default_ref)
                status, detail = eval_diagnostic_level(
                    exp, diagnostics or [], ref, t_end)
            elif exp_type == 'invalid_message_not_published':
                topic = exp.get('topic')
                field = exp.get('field')
                if not isinstance(topic, str) or not topic:
                    raise ValueError('topic must be a non-empty string')
                if not isinstance(field, str) or not field:
                    raise ValueError('field must be a non-empty string')
                samples = (contents_by_field or {}).get((topic, field), [])
                status, detail = eval_invalid_not_published(
                    exp, samples, t_start, t_end)
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
