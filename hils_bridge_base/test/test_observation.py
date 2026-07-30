"""Unit tests for the oracle expectation evaluators and report writers
(no ROS required)."""

import json
import xml.etree.ElementTree as ET

import pytest

from hils_bridge_base.observation.expectations import (
    ERROR, FAIL, PASS, SKIP, evaluate, first_silence,
)
from hils_bridge_base.reporting import (
    build_report, write_json, write_junit_xml,
)


def stream(start, stop, hz=10.0):
    """Arrivals at hz between start and stop."""
    step = 1.0 / hz
    out = []
    t = start
    while t <= stop:
        out.append(round(t, 3))
        t += step
    return out


# 30 s scenario: stream runs 0-10 s, silent 10-20 s, resumes 20-30 s
INTERRUPTED = stream(0.0, 10.0) + stream(20.0, 30.0)


def run_one(exp, arrivals=None, node_names=(), default_ref=10.0,
            t_end=30.0, ages=None, diagnostics=None):
    verdicts = evaluate(
        [exp],
        arrivals_by_topic={'/scan': list(arrivals or [])},
        node_names=list(node_names),
        default_ref=default_ref, t_start=0.0, t_end=t_end,
        ages_by_topic={'/scan': list(ages or [])},
        diagnostics=list(diagnostics or []))
    return verdicts[0]


# -- first_silence --

def test_first_silence_finds_gap():
    assert first_silence(INTERRUPTED, 10.0, 1.0, 30.0) == pytest.approx(10.0)


def test_first_silence_none_when_continuous():
    assert first_silence(stream(0, 30), 10.0, 1.0, 30.0) is None


def test_first_silence_open_ended():
    # Stream just stops at 10 s and never resumes.
    assert first_silence(stream(0, 10), 5.0, 1.0, 30.0) == pytest.approx(10.0)


# -- topic_timeout --

def test_timeout_pass_when_stream_stops_quickly():
    v = run_one({'type': 'topic_timeout', 'topic': '/scan',
                 'within_sec': 2.0}, INTERRUPTED)
    assert v.status == PASS


def test_timeout_fail_when_stream_never_stops():
    v = run_one({'type': 'topic_timeout', 'topic': '/scan',
                 'within_sec': 2.0}, stream(0, 30))
    assert v.status == FAIL


def test_timeout_fail_when_stop_is_late():
    # Reference at 2 s but the stream keeps going until 10 s.
    v = run_one({'type': 'topic_timeout', 'topic': '/scan',
                 'within_sec': 2.0, 'after_event_sec': 2.0}, INTERRUPTED)
    assert v.status == FAIL


# -- topic_resume --

def test_resume_pass():
    v = run_one({'type': 'topic_resume', 'topic': '/scan',
                 'within_sec': 5.0, 'after_event_sec': 18.0}, INTERRUPTED)
    assert v.status == PASS
    assert '20.0' in v.detail


def test_resume_fail_too_late():
    v = run_one({'type': 'topic_resume', 'topic': '/scan',
                 'within_sec': 5.0, 'after_event_sec': 10.0}, INTERRUPTED)
    assert v.status == FAIL


def test_resume_fail_never():
    v = run_one({'type': 'topic_resume', 'topic': '/scan',
                 'within_sec': 5.0, 'after_event_sec': 12.0}, stream(0, 10))
    assert v.status == FAIL


# -- topic_alive --

def test_alive_pass_continuous():
    v = run_one({'type': 'topic_alive', 'topic': '/scan',
                 'max_gap_sec': 1.0}, stream(0, 30))
    assert v.status == PASS


def test_alive_fail_on_gap():
    v = run_one({'type': 'topic_alive', 'topic': '/scan',
                 'max_gap_sec': 2.0}, INTERRUPTED)
    assert v.status == FAIL
    assert '10.0' in v.detail  # the 10 s gap is reported


def test_alive_window_excludes_gap():
    v = run_one({'type': 'topic_alive', 'topic': '/scan',
                 'max_gap_sec': 2.0, 'from_sec': 20.0, 'to_sec': 30.0},
                INTERRUPTED)
    assert v.status == PASS


# -- node_alive / process_alive --

def test_node_alive_pass_substring():
    v = run_one({'type': 'process_alive', 'node': 'livox_driver'},
                node_names=['/livox_driver_node', '/rviz2'])
    assert v.status == PASS


def test_node_alive_fail_missing():
    v = run_one({'type': 'node_alive', 'node': 'velodyne'},
                node_names=['/livox_driver_node'])
    assert v.status == FAIL


def test_node_alive_expected_false():
    v = run_one({'type': 'node_alive', 'node': 'velodyne',
                 'expected': False}, node_names=['/livox_driver_node'])
    assert v.status == PASS


# -- maximum_message_age --

def test_max_age_pass():
    ages = [(t, 0.05) for t in stream(0, 30, hz=2)]
    v = run_one({'type': 'maximum_message_age', 'topic': '/scan',
                 'threshold_ms': 200}, ages=ages)
    assert v.status == PASS


def test_max_age_fail_reports_worst():
    ages = [(t, 0.05) for t in stream(0, 30, hz=2)] + [(12.0, 0.35)]
    v = run_one({'type': 'maximum_message_age', 'topic': '/scan',
                 'threshold_ms': 200}, ages=ages)
    assert v.status == FAIL
    assert '350.0ms' in v.detail


def test_max_age_window():
    ages = [(5.0, 0.5), (25.0, 0.05)]
    v = run_one({'type': 'maximum_message_age', 'topic': '/scan',
                 'threshold_ms': 200, 'from_sec': 20.0}, ages=ages)
    assert v.status == PASS


def test_max_age_requires_threshold():
    v = run_one({'type': 'maximum_message_age', 'topic': '/scan'},
                ages=[(1.0, 0.01)])
    assert v.status == ERROR


# -- diagnostic_level --

DIAG = [
    (5.0, 'livox_ros_driver2: lidar status|mid360', 0),
    (11.5, 'livox_ros_driver2: lidar status|mid360', 2),
    (12.0, 'other_node: temp|x', 1),
]


def test_diag_level_pass():
    v = run_one({'type': 'diagnostic_level', 'node': 'livox_ros_driver2',
                 'expected': 'error', 'within_sec': 3.0},
                diagnostics=DIAG)  # default_ref=10.0
    assert v.status == PASS
    assert '11.50' in v.detail


def test_diag_level_fail_too_late():
    v = run_one({'type': 'diagnostic_level', 'node': 'livox_ros_driver2',
                 'expected': 'error', 'within_sec': 1.0},
                diagnostics=DIAG)
    assert v.status == FAIL


def test_diag_level_warn_or_error_matches_error():
    v = run_one({'type': 'diagnostic_level', 'node': 'livox',
                 'expected': 'warn_or_error', 'within_sec': 3.0},
                diagnostics=DIAG)
    assert v.status == PASS


def test_diag_level_no_matching_status():
    v = run_one({'type': 'diagnostic_level', 'node': 'velodyne',
                 'expected': 'error'}, diagnostics=DIAG)
    assert v.status == FAIL
    assert 'no diagnostic status' in v.detail


def test_diag_level_bad_expected():
    v = run_one({'type': 'diagnostic_level', 'node': 'livox',
                 'expected': 'catastrophic'}, diagnostics=DIAG)
    assert v.status == ERROR


# -- robustness --

def test_unknown_type_skipped_not_failed():
    v = run_one({'type': 'crystal_ball_prediction', 'topic': '/fix'})
    assert v.status == SKIP
    assert v.passed


def test_missing_type_is_error():
    v = run_one({'topic': '/scan'})
    assert v.status == ERROR
    assert not v.passed


def test_bad_parameter_is_error():
    v = run_one({'type': 'topic_resume', 'topic': '/scan',
                 'within_sec': 'soon'})
    assert v.status == ERROR


# -- reporting --

def make_report(tmp_path):
    verdicts = evaluate(
        [{'type': 'topic_resume', 'topic': '/scan', 'within_sec': 5.0,
          'after_event_sec': 18.0},
         {'type': 'topic_resume', 'topic': '/scan', 'within_sec': 1.0,
          'after_event_sec': 10.0},
         {'type': 'crystal_ball_prediction', 'topic': '/fix'}],
        arrivals_by_topic={'/scan': INTERRUPTED},
        node_names=[], default_ref=10.0, t_start=0.0, t_end=30.0)
    return build_report(
        scenario={'id': 'unit_scenario', 'seed': 1},
        verdicts=verdicts,
        observations={'window_sec': 30.0},
        environment={'ros_distro': 'test'})


def test_report_summary_counts(tmp_path):
    report = make_report(tmp_path)
    assert report['summary'] == {
        'pass': 1, 'fail': 1, 'skip': 1, 'error': 0,
        'total': 3, 'passed': False}


def test_json_roundtrip(tmp_path):
    report = make_report(tmp_path)
    path = str(tmp_path / 'r.json')
    write_json(report, path)
    with open(path) as f:
        loaded = json.load(f)
    assert loaded['scenario']['id'] == 'unit_scenario'
    assert len(loaded['expectations']) == 3


def test_junit_xml_structure(tmp_path):
    report = make_report(tmp_path)
    path = str(tmp_path / 'r.xml')
    write_junit_xml(report, path)
    root = ET.parse(path).getroot()
    assert root.tag == 'testsuite'
    assert root.get('tests') == '3'
    assert root.get('failures') == '1'
    assert root.get('skipped') == '1'
    cases = root.findall('testcase')
    assert len(cases) == 3
    assert cases[1].find('failure') is not None


# -- invalid_message_not_published --

def _eval_content(exp, samples, t_end=30.0):
    exp = {'type': 'invalid_message_not_published',
           'topic': '/fix', 'field': 'latitude', **exp}
    return evaluate([exp], arrivals_by_topic={}, node_names=[],
                    default_ref=10.0, t_start=0.0, t_end=t_end,
                    contents_by_field={('/fix', 'latitude'): samples})[0]


def test_content_pass_within_bounds():
    v = _eval_content({'min': -90.0, 'max': 90.0},
                      [(1.0, 35.6), (2.0, 35.7), (3.0, -10.0)])
    assert v.status == PASS
    assert '3 messages' in v.detail


def test_content_fail_above_max():
    v = _eval_content({'min': -90.0, 'max': 90.0},
                      [(1.0, 35.6), (12.5, 123.4)])
    assert v.status == FAIL
    assert 't=12.50s' in v.detail and 'max' in v.detail


def test_content_fail_below_min():
    v = _eval_content({'min': 0.0}, [(5.0, -1.0)])
    assert v.status == FAIL


def test_content_nan_invalid_by_default():
    v = _eval_content({}, [(1.0, float('nan'))])
    assert v.status == FAIL
    assert 'non-finite' in v.detail


def test_content_nan_allowed_when_opted_in():
    v = _eval_content({'allow_nan': True},
                      [(1.0, float('nan')), (2.0, 1.0)])
    assert v.status == PASS


def test_content_non_numeric_fails():
    v = _eval_content({}, [(1.0, 'abc')])
    assert v.status == FAIL
    assert 'non-numeric' in v.detail


def test_content_empty_window_passes():
    v = _eval_content({'min': 0.0}, [])
    assert v.status == PASS
    assert 'no messages' in v.detail


def test_content_window_filters_samples():
    # invalid sample outside [from_sec, to_sec] must be ignored
    v = _eval_content({'max': 10.0, 'from_sec': 5.0, 'to_sec': 20.0},
                      [(1.0, 999.0), (6.0, 1.0)])
    assert v.status == PASS


def test_content_missing_field_is_error():
    v = _eval_content({}, [(1.0, None)])
    assert v.status == ERROR
    assert 'not found' in v.detail


def test_content_missing_topic_or_field_key_is_error():
    v = evaluate([{'type': 'invalid_message_not_published',
                   'topic': '/fix'}],
                 arrivals_by_topic={}, node_names=[], default_ref=0.0,
                 t_start=0.0, t_end=10.0)[0]
    assert v.status == ERROR


def test_extract_field_dotted_path():
    from hils_bridge_base.observation.expectations import extract_field

    class Leaf:
        z = 3.5

    class Root:
        pose = Leaf()

    assert extract_field(Root(), 'pose.z') == 3.5
    assert extract_field(Root(), 'pose.missing') is None
    assert extract_field(Root(), 'nope') is None
