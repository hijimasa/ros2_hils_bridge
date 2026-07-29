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
            t_end=30.0):
    verdicts = evaluate(
        [exp],
        arrivals_by_topic={'/scan': list(arrivals or [])},
        node_names=list(node_names),
        default_ref=default_ref, t_start=0.0, t_end=t_end)
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


# -- robustness --

def test_unknown_type_skipped_not_failed():
    v = run_one({'type': 'diagnostic_level', 'expected': 'error'})
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
         {'type': 'diagnostic_level'}],
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
