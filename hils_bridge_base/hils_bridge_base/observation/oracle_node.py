#!/usr/bin/env python3
"""Scenario test oracle (docs sections 6.4, 12).

Runs on the simulation PC and evaluates a scenario's `expectations:`
against what is actually observable in the robot PC's ROS domain,
without adding any process to the robot PC:

  - control context (this process's default ROS_DOMAIN_ID): polls the
    scenario runner's ~/get_scenario_state for phase and per-event
    actual times
  - observation context (observe_domain_id = the robot domain): a
    read-only TopicRecorder subscribes to the topics referenced by the
    expectations and samples the node graph

When the scenario finishes and every expectation window has elapsed,
the oracle writes JSON and JUnit XML reports and exits 0 (all pass /
skip) or 1 (any fail / error).

Usage (alongside the scenario runner):
    ros2 run hils_bridge_base scenario_oracle --ros-args \
        -p scenario_file:=/path/scenario.yaml \
        -p observe_domain_id:=43 \
        -p output_dir:=/tmp/hils_reports
"""

import datetime
import os
import signal
import socket as _socket
import subprocess
import threading
import time

import yaml

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from hils_bridge_interfaces.srv import GetScenarioState

from hils_bridge_base.observation.expectations import evaluate
from hils_bridge_base.observation.recorder import TopicRecorder
from hils_bridge_base.reporting import (
    build_report, write_json, write_junit_xml,
)
from hils_bridge_base.scenario import load_scenario_file


class ScenarioOracle(Node):
    def __init__(self):
        super().__init__('hils_scenario_oracle')

        self.declare_parameter('scenario_file', '',
            ParameterDescriptor(description='Scenario YAML to evaluate.'))
        self.declare_parameter('observe_domain_id', 43,
            ParameterDescriptor(
                description='ROS_DOMAIN_ID of the robot PC to observe.'))
        self.declare_parameter('runner_ns', '/hils_scenario_runner',
            ParameterDescriptor(description='Scenario runner namespace.'))
        self.declare_parameter('output_dir', '.',
            ParameterDescriptor(description='Report output directory.'))
        self.declare_parameter('settle_sec', 2.0,
            ParameterDescriptor(
                description='Extra observation time after the last '
                            'expectation window.'))
        self.declare_parameter('max_wait_sec', 300.0,
            ParameterDescriptor(
                description='Abort if the scenario has not finished and '
                            'settled within this time.'))
        self.declare_parameter('record_bag', False,
            ParameterDescriptor(
                description='Record the observed topics with "ros2 bag '
                            'record" in the observed domain (docs 6.5).'))
        self.declare_parameter('record_pcap', False,
            ParameterDescriptor(
                description='Record a PCAP with tcpdump. Needs tcpdump '
                            'and CAP_NET_RAW; failure to start is logged '
                            'and ignored.'))
        self.declare_parameter('pcap_interface', 'any',
            ParameterDescriptor(description='tcpdump capture interface.'))
        self.declare_parameter('pcap_filter', '',
            ParameterDescriptor(
                description='tcpdump capture filter, e.g. '
                            '"udp portrange 56000-56501".'))

        scenario_file = self.get_parameter('scenario_file').value
        if not scenario_file:
            raise RuntimeError('scenario_file parameter is required')
        self.scenario = load_scenario_file(scenario_file)
        self.scenario_file = scenario_file
        if not self.scenario.expectations:
            self.get_logger().warning(
                'scenario has no expectations; report will be empty')

        # Observation context in the robot domain (read-only).
        observe_domain = self.get_parameter('observe_domain_id').value
        exps = [e for e in self.scenario.expectations if isinstance(e, dict)]
        topics = sorted({
            e['topic'] for e in exps if isinstance(e.get('topic'), str)})
        age_topics = sorted({
            e['topic'] for e in exps
            if e.get('type') == 'maximum_message_age'
            and isinstance(e.get('topic'), str)})
        watch_diag = any(e.get('type') == 'diagnostic_level' for e in exps)
        self._obs_context = rclpy.Context()
        rclpy.init(context=self._obs_context, domain_id=observe_domain)
        self._recorder = TopicRecorder(
            topics, context=self._obs_context, age_topics=age_topics,
            watch_diagnostics=watch_diag)
        self._obs_executor = SingleThreadedExecutor(
            context=self._obs_context)
        self._obs_executor.add_node(self._recorder)
        self._obs_thread = threading.Thread(
            target=self._obs_executor.spin, daemon=True,
            name='hils_oracle_observation')
        self._obs_thread.start()
        self.get_logger().info(
            f'observing domain {observe_domain}: topics={topics}')

        # Optional evidence recording (docs 6.5, 20 item 11). Started
        # immediately so the pre-fault baseline is captured too.
        self._bag_proc = None
        self._bag_path = None
        self._pcap_proc = None
        self._pcap_path = None
        self._start_recorders(observe_domain, topics)

        # Runner state polling (control domain).
        runner_ns = self.get_parameter('runner_ns').value
        self._state_client = self.create_client(
            GetScenarioState, f'{runner_ns}/get_scenario_state')
        self._poll_busy = False
        self._start_mono = None       # monotonic time of scenario t=0
        self._runner_events = []      # events list from the runner
        self._finished_seen = False
        self._started_wall = None
        self.exit_code = 2
        self.done = threading.Event()
        self._deadline_mono = time.monotonic() + \
            self.get_parameter('max_wait_sec').value
        self.create_timer(0.5, self._poll)

    # -- evidence recording --

    def _start_recorders(self, observe_domain: int, topics):
        out_dir = self.get_parameter('output_dir').value
        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

        if self.get_parameter('record_bag').value:
            if topics:
                self._bag_path = os.path.join(
                    out_dir, f'{self.scenario.scenario_id}_{stamp}_bag')
                env = dict(os.environ,
                           ROS_DOMAIN_ID=str(observe_domain))
                try:
                    self._bag_proc = subprocess.Popen(
                        ['ros2', 'bag', 'record', '-o', self._bag_path]
                        + list(topics),
                        env=env, stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        start_new_session=True)
                    self.get_logger().info(
                        f'recording bag: {self._bag_path}')
                except OSError as e:
                    self.get_logger().warning(f'bag record failed: {e}')
                    self._bag_path = None
            else:
                self.get_logger().warning(
                    'record_bag requested but no topics to record')

        if self.get_parameter('record_pcap').value:
            self._pcap_path = os.path.join(
                out_dir, f'{self.scenario.scenario_id}_{stamp}.pcap')
            cmd = ['tcpdump', '-i',
                   self.get_parameter('pcap_interface').value,
                   '-w', self._pcap_path]
            pcap_filter = self.get_parameter('pcap_filter').value
            if pcap_filter:
                cmd += pcap_filter.split()
            try:
                self._pcap_proc = subprocess.Popen(
                    cmd, stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL, start_new_session=True)
                self.get_logger().info(
                    f'recording pcap: {self._pcap_path}')
            except OSError as e:
                self.get_logger().warning(
                    f'tcpdump failed to start (missing binary or '
                    f'CAP_NET_RAW?): {e}')
                self._pcap_path = None

    def _stop_recorders(self):
        for proc, sig in ((self._bag_proc, signal.SIGINT),
                          (self._pcap_proc, signal.SIGTERM)):
            if proc is None or proc.poll() is not None:
                continue
            try:
                os.killpg(os.getpgid(proc.pid), sig)
                proc.wait(timeout=10.0)
            except (OSError, subprocess.TimeoutExpired):
                proc.kill()
        self._bag_proc = None
        self._pcap_proc = None

    # -- runner polling --

    def _poll(self):
        if self.done.is_set():
            return
        if time.monotonic() > self._deadline_mono:
            self._finish(timed_out=True)
            return
        if self._poll_busy or not self._state_client.service_is_ready():
            return
        self._poll_busy = True
        future = self._state_client.call_async(GetScenarioState.Request())
        future.add_done_callback(self._on_state)

    def _on_state(self, future):
        self._poll_busy = False
        try:
            state = yaml.safe_load(future.result().state_yaml)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f'runner state unavailable: {e}')
            return
        phase = state.get('phase')
        if phase == 'running' and self._start_mono is None:
            elapsed = float(state.get('elapsed_sec', 0.0))
            self._start_mono = time.monotonic() - elapsed
            self._started_wall = datetime.datetime.now().isoformat()
            self.get_logger().info(
                f'scenario running (elapsed {elapsed:.2f}s), clock locked')
        if isinstance(state.get('events'), list):
            self._runner_events = state['events']
        if phase in ('finished', 'stopped') and not self._finished_seen:
            self._finished_seen = True
            self.get_logger().info(f'runner phase: {phase}')
        if self._finished_seen and self._start_mono is not None:
            now_rel = time.monotonic() - self._start_mono
            if now_rel >= self._required_end():
                self._finish(timed_out=False)

    def _required_end(self) -> float:
        """Scenario time at which every expectation window has closed."""
        end = self.scenario.duration_sec
        default_ref = self._default_ref()
        for exp in self.scenario.expectations:
            if not isinstance(exp, dict):
                continue
            ref = exp.get('after_event_sec', default_ref)
            for key in ('within_sec', 'max_gap_sec', 'to_sec',
                        'min_gap_sec'):
                value = exp.get(key)
                if isinstance(value, (int, float)) and \
                        not isinstance(value, bool):
                    end = max(end, float(ref) + float(value))
        return end + float(self.get_parameter('settle_sec').value)

    def _default_ref(self) -> float:
        for ev in self._runner_events:
            if isinstance(ev, dict) and ev.get('actual_sec') is not None:
                return float(ev['actual_sec'])
        if self.scenario.events:
            return self.scenario.events[0].at_sec
        return 0.0

    # -- evaluation --

    def _finish(self, *, timed_out: bool):
        if self.done.is_set():
            return
        self._stop_recorders()
        t_end = (time.monotonic() - self._start_mono
                 if self._start_mono is not None else 0.0)
        start = self._start_mono or time.monotonic()
        arrivals_rel = {
            topic: [round(t - start, 3) for t in times]
            for topic, times in self._recorder.arrivals().items()}
        ages_rel = {
            topic: [(round(t - start, 3), round(age, 4))
                    for t, age in entries]
            for topic, entries in self._recorder.ages().items()}
        diagnostics_rel = [
            (round(t - start, 3), name, level)
            for t, name, level in self._recorder.diagnostics()]
        node_names = self._recorder.node_names()

        verdicts = evaluate(
            self.scenario.expectations,
            arrivals_by_topic=arrivals_rel,
            node_names=node_names,
            default_ref=self._default_ref(),
            t_start=0.0, t_end=t_end,
            ages_by_topic=ages_rel,
            diagnostics=diagnostics_rel)

        observations = {
            'window_sec': round(t_end, 3),
            'arrival_counts': {t: len(v) for t, v in arrivals_rel.items()},
            'unresolved_topics': self._recorder.unresolved_topics(),
            'node_names': node_names,
            'diagnostic_records': len(diagnostics_rel),
            'runner_events': self._runner_events,
            'timed_out': timed_out,
            'bag_path': self._bag_path,
            'pcap_path': self._pcap_path,
        }
        environment = {
            'scenario_file': self.scenario_file,
            'started_at': self._started_wall,
            'hostname': _socket.gethostname(),
            'ros_distro': os.environ.get('ROS_DISTRO', ''),
            'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION',
                                                 'default'),
            'observe_domain_id':
                self.get_parameter('observe_domain_id').value,
        }
        report = build_report(
            scenario={
                'id': self.scenario.scenario_id,
                'description': self.scenario.description,
                'target': self.scenario.target,
                'seed': self.scenario.seed,
            },
            verdicts=verdicts, observations=observations,
            environment=environment)

        out_dir = self.get_parameter('output_dir').value
        json_path = os.path.join(out_dir,
                                 f'{self.scenario.scenario_id}.json')
        xml_path = os.path.join(out_dir,
                                f'{self.scenario.scenario_id}.xml')
        write_json(report, json_path)
        write_junit_xml(report, xml_path)

        for v in verdicts:
            line = f'[oracle] {v.status.upper():5s} #{v.index} ' \
                   f'{v.exp_type}: {v.detail}'
            # rclpy loggers pin the severity per call site, so keep
            # separate statements for info and error.
            if v.passed:
                self.get_logger().info(line)
            else:
                self.get_logger().error(line)
        summary = report['summary']
        overall = 'PASS' if summary['passed'] and not timed_out else 'FAIL'
        self.get_logger().info(
            f'[oracle] {overall}: {summary["pass"]} pass, '
            f'{summary["fail"]} fail, {summary["skip"]} skip, '
            f'{summary["error"]} error -> {json_path}, {xml_path}')
        self.exit_code = 0 if overall == 'PASS' else 1
        self.done.set()

    # -- teardown --

    def shutdown_observation(self):
        self._stop_recorders()
        self._obs_executor.shutdown(timeout_sec=2.0)
        # Join the spin thread before tearing the context down: without
        # this, destroying the node/context can race the still-running
        # executor in the C++ layer ("terminate called without an
        # active exception" abort after the verdict was already out).
        self._obs_thread.join(timeout=3.0)
        self._recorder.destroy_node()
        rclpy.shutdown(context=self._obs_context)


def main(args=None):
    rclpy.init(args=args)
    oracle = ScenarioOracle()
    try:
        while rclpy.ok() and not oracle.done.is_set():
            rclpy.spin_once(oracle, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        code = oracle.exit_code
        oracle.shutdown_observation()
        oracle.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
