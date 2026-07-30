"""Topic arrival recorder running in the observed (robot) ROS domain.

Read-only by design (docs section 6.4): subscribes to the topics the
scenario's expectations reference, records arrival times only (raw
subscriptions - payloads are never deserialized, keeping the observer
cheap even for point clouds), and samples the node graph. Publishes
nothing into the observed domain. Topics referenced by content
expectations (invalid_message_not_published) are the exception: those
are deserialized and the requested fields sampled.
"""

import threading
import time

from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from rosidl_runtime_py.utilities import get_message

from .expectations import extract_field


class TopicRecorder(Node):
    """Records arrival timestamps for a set of topics.

    Topics in `age_topics` are additionally subscribed with
    deserialization so header.stamp can be compared against the arrival
    time (maximum_message_age). `content_topics` maps topic ->
    iterable of dotted field paths whose values are sampled per message
    (invalid_message_not_published). With `watch_diagnostics` the
    recorder also collects /diagnostics status levels.
    """

    def __init__(self, topics, *, context, age_topics=(),
                 content_topics=None, watch_diagnostics=False):
        super().__init__('hils_oracle_recorder', context=context)
        self._lock = threading.Lock()
        self._age_topics = set(age_topics)
        self._content_fields = {
            t: tuple(fields) for t, fields in (content_topics or {}).items()}
        all_topics = (set(topics) | self._age_topics
                      | set(self._content_fields))
        self._pending = set(all_topics)
        self._arrivals = {t: [] for t in all_topics}
        self._ages = {t: [] for t in self._age_topics}
        self._contents = {
            (t, f): [] for t, fields in self._content_fields.items()
            for f in fields}
        self._diagnostics = []  # (monotonic, status name, level)
        self._node_names = []
        self._qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        if watch_diagnostics:
            from diagnostic_msgs.msg import DiagnosticArray
            self.create_subscription(
                DiagnosticArray, '/diagnostics', self._on_diagnostics, 10)
        # Retry until each topic's type is discoverable (the driver may
        # not have started publishing yet).
        self.create_timer(0.5, self._resolve_pending)
        self.create_timer(1.0, self._sample_graph)

    def _resolve_pending(self):
        with self._lock:
            pending = list(self._pending)
        if not pending:
            return
        available = dict(self.get_topic_names_and_types())
        for topic in pending:
            types = available.get(topic)
            if not types:
                continue
            try:
                msg_type = get_message(types[0])
            except (AttributeError, ModuleNotFoundError, ValueError) as e:
                self.get_logger().warning(
                    f'cannot import type {types[0]} for {topic}: {e}')
                continue
            if topic in self._age_topics or topic in self._content_fields:
                # Deserialized: header.stamp and/or field values needed.
                self.create_subscription(
                    msg_type, topic,
                    lambda msg, t=topic: self._on_stamped_message(t, msg),
                    self._qos)
                mode = 'deserialized'
            else:
                self.create_subscription(
                    msg_type, topic,
                    lambda _msg, t=topic: self._on_message(t),
                    self._qos, raw=True)
                mode = 'raw'
            with self._lock:
                self._pending.discard(topic)
            self.get_logger().info(
                f'observing {topic} [{types[0]}] ({mode}, best-effort)')

    def _on_message(self, topic):
        now = time.monotonic()
        with self._lock:
            self._arrivals[topic].append(now)

    def _on_stamped_message(self, topic, msg):
        now = time.monotonic()
        age = None
        if topic in self._age_topics:
            header = getattr(msg, 'header', None)
            if header is not None:
                stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
                age = time.time() - stamp
        fields = self._content_fields.get(topic, ())
        values = [(f, extract_field(msg, f)) for f in fields]
        with self._lock:
            self._arrivals[topic].append(now)
            if age is not None:
                self._ages[topic].append((now, age))
            for f, value in values:
                self._contents[(topic, f)].append((now, value))

    def _on_diagnostics(self, msg):
        now = time.monotonic()
        with self._lock:
            for status in msg.status:
                level = status.level
                if isinstance(level, (bytes, bytearray)):
                    level = level[0] if level else 0
                self._diagnostics.append(
                    (now, f'{status.name}|{status.hardware_id}',
                     int(level)))

    def _sample_graph(self):
        names = [f'{ns.rstrip("/")}/{name}'
                 for name, ns in self.get_node_names_and_namespaces()]
        with self._lock:
            self._node_names = names

    # -- accessors (thread-safe snapshots, monotonic timestamps) --

    def arrivals(self) -> dict:
        with self._lock:
            return {t: list(v) for t, v in self._arrivals.items()}

    def ages(self) -> dict:
        with self._lock:
            return {t: list(v) for t, v in self._ages.items()}

    def contents(self) -> dict:
        """{(topic, field): [(monotonic_time, value), ...]}"""
        with self._lock:
            return {k: list(v) for k, v in self._contents.items()}

    def diagnostics(self) -> list:
        with self._lock:
            return list(self._diagnostics)

    def node_names(self) -> list:
        with self._lock:
            return list(self._node_names)

    def unresolved_topics(self) -> list:
        with self._lock:
            return sorted(self._pending)
