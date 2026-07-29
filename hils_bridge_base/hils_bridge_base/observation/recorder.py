"""Topic arrival recorder running in the observed (robot) ROS domain.

Read-only by design (docs section 6.4): subscribes to the topics the
scenario's expectations reference, records arrival times only (raw
subscriptions - payloads are never deserialized, keeping the observer
cheap even for point clouds), and samples the node graph. Publishes
nothing into the observed domain.
"""

import threading
import time

from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from rosidl_runtime_py.utilities import get_message


class TopicRecorder(Node):
    """Records arrival timestamps for a set of topics."""

    def __init__(self, topics, *, context):
        super().__init__('hils_oracle_recorder', context=context)
        self._lock = threading.Lock()
        self._pending = set(topics)
        self._arrivals = {t: [] for t in topics}
        self._node_names = []
        self._qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
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
            self.create_subscription(
                msg_type, topic,
                lambda _msg, t=topic: self._on_message(t),
                self._qos, raw=True)
            with self._lock:
                self._pending.discard(topic)
            self.get_logger().info(
                f'observing {topic} [{types[0]}] (raw, best-effort)')

    def _on_message(self, topic):
        now = time.monotonic()
        with self._lock:
            self._arrivals[topic].append(now)

    def _sample_graph(self):
        names = [f'{ns.rstrip("/")}/{name}'
                 for name, ns in self.get_node_names_and_namespaces()]
        with self._lock:
            self._node_names = names

    # -- accessors (thread-safe snapshots, monotonic timestamps) --

    def arrivals(self) -> dict:
        with self._lock:
            return {t: list(v) for t, v in self._arrivals.items()}

    def node_names(self) -> list:
        with self._lock:
            return list(self._node_names)

    def unresolved_topics(self) -> list:
        with self._lock:
            return sorted(self._pending)
