"""Device state machine core (docs sections 4.2, 6.2, 7.8, 8.1).

Independent of rclpy: time is injected by the caller (a monotonic
clock function), and timed transitions (e.g. REBOOTING -> target after
boot_duration) are reported to the integration layer, which owns the
timers. Listeners are notified on every transition so device-specific
code can react (e.g. Livox resets its work mode on reboot).
"""

import threading
import time as _time
from typing import Callable, List, Optional

from . import state as st


class DeviceStateMachine:
    """Thread-safe device state with per-state channel gating."""

    def __init__(self, initial_state: str = st.STREAMING, *,
                 channel_policy: Optional[dict] = None,
                 clock: Callable[[], float] = _time.monotonic):
        if not st.is_valid_state(initial_state):
            raise ValueError(f'invalid initial state: {initial_state!r}')
        self._policy = dict(st.DEFAULT_CHANNEL_POLICY)
        if channel_policy:
            self._policy.update(channel_policy)
        self._clock = clock
        self._lock = threading.Lock()
        self._state = initial_state
        self._entered_at = clock()
        self._suppressed_counts = {}  # channel -> count
        self._listeners: List[Callable[[str, str], None]] = []
        self._transition_count = 0

    # -- state --

    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    def time_in_state(self) -> float:
        with self._lock:
            return self._clock() - self._entered_at

    def add_listener(self, callback: Callable[[str, str], None]) -> None:
        """Register callback(old_state, new_state), called on transition.

        Callbacks run in the thread that triggered the transition and
        outside the internal lock.
        """
        self._listeners.append(callback)

    def set_state(self, new_state: str) -> str:
        """Transition to new_state, returning the previous state."""
        if not st.is_valid_state(new_state):
            raise ValueError(f'invalid state: {new_state!r}')
        with self._lock:
            old_state = self._state
            if new_state == old_state:
                return old_state
            self._state = new_state
            self._entered_at = self._clock()
            self._transition_count += 1
        for callback in list(self._listeners):
            callback(old_state, new_state)
        return old_state

    # -- channel gating --

    def is_open(self, channel: str) -> bool:
        """Whether the current state permits channel, without counting.

        Use this for gating *inbound* request processing (a powered-off
        device does not parse requests); use allows() on the send path
        so suppressed transmissions are counted.
        """
        with self._lock:
            return st.channel_open(self._policy, self._state, channel)

    def allows(self, channel: str) -> bool:
        """Whether the current state permits transmission on channel.

        A blocked channel is counted so suppressed traffic is
        observable (docs section 17.2).
        """
        with self._lock:
            if st.channel_open(self._policy, self._state, channel):
                return True
            self._suppressed_counts[channel] = \
                self._suppressed_counts.get(channel, 0) + 1
            return False

    # -- introspection --

    def snapshot(self) -> dict:
        with self._lock:
            return {
                'state': self._state,
                'time_in_state_sec': round(self._clock() - self._entered_at, 3),
                'transition_count': self._transition_count,
                'suppressed_packets': dict(self._suppressed_counts),
            }
