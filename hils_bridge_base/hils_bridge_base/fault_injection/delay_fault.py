"""Delay and jitter fault (docs section 7.2).

The fault only computes the additional delay; actual deferred transmission
is performed by sender.DelayedSender (priority queue + dedicated thread,
docs section 17.7).
"""

from typing import List, Optional

from .fault_base import (
    Fault, ScheduledPacket, require_number, require_choice,
    reject_unknown_keys,
)

_MAX_DELAY_MS = 60_000.0


class DelayFault(Fault):
    """Add fixed delay plus optional jitter to every packet.

    Parameters:
        delay_ms: base delay in milliseconds (default 0).
        jitter_ms: jitter amplitude in milliseconds (default 0).
        distribution: 'uniform' -> delay + U(-jitter, +jitter)
                      'normal'  -> delay + N(0, jitter)
        The resulting delay is clamped to >= 0.
    """

    fault_type = 'delay'

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0, parameters: Optional[dict] = None):
        super().__init__(fault_id, target=target, seed=seed)
        params = parameters or {}
        reject_unknown_keys(params, ('delay_ms', 'jitter_ms', 'distribution'))
        self.delay_ms = require_number(params, 'delay_ms', 0.0, 0.0, _MAX_DELAY_MS)
        self.jitter_ms = require_number(params, 'jitter_ms', 0.0, 0.0, _MAX_DELAY_MS)
        self.distribution = require_choice(
            params, 'distribution', 'uniform', ('uniform', 'normal'))

    def parameters(self) -> dict:
        return {
            'delay_ms': self.delay_ms,
            'jitter_ms': self.jitter_ms,
            'distribution': self.distribution,
        }

    def _sample_delay_s(self) -> float:
        delay_ms = self.delay_ms
        if self.jitter_ms > 0.0:
            if self.distribution == 'normal':
                delay_ms += self.rng.gauss(0.0, self.jitter_ms)
            else:
                delay_ms += self.rng.uniform(-self.jitter_ms, self.jitter_ms)
        return max(0.0, delay_ms) / 1000.0

    def process(self, packets: List[ScheduledPacket],
                channel: str) -> List[ScheduledPacket]:
        self.processed_count += 1
        extra_s = self._sample_delay_s()
        if extra_s <= 0.0:
            return packets
        self.applied_count += 1
        return [ScheduledPacket(pkt.delay_s + extra_s, pkt.data)
                for pkt in packets]
