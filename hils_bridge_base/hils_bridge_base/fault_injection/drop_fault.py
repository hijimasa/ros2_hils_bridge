"""Packet drop fault (docs section 7.3)."""

from typing import List, Optional

from .fault_base import (
    Fault, ScheduledPacket, require_number, require_int, reject_unknown_keys,
)


class DropFault(Fault):
    """Drop packets probabilistically or every N-th packet.

    Parameters:
        probability: drop probability per packet (default 1.0 = mute).
        every_n: if > 0, deterministically drop every N-th packet
                 (counted per fault instance); overrides probability.
    """

    fault_type = 'drop'

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0, parameters: Optional[dict] = None):
        super().__init__(fault_id, target=target, seed=seed)
        params = parameters or {}
        reject_unknown_keys(params, ('probability', 'every_n'))
        self.probability = require_number(params, 'probability', 1.0, 0.0, 1.0)
        self.every_n = require_int(params, 'every_n', 0, 0, 1_000_000)

    def parameters(self) -> dict:
        return {'probability': self.probability, 'every_n': self.every_n}

    def process(self, packets: List[ScheduledPacket],
                channel: str) -> List[ScheduledPacket]:
        self.processed_count += 1
        if self.every_n > 0:
            drop = (self.processed_count % self.every_n) == 0
        else:
            drop = self.rng.random() < self.probability
        if drop:
            self.applied_count += 1
            return []
        return packets
