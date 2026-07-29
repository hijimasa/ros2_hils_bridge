"""Packet duplication fault (docs section 7.4)."""

from typing import List, Optional

from .fault_base import (
    Fault, ScheduledPacket, require_number, require_int, reject_unknown_keys,
)


class DuplicateFault(Fault):
    """Send extra copies of packets.

    Parameters:
        probability: duplication probability per packet (default 1.0).
        copies: number of extra copies (default 1).
    """

    fault_type = 'duplicate'

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0, parameters: Optional[dict] = None):
        super().__init__(fault_id, target=target, seed=seed)
        params = parameters or {}
        reject_unknown_keys(params, ('probability', 'copies'))
        self.probability = require_number(params, 'probability', 1.0, 0.0, 1.0)
        self.copies = require_int(params, 'copies', 1, 1, 16)

    def parameters(self) -> dict:
        return {'probability': self.probability, 'copies': self.copies}

    def process(self, packets: List[ScheduledPacket]) -> List[ScheduledPacket]:
        self.processed_count += 1
        if self.rng.random() >= self.probability:
            return packets
        self.applied_count += 1
        result = []
        for pkt in packets:
            result.append(pkt)
            for _ in range(self.copies):
                result.append(ScheduledPacket(pkt.delay_s, pkt.data))
        return result
