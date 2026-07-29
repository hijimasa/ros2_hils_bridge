"""Packet reorder fault (docs section 7.4).

Buffers packets per channel and releases them in a permuted order once
group_size packets have accumulated. Packets still buffered when the
fault is cleared are discarded (consistent with the fail-safe policy:
never flush stale data late).
"""

from typing import List, Optional

from .fault_base import (
    Fault, ScheduledPacket, require_int, require_choice, reject_unknown_keys,
)


class ReorderFault(Fault):
    """Release packets in shuffled or reversed groups.

    Parameters:
        group_size: packets buffered before release (default 4).
        mode: 'shuffle' (seeded random permutation) or 'reverse'.
    """

    fault_type = 'reorder'

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0, parameters: Optional[dict] = None):
        super().__init__(fault_id, target=target, seed=seed)
        params = parameters or {}
        reject_unknown_keys(params, ('group_size', 'mode'))
        self.group_size = require_int(params, 'group_size', 4, 2, 64)
        self.mode = require_choice(params, 'mode', 'shuffle',
                                   ('shuffle', 'reverse'))
        self._buffers = {}  # channel -> list of ScheduledPacket

    def parameters(self) -> dict:
        return {'group_size': self.group_size, 'mode': self.mode}

    def process(self, packets: List[ScheduledPacket],
                channel: str) -> List[ScheduledPacket]:
        self.processed_count += 1
        buf = self._buffers.setdefault(channel, [])
        buf.extend(packets)
        if len(buf) < self.group_size:
            return []
        group = buf[:]
        buf.clear()
        if self.mode == 'reverse':
            group.reverse()
        else:
            self.rng.shuffle(group)
        self.applied_count += 1
        return group
