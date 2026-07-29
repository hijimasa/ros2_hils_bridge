"""Data corruption fault (docs section 7.5).

Byte-level corruption is implemented here in the common layer; protocol
field-aware corruption belongs to the device-specific layer (docs
section 18).
"""

from typing import List, Optional

from .fault_base import (
    Fault, ScheduledPacket, require_number, require_int, require_choice,
    reject_unknown_keys,
)


class CorruptionFault(Fault):
    """Corrupt packet bytes.

    Parameters:
        probability: corruption probability per packet (default 1.0).
        mode: 'bit_flip' XOR random bytes with a nonzero mask,
              'zero' overwrite bytes with 0x00,
              'truncate' cut the packet at a random position.
        num_bytes: bytes affected by bit_flip/zero (default 1).
        offset: fixed start offset for bit_flip/zero; -1 = random (default).
    """

    fault_type = 'corrupt'

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0, parameters: Optional[dict] = None):
        super().__init__(fault_id, target=target, seed=seed)
        params = parameters or {}
        reject_unknown_keys(
            params, ('probability', 'mode', 'num_bytes', 'offset'))
        self.probability = require_number(params, 'probability', 1.0, 0.0, 1.0)
        self.mode = require_choice(
            params, 'mode', 'bit_flip', ('bit_flip', 'zero', 'truncate'))
        self.num_bytes = require_int(params, 'num_bytes', 1, 1, 65536)
        self.offset = require_int(params, 'offset', -1, -1, 65536)

    def parameters(self) -> dict:
        return {
            'probability': self.probability,
            'mode': self.mode,
            'num_bytes': self.num_bytes,
            'offset': self.offset,
        }

    def _corrupt(self, data: bytes) -> bytes:
        if len(data) == 0:
            return data
        if self.mode == 'truncate':
            # Keep at least 1 byte and cut at least 1 byte.
            if len(data) < 2:
                return data
            return data[:self.rng.randrange(1, len(data))]

        buf = bytearray(data)
        if self.offset >= 0:
            start = min(self.offset, len(buf) - 1)
            positions = range(start, min(start + self.num_bytes, len(buf)))
        else:
            count = min(self.num_bytes, len(buf))
            positions = self.rng.sample(range(len(buf)), count)
        for pos in positions:
            if self.mode == 'zero':
                buf[pos] = 0x00
            else:
                buf[pos] ^= self.rng.randrange(1, 256)
        return bytes(buf)

    def process(self, packets: List[ScheduledPacket]) -> List[ScheduledPacket]:
        self.processed_count += 1
        if self.rng.random() >= self.probability:
            return packets
        self.applied_count += 1
        return [ScheduledPacket(pkt.delay_s, self._corrupt(pkt.data))
                for pkt in packets]
