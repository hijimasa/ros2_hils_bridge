"""Protocol-field-aware faults for serial sensors (docs sections 7.5,
9.2, 18: field-level corruption lives above the byte-level common
faults).

Unlike CorruptionFault (blind byte flips), these target exactly the
checksum field so the rest of the frame stays valid - the way a real
line error most often shows up to a driver: a well-formed frame that
fails verification. A write may carry several frames; the probability
applies per frame.

Both protocols here are public (docs 4.1): NMEA 0183 is an open
standard; the WT901 frame layout is in Witmotion's published manual.
"""

import re
from typing import List, Optional

from .fault_base import (
    Fault, ScheduledPacket, require_number, reject_unknown_keys,
)

_NMEA_CHECKSUM_RE = re.compile(rb'\*([0-9A-Fa-f]{2})')

_WT901_HEADER = 0x55
_WT901_FRAME_LEN = 11


class NmeaChecksumFault(Fault):
    """Invalidate the checksum of NMEA 0183 sentences.

    Each `*HH` checksum in the payload is replaced with (HH+1) mod 256,
    leaving the sentence otherwise intact. A compliant driver must
    discard the sentence (docs 12.1: 異常データを正常値として公開しない).

    Parameters:
        probability: per-sentence corruption probability (default 1.0).
    """

    fault_type = 'nmea_checksum'

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0, parameters: Optional[dict] = None):
        super().__init__(fault_id, target=target, seed=seed)
        params = parameters or {}
        reject_unknown_keys(params, ('probability',))
        self.probability = require_number(params, 'probability', 1.0,
                                          0.0, 1.0)

    def parameters(self) -> dict:
        return {'probability': self.probability}

    def _corrupt_sentence_checksums(self, data: bytes) -> bytes:
        def _sub(match):
            if self.rng.random() >= self.probability:
                return match.group(0)
            bad = (int(match.group(1), 16) + 1) & 0xFF
            return b'*%02X' % bad
        return _NMEA_CHECKSUM_RE.sub(_sub, data)

    def process(self, packets: List[ScheduledPacket],
                channel: str) -> List[ScheduledPacket]:
        self.processed_count += 1
        out = []
        changed = False
        for pkt in packets:
            data = self._corrupt_sentence_checksums(pkt.data)
            changed = changed or data != pkt.data
            out.append(ScheduledPacket(pkt.delay_s, data))
        if changed:
            self.applied_count += 1
        return out


class Wt901ChecksumFault(Fault):
    """Invalidate the checksum byte of WT901 binary frames.

    The payload is scanned for 11-byte frames starting with 0x55; the
    last byte (sum checksum) of each frame is incremented so the frame
    fails verification while header and type stay plausible.

    Parameters:
        probability: per-frame corruption probability (default 1.0).
    """

    fault_type = 'wt901_checksum'

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0, parameters: Optional[dict] = None):
        super().__init__(fault_id, target=target, seed=seed)
        params = parameters or {}
        reject_unknown_keys(params, ('probability',))
        self.probability = require_number(params, 'probability', 1.0,
                                          0.0, 1.0)

    def parameters(self) -> dict:
        return {'probability': self.probability}

    def _corrupt_frames(self, data: bytes) -> bytes:
        buf = bytearray(data)
        i = 0
        while i + _WT901_FRAME_LEN <= len(buf):
            if buf[i] != _WT901_HEADER:
                i += 1
                continue
            if self.rng.random() < self.probability:
                buf[i + _WT901_FRAME_LEN - 1] = \
                    (buf[i + _WT901_FRAME_LEN - 1] + 1) & 0xFF
            i += _WT901_FRAME_LEN
        return bytes(buf)

    def process(self, packets: List[ScheduledPacket],
                channel: str) -> List[ScheduledPacket]:
        self.processed_count += 1
        out = []
        changed = False
        for pkt in packets:
            data = self._corrupt_frames(pkt.data)
            changed = changed or data != pkt.data
            out.append(ScheduledPacket(pkt.delay_s, data))
        if changed:
            self.applied_count += 1
        return out
