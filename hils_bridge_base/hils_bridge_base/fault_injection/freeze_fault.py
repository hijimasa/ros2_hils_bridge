"""Freeze fault (docs section 7.7: fixed/stale value).

Models a sensor that keeps transmitting but whose data is stuck: the
first packet seen after activation is captured per channel, and every
subsequent packet on that channel is replaced with the captured copy.
Timing (packet rate) is unaffected, so drivers that only watch topic
rate or link liveness will not notice - exactly the failure mode this
fault exists to test.
"""

from typing import List, Optional

from .fault_base import Fault, ScheduledPacket, reject_unknown_keys


class FreezeFault(Fault):
    """Repeat the first captured packet on every subsequent send.

    Parameters: none.
    """

    fault_type = 'freeze'

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0, parameters: Optional[dict] = None):
        super().__init__(fault_id, target=target, seed=seed)
        reject_unknown_keys(parameters or {}, ())
        self._frozen = {}  # channel -> bytes

    def process(self, packets: List[ScheduledPacket],
                channel: str) -> List[ScheduledPacket]:
        self.processed_count += 1
        if not packets:
            return packets
        if channel not in self._frozen:
            self._frozen[channel] = packets[0].data
            return packets
        self.applied_count += 1
        frozen = self._frozen[channel]
        return [ScheduledPacket(pkt.delay_s, frozen) for pkt in packets]
