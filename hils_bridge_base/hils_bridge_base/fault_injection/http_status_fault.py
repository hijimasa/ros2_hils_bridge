"""HTTP status code fault (docs section 9.1: invalid HTTP status).

HTTP responses are request/response bodies, not packets, so the status
code lives outside the byte pipeline. This fault passes the body
through unchanged; HTTP-capable emulators (e.g. Ouster) look it up via
FaultPipeline.faults_for() and call sample_status() when building a
response. Emulators without an HTTP server simply never sample it.
"""

from typing import List, Optional

from .fault_base import (
    Fault, ScheduledPacket, require_number, require_int, reject_unknown_keys,
)


class HttpStatusFault(Fault):
    """Override the HTTP status code of responses.

    Parameters:
        status: status code to return (default 500).
        probability: override probability per response (default 1.0).
    """

    fault_type = 'http_status'

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0, parameters: Optional[dict] = None):
        super().__init__(fault_id, target=target, seed=seed)
        params = parameters or {}
        reject_unknown_keys(params, ('status', 'probability'))
        self.status = require_int(params, 'status', 500, 100, 599)
        self.probability = require_number(params, 'probability', 1.0, 0.0, 1.0)

    def parameters(self) -> dict:
        return {'status': self.status, 'probability': self.probability}

    def process(self, packets: List[ScheduledPacket],
                channel: str) -> List[ScheduledPacket]:
        # Body bytes are untouched; the status override happens via
        # sample_status() on the HTTP response path.
        return packets

    def sample_status(self) -> Optional[int]:
        """Draw the status override for one response (seeded)."""
        self.processed_count += 1
        if self.rng.random() < self.probability:
            self.applied_count += 1
            return self.status
        return None
