"""Scenario event scheduling core (docs section 6.3).

Independent of rclpy: the runner drives EventSchedule with elapsed
scenario time, and the schedule reports which events are due. Planned
and actual firing times are recorded for every event so reports can
show injection timing accuracy (docs sections 17.2, 17.6).
"""

from dataclasses import dataclass, field
from typing import List, Optional

from .scenario_loader import ScenarioEvent

PENDING = 'pending'
DISPATCHED = 'dispatched'
SUCCEEDED = 'succeeded'
FAILED = 'failed'


@dataclass
class TrackedEvent:
    event: ScenarioEvent
    status: str = PENDING
    actual_sec: Optional[float] = None   # elapsed time when dispatched
    result: str = ''                     # service response / error message
    fault_id: str = ''                   # id returned by inject_fault

    def describe(self) -> dict:
        info = {
            'index': self.event.index,
            'action': self.event.action,
            'planned_sec': self.event.at_sec,
            'actual_sec': self.actual_sec,
            'status': self.status,
        }
        if self.event.action == 'inject_fault':
            info['fault_type'] = self.event.fault.get('fault_type')
            if self.fault_id:
                info['fault_id'] = self.fault_id
        elif self.event.action == 'clear_fault':
            info['fault_id'] = self.event.fault_id
        if self.result:
            info['result'] = self.result
        return info


@dataclass
class EventSchedule:
    tracked: List[TrackedEvent] = field(default_factory=list)

    @classmethod
    def from_events(cls, events: List[ScenarioEvent]) -> 'EventSchedule':
        return cls([TrackedEvent(e) for e in events])

    def due(self, elapsed_sec: float) -> List[TrackedEvent]:
        """Mark and return pending events whose time has come."""
        fired = []
        for t in self.tracked:
            if t.status == PENDING and t.event.at_sec <= elapsed_sec:
                t.status = DISPATCHED
                t.actual_sec = elapsed_sec
                fired.append(t)
        return fired

    @property
    def pending_count(self) -> int:
        return sum(1 for t in self.tracked if t.status == PENDING)

    @property
    def in_flight_count(self) -> int:
        return sum(1 for t in self.tracked if t.status == DISPATCHED)

    @property
    def all_done(self) -> bool:
        return all(t.status in (SUCCEEDED, FAILED) for t in self.tracked)

    def snapshot(self) -> List[dict]:
        return [t.describe() for t in self.tracked]
