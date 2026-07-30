"""
Core abstractions for the HILS fault injection pipeline.

This module is independent of rclpy so the fault logic can be unit-tested
without a ROS 2 environment (see docs/fault_injection_implementation_policy.md
section 4.3: normal data generation and fault injection are separate layers).

A Fault transforms the list of packets scheduled for one send call:
  - dropping returns an empty list
  - duplicating returns additional entries
  - delaying increases ScheduledPacket.delay_s
  - corrupting mutates ScheduledPacket.data

Reproducibility (section 4.4): every fault owns a private random.Random
seeded from (seed, fault_id), so the generated fault sequence depends only
on the scenario configuration, never on other faults or wall-clock time.
"""

import random
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class ScheduledPacket:
    """A packet queued for transmission after delay_s seconds."""
    delay_s: float
    data: bytes


class FaultSpecError(ValueError):
    """Raised when a fault specification is invalid (rejected before start)."""


class Fault(ABC):
    """Base class for a single fault transformation."""

    fault_type = 'base'

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0):
        self.fault_id = fault_id
        self.target = target  # channel name; None applies to all channels
        self.seed = seed
        self.rng = random.Random(f'{seed}:{fault_id}')
        self.processed_count = 0
        self.applied_count = 0

    def applies_to(self, channel: str) -> bool:
        return self.target is None or self.target == channel

    @abstractmethod
    def process(self, packets: List[ScheduledPacket],
                channel: str) -> List[ScheduledPacket]:
        """Transform the scheduled packets for one send call.

        channel is the logical stream the packets belong to; stateful
        faults (freeze, reorder) keep independent state per channel.
        """

    def parameters(self) -> dict:
        return {}

    def on_added(self, firmware_transport) -> None:
        """Hook called by FaultPipeline.add_fault.

        firmware_transport is the pipeline's firmware command sender
        (None on nodes without device firmware). Host-side faults
        ignore it; firmware-cooperative faults forward a set command.
        """

    def on_removed(self, firmware_transport) -> None:
        """Hook called when the fault leaves the pipeline (clear/expiry)."""

    def describe(self) -> dict:
        return {
            'fault_id': self.fault_id,
            'fault_type': self.fault_type,
            'target': self.target,
            'seed': self.seed,
            'processed_count': self.processed_count,
            'applied_count': self.applied_count,
            'parameters': self.parameters(),
        }


def require_number(params: dict, key: str, default, lo, hi) -> float:
    """Fetch a numeric parameter with range check (rejects bool)."""
    value = params.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FaultSpecError(f'{key} must be a number, got {value!r}')
    if not (lo <= value <= hi):
        raise FaultSpecError(f'{key}={value} out of range [{lo}, {hi}]')
    return float(value)


def require_int(params: dict, key: str, default, lo, hi) -> int:
    value = params.get(key, default)
    if isinstance(value, bool) or not isinstance(value, int):
        raise FaultSpecError(f'{key} must be an integer, got {value!r}')
    if not (lo <= value <= hi):
        raise FaultSpecError(f'{key}={value} out of range [{lo}, {hi}]')
    return value


def require_choice(params: dict, key: str, default: str, choices) -> str:
    value = params.get(key, default)
    if value not in choices:
        raise FaultSpecError(f'{key}={value!r} must be one of {sorted(choices)}')
    return value


def reject_unknown_keys(params: dict, known) -> None:
    unknown = set(params) - set(known)
    if unknown:
        raise FaultSpecError(f'unknown parameter(s): {sorted(unknown)}')
