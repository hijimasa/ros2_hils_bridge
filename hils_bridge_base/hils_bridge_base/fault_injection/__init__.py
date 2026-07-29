"""
hils_bridge_base.fault_injection - Common fault injection layer.

rclpy-independent core:
  fault_base        - Fault ABC, ScheduledPacket, parameter validation
  pipeline          - FaultPipeline applied to every outgoing packet
  sender            - DelayedSender (deferred transmission thread)
  delay_fault       - fixed delay + uniform/normal jitter
  drop_fault        - probabilistic / every-N packet loss
  corruption_fault  - byte-level bit flip / zero / truncate
  duplicate_fault   - packet duplication

ROS 2 integration (imported separately to keep the core testable
without ROS):
  fault_controller  - ROS 2 service API (/inject_fault etc., docs section 11)
"""

from typing import Optional

from .fault_base import Fault, FaultSpecError, ScheduledPacket
from .pipeline import FaultPipeline
from .sender import DelayedSender
from .delay_fault import DelayFault
from .drop_fault import DropFault
from .corruption_fault import CorruptionFault
from .duplicate_fault import DuplicateFault

FAULT_CLASSES = {
    cls.fault_type: cls
    for cls in (DelayFault, DropFault, CorruptionFault, DuplicateFault)
}


def create_fault(fault_type: str, fault_id: str, *,
                 target: Optional[str] = None, seed: int = 0,
                 parameters: Optional[dict] = None) -> Fault:
    """Instantiate a fault from a scenario/service specification.

    Raises FaultSpecError for unknown types or invalid parameters, so
    invalid scenarios are rejected before the test starts (docs section 18).
    """
    cls = FAULT_CLASSES.get(fault_type)
    if cls is None:
        raise FaultSpecError(
            f'unknown fault_type {fault_type!r}, '
            f'available: {sorted(FAULT_CLASSES)}')
    if parameters is not None and not isinstance(parameters, dict):
        raise FaultSpecError('parameters must be a mapping')
    return cls(fault_id, target=target, seed=seed, parameters=parameters)


__all__ = [
    'Fault', 'FaultSpecError', 'ScheduledPacket',
    'FaultPipeline', 'DelayedSender',
    'DelayFault', 'DropFault', 'CorruptionFault', 'DuplicateFault',
    'FAULT_CLASSES', 'create_fault',
]
