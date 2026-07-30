"""Fault injection pipeline (docs sections 4.3, 6.1).

The pipeline holds an ordered set of active faults and applies them to
every outgoing packet. With no active faults, apply() is a passthrough
so normal operation is unchanged (docs section 17.4).
"""

import threading
from collections import OrderedDict
from typing import List

from .fault_base import Fault, ScheduledPacket


class FaultPipeline:
    """Ordered, thread-safe collection of active faults."""

    def __init__(self):
        self._lock = threading.Lock()
        self._faults: 'OrderedDict[str, Fault]' = OrderedDict()
        self._input_count = 0
        self._output_count = 0
        self._dropped_count = 0
        self._firmware_transport = None

    def set_firmware_transport(self, transport) -> None:
        """Install the firmware command sender (docs section 9.3).

        transport(payload_bytes) must deliver a 0x50-series command to
        the device firmware, bypassing this pipeline so active faults
        cannot swallow their own set/clear commands. Nodes without
        device firmware leave this unset; firmware-cooperative faults
        then fail at injection time with a clear error.
        """
        self._firmware_transport = transport

    @property
    def has_faults(self) -> bool:
        return bool(self._faults)

    def add_fault(self, fault: Fault) -> None:
        with self._lock:
            if fault.fault_id in self._faults:
                raise ValueError(f'fault_id already active: {fault.fault_id}')
        # on_added may raise (e.g. no firmware transport) - call it
        # before the fault becomes active so a failed arm leaves the
        # pipeline unchanged.
        fault.on_added(self._firmware_transport)
        with self._lock:
            if fault.fault_id in self._faults:
                raise ValueError(f'fault_id already active: {fault.fault_id}')
            self._faults[fault.fault_id] = fault

    def remove_fault(self, fault_id: str) -> bool:
        with self._lock:
            fault = self._faults.pop(fault_id, None)
        if fault is not None:
            fault.on_removed(self._firmware_transport)
        return fault is not None

    def clear(self) -> int:
        with self._lock:
            faults = list(self._faults.values())
            self._faults.clear()
        for fault in faults:
            fault.on_removed(self._firmware_transport)
        return len(faults)

    def faults_for(self, channel: str):
        """Active faults applying to channel (snapshot list).

        Used by emulators that need out-of-band fault attributes, e.g.
        the HTTP status override (http_status_fault).
        """
        with self._lock:
            return [f for f in self._faults.values()
                    if f.applies_to(channel)]

    def apply(self, data: bytes, channel: str = 'data') -> List[ScheduledPacket]:
        """Run one outgoing packet through all active faults.

        Returns the packets to actually transmit, each with a relative
        send delay in seconds. An empty list means the packet is dropped.
        """
        with self._lock:
            if not self._faults:
                return [ScheduledPacket(0.0, data)]
            faults = list(self._faults.values())
            self._input_count += 1

        packets = [ScheduledPacket(0.0, data)]
        for fault in faults:
            if not fault.applies_to(channel):
                continue
            packets = fault.process(packets, channel)
            if not packets:
                break

        with self._lock:
            self._output_count += len(packets)
            if not packets:
                self._dropped_count += 1
        return packets

    def snapshot(self) -> dict:
        """Return active faults and counters for state queries and logs."""
        with self._lock:
            return {
                'active_faults': [f.describe() for f in self._faults.values()],
                'input_packets': self._input_count,
                'output_packets': self._output_count,
                'dropped_packets': self._dropped_count,
            }
