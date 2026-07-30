"""netem (tc qdisc) backend for statistical transport faults (docs 4.5).

Statistical delay/jitter/loss/duplication/reordering/corruption can be
delegated to the Linux kernel's netem qdisc instead of the Python
pipeline: the emulator applies `tc qdisc` to its own egress interface,
so time precision and throughput are the kernel's problem. Use this for
*statistical* faults only - netem is not seed-reproducible, knows no
protocol fields, and hits every packet on the interface (docs 4.5's
layer split; deterministic and field-aware faults stay in the
pipeline).

The fault is lifecycle-driven like the firmware faults: process() is a
passthrough, on_added() applies the qdisc, on_removed() deletes it.
The exact tc command is exposed via parameters() so injection logs and
reports record the applied configuration (docs 4.5).

Requires CAP_NET_ADMIN and iproute2 in the emulator's namespace
(docker-compose adds NET_ADMIN to the sim container).
"""

import subprocess
import threading
from typing import List, Optional

from .fault_base import (
    Fault, FaultSpecError, ScheduledPacket, require_number,
    reject_unknown_keys,
)

_PARAM_KEYS = ('interface', 'delay_ms', 'jitter_ms', 'loss_percent',
               'duplicate_percent', 'reorder_percent', 'corrupt_percent',
               'rate_kbit')


def _run_tc(args: List[str]) -> None:
    """Run a tc command; raise ValueError with stderr on failure."""
    try:
        proc = subprocess.run(['tc'] + args, capture_output=True,
                              text=True, timeout=10)
    except FileNotFoundError:
        raise ValueError('tc not found - install iproute2 in the '
                         'emulator environment')
    except subprocess.TimeoutExpired:
        raise ValueError('tc command timed out')
    if proc.returncode != 0:
        raise ValueError(
            f'tc {" ".join(args)} failed: '
            f'{proc.stderr.strip() or proc.stdout.strip()} '
            f'(CAP_NET_ADMIN missing?)')


class NetemFault(Fault):
    """Apply a netem qdisc to a network interface for the fault's
    lifetime.

    Parameters (at least one effect is required):
        interface: target interface (default "eth0").
        delay_ms / jitter_ms: constant delay and jitter.
        loss_percent, duplicate_percent, corrupt_percent: statistical
            packet effects, 0-100.
        reorder_percent: reordering probability 0-100 (requires
            delay_ms > 0; netem reorders by letting packets skip the
            delay queue).
        rate_kbit: egress rate limit in kbit/s.

    Only one netem fault may be active per interface; the qdisc root is
    replaced on inject and deleted on clear/expiry.
    """

    fault_type = 'netem'

    # interface -> fault_id, guarded for concurrent service calls
    _active = {}
    _active_lock = threading.Lock()

    # test seam: unit tests replace this with a recorder
    tc_runner = staticmethod(_run_tc)

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0, parameters: Optional[dict] = None):
        super().__init__(fault_id, target=target, seed=seed)
        params = parameters or {}
        reject_unknown_keys(params, _PARAM_KEYS)
        interface = params.get('interface', 'eth0')
        if not isinstance(interface, str) or not interface \
                or any(c in interface for c in ' ;|&'):
            raise FaultSpecError('interface must be a plain interface name')
        self.interface = interface
        self.delay_ms = require_number(params, 'delay_ms', 0.0, 0.0, 60000.0)
        self.jitter_ms = require_number(params, 'jitter_ms', 0.0, 0.0,
                                        60000.0)
        self.loss_percent = require_number(params, 'loss_percent', 0.0,
                                           0.0, 100.0)
        self.duplicate_percent = require_number(
            params, 'duplicate_percent', 0.0, 0.0, 100.0)
        self.reorder_percent = require_number(
            params, 'reorder_percent', 0.0, 0.0, 100.0)
        self.corrupt_percent = require_number(
            params, 'corrupt_percent', 0.0, 0.0, 100.0)
        self.rate_kbit = require_number(params, 'rate_kbit', 0.0, 0.0,
                                        10_000_000.0)
        if self.jitter_ms > 0.0 and self.delay_ms <= 0.0:
            raise FaultSpecError('jitter_ms requires delay_ms > 0')
        if self.reorder_percent > 0.0 and self.delay_ms <= 0.0:
            raise FaultSpecError('reorder_percent requires delay_ms > 0')
        if not self._netem_args():
            raise FaultSpecError(
                'at least one effect is required (delay_ms, '
                'loss_percent, duplicate_percent, corrupt_percent, '
                'rate_kbit)')

    def _netem_args(self) -> List[str]:
        args = []
        if self.delay_ms > 0.0:
            args += ['delay', f'{self.delay_ms}ms']
            if self.jitter_ms > 0.0:
                args += [f'{self.jitter_ms}ms']
        if self.loss_percent > 0.0:
            args += ['loss', f'{self.loss_percent}%']
        if self.duplicate_percent > 0.0:
            args += ['duplicate', f'{self.duplicate_percent}%']
        if self.reorder_percent > 0.0:
            args += ['reorder', f'{self.reorder_percent}%']
        if self.corrupt_percent > 0.0:
            args += ['corrupt', f'{self.corrupt_percent}%']
        if self.rate_kbit > 0.0:
            args += ['rate', f'{self.rate_kbit}kbit']
        return args

    def tc_apply_args(self) -> List[str]:
        return (['qdisc', 'replace', 'dev', self.interface, 'root',
                 'netem'] + self._netem_args())

    def tc_delete_args(self) -> List[str]:
        return ['qdisc', 'del', 'dev', self.interface, 'root']

    def parameters(self) -> dict:
        # The recorded config doubles as evidence of the applied qdisc
        # (docs 4.5: record the netem settings in test results).
        return {
            'interface': self.interface,
            'delay_ms': self.delay_ms,
            'jitter_ms': self.jitter_ms,
            'loss_percent': self.loss_percent,
            'duplicate_percent': self.duplicate_percent,
            'reorder_percent': self.reorder_percent,
            'corrupt_percent': self.corrupt_percent,
            'rate_kbit': self.rate_kbit,
            'tc_command': 'tc ' + ' '.join(self.tc_apply_args()),
        }

    def process(self, packets: List[ScheduledPacket],
                channel: str) -> List[ScheduledPacket]:
        self.processed_count += 1
        return packets

    def on_added(self, firmware_transport) -> None:
        del firmware_transport  # kernel-level fault; no device transport
        with self._active_lock:
            holder = self._active.get(self.interface)
            if holder is not None:
                raise ValueError(
                    f'netem already active on {self.interface} '
                    f'(fault_id={holder}); clear it first')
            self._active[self.interface] = self.fault_id
        try:
            type(self).tc_runner(self.tc_apply_args())
        except ValueError:
            with self._active_lock:
                self._active.pop(self.interface, None)
            raise
        self.applied_count += 1

    def on_removed(self, firmware_transport) -> None:
        del firmware_transport
        with self._active_lock:
            if self._active.get(self.interface) != self.fault_id:
                return
            self._active.pop(self.interface, None)
        try:
            type(self).tc_runner(self.tc_delete_args())
        except ValueError:
            # Best-effort: clear/expiry must not fail because the qdisc
            # is already gone (e.g. interface went down).
            pass
