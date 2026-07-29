"""Deferred packet transmission (docs section 17.7).

DelayedSender executes send callables after a requested delay using a
priority queue and a single dedicated thread. Ties on due time preserve
submission order.

Fail-safe (docs section 17.5): stop() discards pending packets, so a
stopped fault injection process never keeps transmitting stale data.
"""

import heapq
import threading
import time
from typing import Callable, Optional


class DelayedSender:
    """Priority-queue based deferred executor for packet sends."""

    def __init__(self, on_error: Optional[Callable[[Exception], None]] = None,
                 thread_name: str = 'hils_delayed_sender'):
        self._on_error = on_error
        self._thread_name = thread_name
        self._heap = []  # (due_monotonic, seq, send_fn)
        self._seq = 0
        self._cond = threading.Condition()
        self._thread: Optional[threading.Thread] = None
        self._stopped = False

    @property
    def pending_count(self) -> int:
        with self._cond:
            return len(self._heap)

    def submit(self, delay_s: float, send_fn: Callable[[], None]) -> bool:
        """Schedule send_fn to run after delay_s seconds."""
        due = time.monotonic() + max(0.0, delay_s)
        with self._cond:
            if self._stopped:
                return False
            heapq.heappush(self._heap, (due, self._seq, send_fn))
            self._seq += 1
            if self._thread is None:
                self._thread = threading.Thread(
                    target=self._run, name=self._thread_name, daemon=True)
                self._thread.start()
            self._cond.notify()
        return True

    def stop(self) -> int:
        """Stop the sender and discard pending packets.

        Returns the number of discarded packets.
        """
        with self._cond:
            discarded = len(self._heap)
            self._heap.clear()
            self._stopped = True
            self._cond.notify_all()
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        return discarded

    def _run(self):
        while True:
            with self._cond:
                while not self._stopped:
                    if not self._heap:
                        self._cond.wait()
                        continue
                    now = time.monotonic()
                    due = self._heap[0][0]
                    if due <= now:
                        break
                    self._cond.wait(timeout=due - now)
                if self._stopped:
                    return
                _, _, send_fn = heapq.heappop(self._heap)
            try:
                send_fn()
            except Exception as e:  # noqa: BLE001 - report, keep thread alive
                if self._on_error is not None:
                    self._on_error(e)
