"""Phase 2.2 lifecycle stub. Counts ticks until stop_event is set."""
from __future__ import annotations

import threading


class Peripheral:
    def __init__(self, runner, config):
        self.runner = runner
        self.tick_period_s = float(config.get('tick_period_s', 0.05))
        self.ticks = 0
        self.stopped_cleanly = False
        self.stop_called = False

    def run(self, stop_event: threading.Event) -> None:
        while not stop_event.wait(timeout=self.tick_period_s):
            self.ticks += 1
        self.stopped_cleanly = True

    def stop(self) -> None:
        self.stop_called = True
