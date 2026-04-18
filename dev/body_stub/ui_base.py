"""Abstract UI contract for the body stub.

A UI receives a StubController at construction, runs a mainloop in
run() (returns process exit code), and polls controller.state via
its own redraw timer. The controller is UI-agnostic, so swapping
Qt for e.g. a web UI only requires a new StubUI implementation.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

from .config import StubConfig
from .controller import StubController


class StubUI(ABC):
    def __init__(self, controller: StubController, config: StubConfig):
        self.controller = controller
        self.config = config

    @abstractmethod
    def run(self) -> int:
        """Block until the user closes the UI; return an exit code."""
        raise NotImplementedError
