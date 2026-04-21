"""
PeripheralRunner — daemon-thread host for one peripheral instance.

A peripheral is a long-lived adapter between the cognitive layer and an
external real-time system (e.g. a robot body). Unlike sensors (which
poll on a schedule and the runner drives the cadence), peripherals own
their own internal loops at the device's native rate. The runner's job
is lifecycle (start/stop, daemon thread, module load) and the host-side
API the peripheral uses to register tools and emit sense events.

Phase 2.2 scope: lifecycle + module loading + helper API surface only.
Zenoh peer setup, tool registration into the executor, and sense_data
publication are NOT wired here — runner-facing callbacks are accepted
in __init__ but default to logging-only stubs so a peripheral can be
exercised in isolation. Phases 2.3+ supply the live callbacks.

Module contract: a peripheral's `peripheral.py` may export a
`Peripheral` class. If present, the runner instantiates it as
`Peripheral(runner=<self>, config=<dict>)` and calls
`instance.run(stop_event)` on the daemon thread. The peripheral is
expected to block in its own loop until `stop_event` is set, then
return cleanly. An optional `instance.stop()` is called before joining
the thread to give the peripheral a chance to flush state.

Modules without a `Peripheral` class load inertly — the runner's
daemon thread waits on stop_event and exits. Useful for pure-stub
peripherals (e.g. _test_loadable from Phase 2.1).
"""
from __future__ import annotations

import importlib.util
import logging
import threading
from typing import Any, Callable, Dict, Optional

logger = logging.getLogger(__name__)


class PeripheralRunner:
    """One daemon thread per loaded peripheral. See module docstring."""

    def __init__(
        self,
        metadata: Dict[str, Any],
        character_name: str,
        *,
        register_tool_fn: Optional[Callable[[str, Callable], None]] = None,
        publish_sense_fn: Optional[Callable[[str, str, Dict[str, Any]], None]] = None,
    ):
        """
        Args:
            metadata: One entry from peripheral_loader.load_peripherals().
                Required keys: name, python_file, config.
            character_name: Owning character (for log scoping; not yet wired
                into Zenoh topics — Phase 2.5).
            register_tool_fn: Optional callback the runner uses to register
                a peripheral-exposed tool with the executor's available_tools
                registry. None → log-only stub mode.
            publish_sense_fn: Optional callback for emitting sense events
                to cognition. Signature: (disposition, kind, payload). None
                → log-only stub mode.
        """
        self.metadata = metadata
        self.name = metadata['name']
        self.character_name = character_name

        self._register_tool_fn = register_tool_fn
        self._publish_sense_fn = publish_sense_fn

        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._peripheral_instance: Any = None
        self._started = False

    # ── Lifecycle ──────────────────────────────────────────────────────

    def start(self) -> None:
        """Spawn the daemon thread. Idempotent."""
        if self._started:
            return
        self._thread = threading.Thread(
            target=self._run,
            name=f"peripheral:{self.name}",
            daemon=True,
        )
        self._started = True
        self._thread.start()

    def stop(self, timeout: float = 5.0) -> None:
        """Signal stop, give the peripheral a chance to clean up, join."""
        self._stop_event.set()
        inst = self._peripheral_instance
        if inst is not None and hasattr(inst, 'stop'):
            try:
                inst.stop()
            except Exception:
                logger.exception(f"peripheral {self.name}: stop() raised")
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=timeout)
            if self._thread.is_alive():
                logger.warning(
                    f"peripheral {self.name}: thread did not stop "
                    f"within {timeout}s"
                )

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ── Daemon-thread entry ────────────────────────────────────────────

    def _run(self) -> None:
        try:
            module = self._load_peripheral_module()
        except Exception:
            logger.exception(
                f"peripheral {self.name}: module load failed; thread exiting"
            )
            return

        peripheral_cls = getattr(module, 'Peripheral', None)
        if peripheral_cls is None:
            logger.info(
                f"peripheral {self.name}: no `Peripheral` class in module — "
                f"loading inert (stub mode); thread will idle until stop"
            )
            self._stop_event.wait()
            return

        try:
            self._peripheral_instance = peripheral_cls(
                runner=self,
                config=dict(self.metadata.get('config', {})),
            )
        except Exception:
            logger.exception(
                f"peripheral {self.name}: Peripheral() instantiation raised; "
                f"thread exiting"
            )
            return

        run_fn = getattr(self._peripheral_instance, 'run', None)
        if run_fn is None:
            logger.warning(
                f"peripheral {self.name}: instance has no run(stop_event) "
                f"method — thread will idle until stop"
            )
            self._stop_event.wait()
            return

        try:
            run_fn(self._stop_event)
        except Exception:
            logger.exception(
                f"peripheral {self.name}: run(stop_event) raised; thread exiting"
            )

    def _load_peripheral_module(self):
        path = self.metadata['python_file']
        spec = importlib.util.spec_from_file_location(
            f"peripherals.{self.name}", path,
        )
        if spec is None or spec.loader is None:
            raise ImportError(f"could not build import spec for {path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    # ── Peripheral-facing helpers ──────────────────────────────────────

    def register_tool(self, name: str, fn: Callable) -> None:
        """A peripheral calls this from __init__ or run() to expose a
        callable to the planner under the given name (e.g. 'body.set_velocity').

        Phase 2.2: if no register_tool_fn was wired at construction, this
        is a log-only stub. Phase 2.4 wires this to the executor's
        available_tools registry.
        """
        if self._register_tool_fn is None:
            logger.info(
                f"peripheral {self.name}: register_tool('{name}') "
                f"called — no executor wired (stub mode)"
            )
            return
        try:
            self._register_tool_fn(name, fn)
        except Exception:
            logger.exception(
                f"peripheral {self.name}: register_tool('{name}') raised"
            )

    def publish_sense_event(
        self, disposition: str, kind: str, payload: Dict[str, Any],
    ) -> None:
        """A peripheral calls this to emit an alert/inform to cognition.
        `disposition` ∈ {'alert', 'inform', 'trigger', 'trigger-task'} —
        same vocabulary as the existing sense_data channel.

        Phase 2.2: if no publish_sense_fn was wired, log-only stub.
        Phase 2.5 wires this to cognitive/{character}/sense_data.
        """
        if self._publish_sense_fn is None:
            logger.info(
                f"peripheral {self.name}: publish_sense_event "
                f"({disposition}, {kind}) — no sense channel wired "
                f"(stub mode); payload keys={sorted(payload.keys())}"
            )
            return
        try:
            self._publish_sense_fn(disposition, kind, payload)
        except Exception:
            logger.exception(
                f"peripheral {self.name}: publish_sense_event "
                f"({disposition}, {kind}) raised"
            )
