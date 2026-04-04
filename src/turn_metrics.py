"""Per-turn LLM and tool latency instrumentation.

Produces a single PERF log line per user turn summarizing where wall-clock
time went: LLM calls (grouped by phase), tool calls, and overhead.

Usage:
    metrics = TurnMetrics()              # at turn start
    with metrics.perf_phase("orient"):   # at phase boundaries
        ...
    metrics.record_llm("openai", 1.23, out_tokens=150)  # inside llm_generate
    metrics.record_tool("search-web", 2.1)               # at tool dispatch
    logger.info(metrics.summary())                        # at turn end
"""

import logging
import threading
import time
from collections import defaultdict
from contextlib import contextmanager

logger = logging.getLogger(__name__)


class TurnMetrics:
    """Accumulates timing data for a single user turn."""

    def __init__(self):
        self.start = time.monotonic()
        self.llm_calls = []     # [(phase, backend, elapsed_s, out_tokens), ...]
        self.tool_calls = []    # [(tool_name, elapsed_s), ...]
        self._phase_stack = ["unknown"]
        self._lock = threading.Lock()

    # ── Phase tracking ──────────────────────────────────────────────

    @contextmanager
    def perf_phase(self, name: str):
        """Context manager to label LLM calls within a phase."""
        self._phase_stack.append(name)
        try:
            yield
        finally:
            self._phase_stack.pop()

    @property
    def current_phase(self) -> str:
        return self._phase_stack[-1]

    # ── Recording ───────────────────────────────────────────────────

    def record_llm(self, backend: str, elapsed_s: float, out_tokens: int = 0):
        with self._lock:
            self.llm_calls.append((self.current_phase, backend, elapsed_s, out_tokens))

    def record_tool(self, tool_name: str, elapsed_s: float):
        with self._lock:
            self.tool_calls.append((tool_name, elapsed_s))

    # ── Summary ─────────────────────────────────────────────────────

    def summary(self) -> str:
        """One-line PERF summary for this turn."""
        total_s = time.monotonic() - self.start

        # LLM totals by phase
        phase_time = defaultdict(float)
        phase_count = defaultdict(int)
        phase_tokens = defaultdict(int)
        llm_total = 0.0
        total_out_tokens = 0
        for phase, _backend, elapsed, out_tok in self.llm_calls:
            phase_time[phase] += elapsed
            phase_count[phase] += 1
            phase_tokens[phase] += out_tok
            llm_total += elapsed
            total_out_tokens += out_tok

        llm_count = len(self.llm_calls)
        phase_parts = []
        for phase in phase_time:
            tok_str = f"/{phase_tokens[phase]}tok" if phase_tokens[phase] else ""
            phase_parts.append(f"{phase} {phase_time[phase]:.1f}s/{phase_count[phase]}{tok_str}")
        llm_detail = " ".join(phase_parts) if phase_parts else "none"

        tok_total_str = f"/{total_out_tokens}tok" if total_out_tokens else ""

        # Tool totals by name
        tool_time_by_name = defaultdict(float)
        tool_total = 0.0
        for name, elapsed in self.tool_calls:
            tool_time_by_name[name] += elapsed
            tool_total += elapsed

        tool_parts = []
        for name in tool_time_by_name:
            tool_parts.append(f"{name} {tool_time_by_name[name]:.1f}s")
        tool_detail = " ".join(tool_parts) if tool_parts else "none"

        overhead = max(0.0, total_s - llm_total - tool_total)

        return (
            f"PERF | turn {total_s:.1f}s"
            f" | llm {llm_total:.1f}s/{llm_count}calls{tok_total_str} [{llm_detail}]"
            f" | tools {tool_total:.1f}s [{tool_detail}]"
            f" | overhead {overhead:.1f}s"
        )
