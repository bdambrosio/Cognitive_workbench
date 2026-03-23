"""
Autonomous Goal Scheduler

Timer-based coordinator that periodically checks for eligible goals and
auto-proceeds them by enqueuing synthetic "proceed" commands into the
existing text_input_queue.  One goal at a time — the executing guard
(_executing_goal_id) is set before proceed and cleared by step/terminal
notifications from the executive node.

Autonomy budget: autonomous goals (origin="autonomous") are subject to a
rolling time budget. When the budget is exhausted, autonomous goals are
deferred until it replenishes. User-initiated goals are always exempt.
"""

import threading
import logging
import time
from collections import deque

logger = logging.getLogger('goal_scheduler')

# Default: 15 minutes of autonomous work per 60-minute rolling window
DEFAULT_BUDGET_MINUTES = 15
DEFAULT_BUDGET_WINDOW_MINUTES = 60


class GoalScheduler:
    def __init__(self, character_name: str, interval: float = 15.0, enabled: bool = False,
                 budget_minutes: float = DEFAULT_BUDGET_MINUTES,
                 budget_window_minutes: float = DEFAULT_BUDGET_WINDOW_MINUTES):
        self.character_name = character_name
        self.interval = interval          # seconds between checks
        self.enabled = enabled
        self._executing_goal_id = None    # one-at-a-time guard
        self._stop_event = threading.Event()
        self._thread = None
        self._check_fn = None             # callback: () -> list[eligible goals]
        self._proceed_fn = None           # callback: (goal_id) -> None
        self._last_action = ""
        self._last_check_time = 0.0

        # Autonomy budget
        self.budget_seconds = budget_minutes * 60.0
        self.budget_window_seconds = budget_window_minutes * 60.0
        # Ring buffer of (start_time, duration) for completed autonomous executions
        self._autonomous_runs: deque = deque()
        self._autonomous_start_time: float = 0.0  # when current autonomous goal started
        self._executing_is_autonomous: bool = False

    def start(self, check_fn, proceed_fn):
        """Start the scheduler daemon thread."""
        self._check_fn = check_fn
        self._proceed_fn = proceed_fn
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name=f"goal-scheduler-{self.character_name}")
        self._thread.start()
        logger.info(f"GoalScheduler started for {self.character_name} "
                     f"(interval={self.interval}s, enabled={self.enabled}, "
                     f"budget={self.budget_seconds/60:.0f}m/{self.budget_window_seconds/60:.0f}m)")

    def stop(self):
        """Stop the scheduler thread."""
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5.0)
        self._thread = None
        logger.info(f"GoalScheduler stopped for {self.character_name}")

    def set_enabled(self, enabled: bool):
        self.enabled = enabled
        self._last_action = f"{'enabled' if enabled else 'disabled'} by user"
        logger.info(f"GoalScheduler {self._last_action} for {self.character_name}")

    def set_interval(self, seconds: float):
        seconds = max(60.0, min(86400.0, seconds))
        self.interval = seconds
        logger.info(f"GoalScheduler interval set to {seconds}s for {self.character_name}")

    def notify_step_completed(self, goal_id: str):
        """Called after a goal step finishes — clears the executing guard."""
        if self._executing_goal_id == goal_id:
            logger.info(f"GoalScheduler: step completed for {goal_id}, clearing guard")
            self._record_autonomous_completion()
            self._executing_goal_id = None
            self._executing_is_autonomous = False

    def notify_goal_terminal(self, goal_id: str):
        """Called when a goal reaches a terminal state (completed/blocked/abandoned)."""
        if self._executing_goal_id == goal_id:
            logger.info(f"GoalScheduler: terminal state for {goal_id}, clearing guard")
            self._record_autonomous_completion()
            self._executing_goal_id = None
            self._executing_is_autonomous = False

    # ── Autonomy budget ──────────────────────────────────────────────

    def _record_autonomous_completion(self):
        """Record execution time for the current autonomous goal."""
        if not self._executing_is_autonomous or self._autonomous_start_time == 0.0:
            return
        duration = time.monotonic() - self._autonomous_start_time
        self._autonomous_runs.append((time.monotonic(), duration))
        self._autonomous_start_time = 0.0
        logger.info(f"GoalScheduler: autonomous run completed, {duration:.1f}s used")

    def _prune_old_runs(self):
        """Remove entries outside the rolling window."""
        cutoff = time.monotonic() - self.budget_window_seconds
        while self._autonomous_runs and self._autonomous_runs[0][0] < cutoff:
            self._autonomous_runs.popleft()

    def budget_remaining(self) -> float:
        """Return remaining autonomous budget in seconds for the current window."""
        self._prune_old_runs()
        used = sum(duration for _, duration in self._autonomous_runs)
        # Include time for the currently running autonomous goal (if any)
        if self._executing_is_autonomous and self._autonomous_start_time > 0:
            used += time.monotonic() - self._autonomous_start_time
        return max(0.0, self.budget_seconds - used)

    def budget_used(self) -> float:
        """Return autonomous time used in the current window (seconds)."""
        return self.budget_seconds - self.budget_remaining()

    # ── Status ───────────────────────────────────────────────────────

    def get_status(self) -> dict:
        """Return scheduler state for UI consumption."""
        remaining = self.budget_remaining()
        return {
            'enabled': self.enabled,
            'interval': self.interval,
            'executing_goal_id': self._executing_goal_id,
            'executing_is_autonomous': self._executing_is_autonomous,
            'last_action': self._last_action,
            'last_check_time': self._last_check_time,
            'budget_total_seconds': self.budget_seconds,
            'budget_remaining_seconds': round(remaining, 1),
            'budget_window_seconds': self.budget_window_seconds,
        }

    # ── daemon loop ──────────────────────────────────────────────────

    def _run(self):
        """Daemon thread: sleep(interval), then check-and-proceed if enabled and idle."""
        while not self._stop_event.is_set():
            self._stop_event.wait(min(self.interval, 60))
            if self._stop_event.is_set():
                break
            if not self.enabled or self._executing_goal_id is not None:
                continue
            try:
                self._last_check_time = time.time()
                eligible = self._check_fn()
                if eligible:
                    goal = eligible[0]  # oldest first (already sorted by creation)
                    gid = goal.get("goal_id") or goal.get("id")
                    if not gid:
                        continue
                    is_autonomous = goal.get("origin") == "autonomous"
                    # Budget gate: autonomous goals only proceed if budget allows
                    if is_autonomous and self.budget_remaining() <= 0:
                        self._last_action = f"budget exhausted, deferring {gid}"
                        logger.info(
                            f"GoalScheduler: autonomy budget exhausted "
                            f"({self.budget_seconds/60:.0f}m window), "
                            f"deferring {gid}")
                        continue
                    self._executing_goal_id = gid
                    self._executing_is_autonomous = is_autonomous
                    if is_autonomous:
                        self._autonomous_start_time = time.monotonic()
                    self._last_action = f"auto-proceeding {gid}"
                    logger.info(f"GoalScheduler: auto-proceeding {gid} "
                                f"(\"{goal.get('name', '')[:40]}\""
                                f"{', autonomous' if is_autonomous else ''})")
                    self._proceed_fn(gid)
            except Exception as e:
                logger.error(f"GoalScheduler check error: {e}")
                self._executing_goal_id = None
                self._executing_is_autonomous = False
