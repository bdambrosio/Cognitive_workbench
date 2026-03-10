"""
Autonomous Goal Scheduler

Timer-based coordinator that periodically checks for eligible goals and
auto-proceeds them by enqueuing synthetic "proceed" commands into the
existing text_input_queue.  One goal at a time — the executing guard
(_executing_goal_id) is set before proceed and cleared by step/terminal
notifications from the executive node.
"""

import threading
import logging
import time

logger = logging.getLogger('goal_scheduler')


class GoalScheduler:
    def __init__(self, character_name: str, interval: float = 15.0, enabled: bool = False):
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

    def start(self, check_fn, proceed_fn):
        """Start the scheduler daemon thread."""
        self._check_fn = check_fn
        self._proceed_fn = proceed_fn
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name=f"goal-scheduler-{self.character_name}")
        self._thread.start()
        logger.info(f"GoalScheduler started for {self.character_name} "
                     f"(interval={self.interval}s, enabled={self.enabled})")

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
            self._executing_goal_id = None

    def notify_goal_terminal(self, goal_id: str):
        """Called when a goal reaches a terminal state (completed/blocked/abandoned)."""
        if self._executing_goal_id == goal_id:
            logger.info(f"GoalScheduler: terminal state for {goal_id}, clearing guard")
            self._executing_goal_id = None

    def get_status(self) -> dict:
        """Return scheduler state for UI consumption."""
        return {
            'enabled': self.enabled,
            'interval': self.interval,
            'executing_goal_id': self._executing_goal_id,
            'last_action': self._last_action,
            'last_check_time': self._last_check_time,
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
                    self._executing_goal_id = gid
                    self._last_action = f"auto-proceeding {gid}"
                    logger.info(f"GoalScheduler: auto-proceeding {gid} "
                                f"(\"{goal.get('name', '')[:40]}\")")
                    self._proceed_fn(gid)
            except Exception as e:
                logger.error(f"GoalScheduler check error: {e}")
                self._executing_goal_id = None
