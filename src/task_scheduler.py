"""
Autonomous Task Scheduler

Timer-based coordinator that periodically checks for eligible tasks and
auto-proceeds them by enqueuing synthetic "proceed" commands into the
existing text_input_queue.  One task at a time — the executing guard
(_executing_task_id) is set before proceed and cleared by step/terminal
notifications from the executive node.
"""

import threading
import logging
import time

logger = logging.getLogger('task_scheduler')


class TaskScheduler:
    def __init__(self, character_name: str, interval: float = 15.0, enabled: bool = False):
        self.character_name = character_name
        self.interval = interval          # seconds between checks
        self.enabled = enabled
        self._executing_task_id = None    # one-at-a-time guard
        self._stop_event = threading.Event()
        self._thread = None
        self._check_fn = None             # callback: () -> list[eligible tasks]
        self._proceed_fn = None           # callback: (task_id) -> None
        self._last_action = ""
        self._last_check_time = 0.0

    def start(self, check_fn, proceed_fn):
        """Start the scheduler daemon thread."""
        self._check_fn = check_fn
        self._proceed_fn = proceed_fn
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name=f"task-scheduler-{self.character_name}")
        self._thread.start()
        logger.info(f"TaskScheduler started for {self.character_name} "
                     f"(interval={self.interval}s, enabled={self.enabled})")

    def stop(self):
        """Stop the scheduler thread."""
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5.0)
        self._thread = None
        logger.info(f"TaskScheduler stopped for {self.character_name}")

    def set_enabled(self, enabled: bool):
        self.enabled = enabled
        self._last_action = f"{'enabled' if enabled else 'disabled'} by user"
        logger.info(f"TaskScheduler {self._last_action} for {self.character_name}")

    def set_interval(self, seconds: float):
        seconds = max(60.0, min(86400.0, seconds))
        self.interval = seconds
        logger.info(f"TaskScheduler interval set to {seconds}s for {self.character_name}")

    def notify_step_completed(self, task_id: str):
        """Called after a task step finishes — clears the executing guard."""
        if self._executing_task_id == task_id:
            logger.info(f"TaskScheduler: step completed for {task_id}, clearing guard")
            self._executing_task_id = None

    def notify_task_terminal(self, task_id: str):
        """Called when a task reaches a terminal state (completed/blocked/abandoned)."""
        if self._executing_task_id == task_id:
            logger.info(f"TaskScheduler: terminal state for {task_id}, clearing guard")
            self._executing_task_id = None

    def get_status(self) -> dict:
        """Return scheduler state for UI consumption."""
        return {
            'enabled': self.enabled,
            'interval': self.interval,
            'executing_task_id': self._executing_task_id,
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
            if not self.enabled or self._executing_task_id is not None:
                continue
            try:
                self._last_check_time = time.time()
                eligible = self._check_fn()
                if eligible:
                    task = eligible[0]  # oldest first (already sorted by creation)
                    tid = task.get("task_id") or task.get("goal_id") or task.get("id")
                    if not tid:
                        continue
                    self._executing_task_id = tid
                    self._last_action = f"auto-proceeding {tid}"
                    logger.info(f"TaskScheduler: auto-proceeding {tid} "
                                f"(\"{task.get('name', '')[:40]}\")")
                    self._proceed_fn(tid)
            except Exception as e:
                logger.error(f"TaskScheduler check error: {e}")
                self._executing_task_id = None
