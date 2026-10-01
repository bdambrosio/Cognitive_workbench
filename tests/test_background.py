"""`dispatch` results and Bruce's schedule (chat/background.py), agreed with
Jill 2026-10-01: a dispatch returns at once; its result or failure reaches
exactly one later turn, tagged; a scheduled item fires at its time, once
per occurrence, and is listed in her prompt as Bruce's."""
import queue
import sys
import threading
from datetime import datetime, timedelta
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from chat.background import BackgroundMixin, occurrence_due, next_occurrence  # noqa: E402


class _Host(BackgroundMixin):
    character_name = "TestJill"

    def __init__(self, tmp: Path, tool=None, autonomy=True):
        self._mem = tmp
        self._tool = tool
        self.turns = []
        self._inbox = queue.Queue()
        self._autonomy_enabled = autonomy
        self._current_turn = {'source': 'Claude'}
        self._init_background()

    def _counterpart_for_turn(self, source):
        return source

    def _memory_dir(self):
        return self._mem

    def _make_backend(self):
        return None

    def _bg_run_tool(self, tool, query, backend):
        return self._tool(tool, query)

    def _process_user_turn(self, **kw):
        self.turns.append(kw)


def _wait_for_results(host, n=1, timeout=5.0):
    end = datetime.now() + timedelta(seconds=timeout)
    while datetime.now() < end:
        with host._bg_lock:
            if len(host._bg_done) >= n:
                return
        threading.Event().wait(0.01)
    raise AssertionError("background work did not finish")


def test_dispatch_returns_before_the_work_ends_and_the_result_reaches_one_turn(tmp_path):
    release = threading.Event()

    def slow(tool, query):
        release.wait(5)
        return f"OK: answer to {query}"
    host = _Host(tmp_path, slow)
    obs = host._run_dispatch("lockfile", "inspect", "is Cargo.lock committed?")
    assert obs.startswith("OK: dispatched 'lockfile'")
    assert host._take_background_results() == []          # still running: nothing yet
    release.set()
    _wait_for_results(host)
    first = host._take_background_results()
    assert [(r["label"], r["result"]) for r in first] == [
        ("lockfile", "OK: answer to is Cargo.lock committed?")]
    assert host._take_background_results() == []          # delivered once only
    block = host._render_background_results(first)
    assert "lockfile — inspect: is Cargo.lock committed?" in block


def test_a_failed_dispatch_is_delivered_as_its_failure(tmp_path):
    def broken(tool, query):
        raise RuntimeError("server down")
    host = _Host(tmp_path, broken)
    host._run_dispatch("web", "search-web", "chhoto url")
    _wait_for_results(host)
    [r] = host._take_background_results()
    assert r["result"].startswith("ERROR: search-web raised: server down")


def test_dispatch_refuses_an_unknown_tool_a_running_label_and_a_fourth_job(tmp_path):
    release = threading.Event()
    host = _Host(tmp_path, lambda t, q: (release.wait(5), "OK: x")[1])
    assert host._run_dispatch("a", "security", "q").startswith("ERROR:")
    for label in ("a", "b", "c"):
        assert host._run_dispatch(label, "inspect", "q").startswith("OK:")
    assert "still running" in host._run_dispatch("a", "inspect", "q")
    assert host._run_dispatch("d", "inspect", "q").startswith("ERROR: 3 dispatches")
    release.set()


def test_occurrences_are_anchored_to_the_clock_not_to_the_last_fire():
    at = datetime(2026, 10, 1, 15, 0)
    now = datetime(2026, 10, 3, 9, 30)
    assert occurrence_due(at, 24, now) == datetime(2026, 10, 2, 15, 0)
    assert next_occurrence(at, 24, now) == datetime(2026, 10, 3, 15, 0)
    assert occurrence_due(at, None, now) == at and next_occurrence(at, None, now) is None
    assert occurrence_due(datetime(2026, 10, 4, 8, 0), None, now) is None


def test_a_due_item_fires_once_says_it_is_late_and_is_listed_as_bruces(tmp_path):
    past = (datetime.now() - timedelta(hours=3)).strftime("%Y-%m-%d %H:%M")
    future = (datetime.now() + timedelta(days=2)).strftime("%Y-%m-%d %H:%M")
    (tmp_path / "schedule.yaml").write_text(
        f"- text: Outreach figures\n  at: '{past}'\n  instruction: Report the week's sends.\n"
        f"- text: Renewal reminder\n  at: '{future}'\n  every_hours: 168\n"
        f"- text: broken item with no time\n")
    host = _Host(tmp_path)
    assert host._fire_due_schedule() is True
    [turn] = host.turns
    assert turn["autonomous"] is True and turn["source"] == "TestJill"
    assert "An item Bruce scheduled has come due: Outreach figures" in turn["text"]
    assert "runs late" in turn["text"] and "Report the week's sends." in turn["text"]
    assert host._fire_due_schedule() is False               # once per occurrence
    block = host._render_schedule_block()
    assert block.startswith("## Scheduled by Bruce")
    assert "Renewal reminder" in block and "every 168h" in block
    assert "Outreach figures" not in block                  # single item, past


def test_a_finish_wakes_her_once_and_the_woken_turn_answers_the_dispatcher(tmp_path):
    """Two finishes before the loop drains the queue make one wake-up; the
    woken turn takes both results and is addressed to whoever the
    dispatching turn was with."""
    host = _Host(tmp_path, lambda t, q: f"OK: {q}")
    host._run_dispatch("a", "inspect", "first")
    host._run_dispatch("b", "inspect", "second")
    _wait_for_results(host, n=2)
    assert host._inbox.qsize() == 1 and host._inbox.get() == {'kind': 'background'}
    host._handle_background_wake()
    [turn] = host.turns
    assert turn["autonomous"] is True and turn["counterpart"] == "Claude"
    assert "has finished: a, b" in turn["text"] or "has finished: b, a" in turn["text"]


def test_no_woken_turn_when_results_were_taken_or_autonomy_is_off(tmp_path):
    host = _Host(tmp_path, lambda t, q: "OK: x")
    host._run_dispatch("a", "inspect", "q")
    _wait_for_results(host)
    host._take_background_results()                 # another turn got there first
    host._handle_background_wake()
    assert host.turns == []
    off = _Host(tmp_path, lambda t, q: "OK: x", autonomy=False)
    off._run_dispatch("a", "inspect", "q")
    _wait_for_results(off)
    off._handle_background_wake()
    assert off.turns == [] and len(off._bg_done) == 1   # waits for the next turn
