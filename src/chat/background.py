"""Background results and Bruce's schedule (agreed with Jill, 2026-10-01).

BACKGROUND RESULTS. The `dispatch` action runs one existing tool, `inspect` or
`search-web`, on a thread and returns at once with the label the agent gave
it; her turn continues. Each finished result, or its failure, is held and
added to the input of her next turn of any kind (a user turn, a sensor turn,
a peer message, a concern fire, a scheduled item), tagged with its label and
what was asked, and delivered exactly once: it is taken out of the holding
list when that turn starts, whether the turn then speaks or stays silent.
A worker gets a backend of its own, so its calls never overwrite the main
backend's per-call state (`last_finish_reason`) while a turn reads it.

THE SCHEDULE. Bruce writes `<memory>/schedule.yaml`, a list of items:

    - text: Weekly outreach figures
      at: 2026-10-08 15:00          # local time
      every_hours: 168              # optional; omit for once
      instruction: Report the week's sends and replies from the outreach page.

An item fires at its time, not after an interval since its last fire. The
tick reads the file each pass, so an edit needs no restart. A due
occurrence fires once, with autonomy on; one that came due while she was
down fires late, once, and says so. Each fired occurrence is recorded in
`<memory>/schedule_fired.json` before its turn runs. Her prompt lists the
upcoming items under a heading saying Bruce wrote them, apart from the
concerns she or reflection create.
"""
from __future__ import annotations

import json
import logging
import threading
import uuid
from datetime import datetime, timedelta
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

#: The tools `dispatch` may run. Each is an existing tool; dispatch changes
#: only when its result arrives.
DISPATCHABLE = ('inspect', 'search-web')
#: Dispatches running at once. A further dispatch is refused until one ends.
MAX_RUNNING = 3
SCHEDULE_FILE = 'schedule.yaml'
SCHEDULE_FIRED_FILE = 'schedule_fired.json'
DISPATCH_LOG = 'background_dispatch.jsonl'
#: How many upcoming schedule items the prompt lists.
SCHEDULE_PROMPT_ITEMS = 5


def _now() -> datetime:
    """Local wall-clock time, naive, as Bruce writes it in the schedule."""
    return datetime.now()


def _parse_at(value: Any) -> Optional[datetime]:
    if isinstance(value, datetime):
        return value.replace(tzinfo=None)
    try:
        return datetime.fromisoformat(str(value).strip())
    except ValueError:
        return None


def occurrence_due(at: datetime, every_hours: Optional[float],
                   now: datetime) -> Optional[datetime]:
    """The latest occurrence at or before `now`, or None when none is."""
    if at > now:
        return None
    if not every_hours:
        return at
    step = timedelta(hours=float(every_hours))
    return at + step * int((now - at) / step)


def next_occurrence(at: datetime, every_hours: Optional[float],
                    now: datetime) -> Optional[datetime]:
    """The first occurrence after `now`, or None for a single item past."""
    if at > now:
        return at
    if not every_hours:
        return None
    return occurrence_due(at, every_hours, now) + timedelta(hours=float(every_hours))


class BackgroundMixin:

    # ---- dispatch ---------------------------------------------------------

    def _init_background(self) -> None:
        self._bg_lock = threading.Lock()
        self._bg_running: Dict[str, Dict[str, Any]] = {}
        self._bg_done: List[Dict[str, Any]] = []
        # Results taken at the start of the current turn; rendered into its
        # input by _build_react_user_prefix.
        self._background_turn_results: List[Dict[str, Any]] = []

    def _bg_log(self, event: Dict[str, Any]) -> None:
        from utils.file_utils import append_jsonl
        try:
            append_jsonl(self._memory_dir() / DISPATCH_LOG,
                         {'at': datetime.now().isoformat(timespec='seconds'), **event})
        except Exception as e:
            logger.warning(f"[{self.character_name}] background log write failed: {e}")

    def _run_dispatch(self, label: str, tool: str, query: str) -> str:
        label = str(label or '').strip()
        tool = str(tool or '').strip()
        query = str(query or '').strip()
        if not label:
            return "ERROR: dispatch needs a `label` to tag its result with"
        if tool not in DISPATCHABLE:
            return (f"ERROR: dispatch runs {' or '.join(DISPATCHABLE)}, "
                    f"not {tool!r}")
        if not query:
            return "ERROR: dispatch needs a `query` for the tool"
        with self._bg_lock:
            if label in self._bg_running:
                return f"ERROR: a dispatch labelled {label!r} is still running"
            if len(self._bg_running) >= MAX_RUNNING:
                return (f"ERROR: {MAX_RUNNING} dispatches are running "
                        f"({', '.join(self._bg_running)}); wait for one to finish")
            job = {'id': uuid.uuid4().hex[:8], 'label': label, 'tool': tool,
                   'query': query, 'started_at': datetime.now().isoformat(timespec='seconds')}
            self._bg_running[label] = job
        threading.Thread(target=self._bg_worker, args=(job,), daemon=True,
                         name=f"dispatch-{label}").start()
        self._bg_log({'event': 'dispatched', **job})
        return (f"OK: dispatched {label!r} ({tool}); it runs in the background and its "
                f"result, or its failure, will be in the input of your next turn")

    def _bg_run_tool(self, tool: str, query: str, backend) -> str:
        """The tool itself, on its own backend. Returns an OK:/EMPTY:/ERROR:
        observation, as the tool would inside a turn."""
        if tool == 'inspect':
            from pathlib import Path
            from chat.react import _REPO_ROOT
            from chat.subagents.code_subagent import inspect as _inspect
            answer = _inspect(
                query=query,
                repo_root=Path(getattr(self, '_inspect_root', None) or _REPO_ROOT),
                llm_backend=backend,
                trace_dir=self._inspect_traces_dir(),
                reasoning_effort=self._reasoning_effort,
                map_enabled=getattr(self, '_subagent_map', True),
                continue_id=None)
            text = str(answer or '').strip()
            return ('OK: ' + text) if text else 'EMPTY: inspect subagent produced no answer'
        mod = self._load_discovered_tool_module(tool)
        invoke = getattr(mod, 'react_invoke', None) if mod is not None else None
        if invoke is None:
            return f"ERROR: tool {tool} failed to load"
        result = invoke({'query': query}, character_name=self.character_name,
                        backend=backend, logger=logger)
        if not isinstance(result, dict):
            return f"ERROR: {tool} returned non-dict ({type(result).__name__})"
        text = str(result.get('text', '')).strip()
        status = result.get('status')
        if status == 'ok':
            return 'OK: ' + text
        if status == 'empty':
            return 'EMPTY: ' + (text or f'{tool} produced no result')
        return 'ERROR: ' + (text or f'{tool} failed')

    def _bg_worker(self, job: Dict[str, Any]) -> None:
        try:
            obs = self._bg_run_tool(job['tool'], job['query'], self._make_backend())
        except Exception as e:
            logger.warning(f"[{self.character_name}] dispatch {job['label']!r} raised: {e}")
            obs = f"ERROR: {job['tool']} raised: {e}"
        done = {**job, 'finished_at': datetime.now().isoformat(timespec='seconds'),
                'result': obs}
        with self._bg_lock:
            self._bg_running.pop(job['label'], None)
            self._bg_done.append(done)
        self._bg_log({'event': 'finished', 'id': job['id'], 'label': job['label'],
                      'status': obs.split(':', 1)[0], 'chars': len(obs)})

    def _take_background_results(self) -> List[Dict[str, Any]]:
        """Every finished result not yet delivered, removed from the holding
        list: a result reaches exactly one turn."""
        with self._bg_lock:
            taken, self._bg_done = self._bg_done, []
        for r in taken:
            self._bg_log({'event': 'delivered', 'id': r['id'], 'label': r['label']})
        return taken

    def _render_background_results(self, results: List[Dict[str, Any]]) -> str:
        lines = ["## Background results (tasks I dispatched; each is shown to me once, here)"]
        for r in results:
            lines.append(f"### {r['label']} — {r['tool']}: {r['query']}")
            lines.append(f"(dispatched {r['started_at']}, finished {r['finished_at']})")
            lines.append(r['result'])
            lines.append("")
        return "\n".join(lines)

    # ---- the schedule -----------------------------------------------------

    def _schedule_items(self) -> List[Dict[str, Any]]:
        """Bruce's schedule, parsed; an item that cannot be read is logged and
        left out."""
        f = self._memory_dir() / SCHEDULE_FILE
        if not f.is_file():
            return []
        import yaml
        try:
            raw = yaml.safe_load(f.read_text(encoding='utf-8')) or []
        except Exception as e:
            logger.warning(f"[{self.character_name}] {SCHEDULE_FILE} unreadable: {e}")
            return []
        if not isinstance(raw, list):
            logger.warning(f"[{self.character_name}] {SCHEDULE_FILE} is not a list of items")
            return []
        items = []
        for n, it in enumerate(raw, 1):
            at = _parse_at((it or {}).get('at')) if isinstance(it, dict) else None
            text = str((it or {}).get('text') or '').strip() if isinstance(it, dict) else ''
            every = (it or {}).get('every_hours') if isinstance(it, dict) else None
            try:
                every = float(every) if every not in (None, '') else None
            except (TypeError, ValueError):
                every = -1
            if at is None or not text or (every is not None and every <= 0):
                logger.warning(f"[{self.character_name}] {SCHEDULE_FILE} item {n} left out: "
                               f"it needs `text`, an `at` time, and a positive "
                               f"`every_hours` if any")
                continue
            items.append({'text': text, 'at': at, 'every_hours': every,
                          'instruction': str(it.get('instruction') or '').strip()})
        return items

    def _schedule_fired(self) -> Dict[str, str]:
        f = self._memory_dir() / SCHEDULE_FIRED_FILE
        if not f.is_file():
            return {}
        try:
            return json.loads(f.read_text(encoding='utf-8'))
        except Exception as e:
            logger.warning(f"[{self.character_name}] {SCHEDULE_FIRED_FILE} unreadable: {e}")
            return {}

    @staticmethod
    def _schedule_key(item: Dict[str, Any], occurrence: datetime) -> str:
        return f"{item['text']}|{item['at'].isoformat()}|{occurrence.isoformat()}"

    def _due_schedule_item(self, now: Optional[datetime] = None
                           ) -> Optional[Tuple[Dict[str, Any], datetime]]:
        """The earliest due occurrence not yet fired, or None."""
        now = now or _now()
        fired = self._schedule_fired()
        due = []
        for it in self._schedule_items():
            occ = occurrence_due(it['at'], it['every_hours'], now)
            if occ is not None and self._schedule_key(it, occ) not in fired:
                due.append((occ, it))
        if not due:
            return None
        occ, it = min(due, key=lambda x: x[0])
        return it, occ

    def _fire_due_schedule(self) -> bool:
        """Fire one due scheduled item as an autonomous turn. Returns True
        when one fired. The occurrence is recorded before the turn runs, so a
        crash inside the turn cannot make it fire again."""
        hit = self._due_schedule_item()
        if hit is None:
            return False
        item, occ = hit
        now = _now()
        fired = self._schedule_fired()
        fired[self._schedule_key(item, occ)] = now.isoformat(timespec='seconds')
        from utils.file_utils import atomic_write_json
        atomic_write_json(self._memory_dir() / SCHEDULE_FIRED_FILE, fired)
        late = now - occ
        when = (f"Due {occ:%Y-%m-%d %H:%M}"
                + (f"; it is now {now:%Y-%m-%d %H:%M}, so this runs late"
                   if late > timedelta(minutes=10) else ""))
        text = (f"An item Bruce scheduled has come due: {item['text']}\n"
                f"{when}\nMode: autonomous\n\n"
                f"Execute the following procedure now and produce the appropriate "
                f"output. If the procedure specifies silence under some condition, "
                f"stay silent.\n\n{item['instruction'] or item['text']}")
        logger.info(f"[{self.character_name}] scheduled item fired: {item['text']!r} ({when})")
        self._process_user_turn(source=self.character_name, text=text, close=False,
                                autonomous=True)
        return True

    def _render_schedule_block(self, now: Optional[datetime] = None) -> str:
        """Bruce's upcoming items for the prompt; empty when there are none."""
        now = now or _now()
        rows = []
        for it in self._schedule_items():
            nxt = next_occurrence(it['at'], it['every_hours'], now)
            if nxt is not None:
                rows.append((nxt, it))
        if not rows:
            return ''
        rows.sort(key=lambda x: x[0])
        lines = ["## Scheduled by Bruce (he wrote these; each fires at its time, "
                 "apart from my concerns)"]
        for nxt, it in rows[:SCHEDULE_PROMPT_ITEMS]:
            rep = (f", then every {it['every_hours']:g}h" if it['every_hours'] else "")
            lines.append(f"- next {nxt:%a %Y-%m-%d %H:%M}{rep}: {it['text']}")
        return "\n".join(lines)
