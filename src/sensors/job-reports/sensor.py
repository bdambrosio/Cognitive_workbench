"""Job reports sensor (agreed with Jill, 2026-10-03).

A background job that the agent did not start (a timer, a workflow, Bruce)
writes one report file into the inbox folder when it ends. Each poll
delivers the oldest waiting report as one turn and moves it to `seen/`.
One report is one turn (Bruce and Jill, 2026-10-03): a second report waits
for the next poll.

Delivered means moved. Nothing is kept in memory, so a report written while
the agent was down is delivered when it starts, and a restart repeats
nothing.

`parameters.expected` lists reports that should arrive each day, each as
`{job: <name>, by: "HH:MM"}`. When the time has passed and no report of that
job carries today's date, the sensor says so, once: a marker file in `seen/`
records that the day's warning was given. A session that starts after the
time, before a late job has finished, gets the warning and then the report.

Emits a prose situation report in `content`; see rss-watcher for why.
"""
import logging
import os
from datetime import datetime
from pathlib import Path

logger = logging.getLogger(__name__)

_REPO_ROOT = Path(__file__).resolve().parents[3]
_NOTHING = {'status': 'nothing', 'content': '', 'metadata': {}}

#: Characters of a report delivered. Jill asked for 2,000 to 4,000: beyond
#: that a report is a log, and the job should have summarised it.
MAX_REPORT_CHARS = 3000

_CLOSING = ("Nobody has said anything — you noticed this yourself. Tell Bruce "
            "only if it needs him: a failure, a result far from what the "
            "report says was expected, or something waiting on him. "
            "Otherwise stay silent.")


def _deliver(path: Path, seen: Path) -> dict:
    text = path.read_text(encoding='utf-8', errors='replace')
    target = seen / path.name
    if target.exists():
        target = seen / f"{path.stem}-{datetime.now():%H%M%S%f}{path.suffix}"
    os.replace(path, target)
    cut = len(text) > MAX_REPORT_CHARS
    lines = ["A background job left a report.", "", text[:MAX_REPORT_CHARS].rstrip()]
    if cut:
        lines.append(f"(The report is cut here; the whole of it is at {target}.)")
    lines += ["", _CLOSING]
    logger.info(f"job-reports: delivered {path.name}")
    return {'status': 'ok', 'content': "\n".join(lines),
            'metadata': {'report': target.name, 'truncated': cut}}


def _missing(inbox: Path, seen: Path, expected: list, now: datetime):
    """The first expected report that is overdue today and not yet warned
    about, as (job, by), or None."""
    for item in expected:
        job, by = str(item.get('job') or ''), str(item.get('by') or '')
        try:
            due = datetime.combine(now.date(), datetime.strptime(by, "%H:%M").time())
        except ValueError:
            logger.warning(f"job-reports: expected item {item!r} needs `job` "
                           f"and `by` as HH:MM; left out")
            continue
        if not job or now < due:
            continue
        today = f"{job}-{now:%Y-%m-%d}T"
        if any(p.name.startswith(today) for d in (inbox, seen) for p in d.glob(f"{job}-*")):
            continue
        marker = seen / f".missing-{job}-{now:%Y-%m-%d}"
        if marker.exists():
            continue
        marker.write_text(now.isoformat(timespec='seconds'), encoding='utf-8')
        return job, by
    return None


def run(context):
    params = context['parameters']
    if not params.get('inbox'):
        return _NOTHING
    inbox = Path(params['inbox'])
    if not inbox.is_absolute():
        inbox = _REPO_ROOT / inbox
    seen = inbox / 'seen'
    seen.mkdir(parents=True, exist_ok=True)

    waiting = sorted((p for p in inbox.iterdir()
                      if p.is_file() and not p.name.startswith('.')),
                     key=lambda p: p.stat().st_mtime)
    if waiting:
        return _deliver(waiting[0], seen)

    now = datetime.now()
    hit = _missing(inbox, seen, params.get('expected') or [], now)
    if hit is None:
        return _NOTHING
    job, by = hit
    logger.info(f"job-reports: expected report {job} missing at {now:%H:%M}")
    return {'status': 'ok',
            'content': (f"An expected report has not arrived: `{job}` was due "
                        f"by {by} today and it is now {now:%H:%M}. No report "
                        f"of that job carries today's date. The job may not "
                        f"have started, may still be running, or may have "
                        f"stopped without writing a report.\n\n"
                        "Nobody has said anything — you noticed this "
                        "yourself. Tell Bruce once."),
            'metadata': {'missing': job}}
