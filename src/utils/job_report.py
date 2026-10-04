"""Write a background job's report into an agent's inbox folder.

The job-reports sensor delivers each file it finds there as one turn. The
report is written under `.tmp/` and renamed into the inbox, so the sensor
never reads a half-written file (Jill's condition, 2026-10-03).

From a shell:

    python3 src/utils/job_report.py --inbox DIR --job NAME --status ok|failed \\
        --started 2026-10-04T08:30:00 --body-file FILE
"""
import argparse
import os
from datetime import datetime
from pathlib import Path

STATUSES = ('ok', 'failed')


def report_name(job: str, finished: datetime) -> str:
    """The file name of a job's report. The sensor reads the job and the day
    back from it to tell whether an expected report has arrived."""
    return f"{job}-{finished:%Y-%m-%dT%H%M%S}.txt"


def write_report(inbox: Path, job: str, status: str, started: datetime,
                 body: str, finished: datetime = None) -> Path:
    if status not in STATUSES:
        raise ValueError(f"status must be one of {STATUSES}, got {status!r}")
    finished = finished or datetime.now()
    minutes = max(0, round((finished - started).total_seconds() / 60))
    text = (f"job: {job}\n"
            f"status: {status}\n"
            f"started: {started:%Y-%m-%d %H:%M}\n"
            f"finished: {finished:%Y-%m-%d %H:%M}\n"
            f"duration: {minutes} min\n\n"
            f"{body.strip()}\n")
    tmp_dir = inbox / '.tmp'
    tmp_dir.mkdir(parents=True, exist_ok=True)
    name = report_name(job, finished)
    tmp = tmp_dir / name
    tmp.write_text(text, encoding='utf-8')
    os.replace(tmp, inbox / name)
    return inbox / name


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--inbox', type=Path, required=True)
    ap.add_argument('--job', required=True)
    ap.add_argument('--status', choices=STATUSES, required=True)
    ap.add_argument('--started', required=True, help="local time, ISO format")
    ap.add_argument('--body-file', type=Path, required=True)
    a = ap.parse_args()
    started = datetime.fromisoformat(a.started).replace(tzinfo=None)
    print(write_report(a.inbox, a.job, a.status, started,
                       a.body_file.read_text(encoding='utf-8', errors='replace')))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
