#!/usr/bin/env python3
"""Which intake and which run an engagement is currently working from.

    python3 workflowsv2/engagement_state.py <engagement> new [--clone <url or path>]
    python3 workflowsv2/engagement_state.py <engagement> status
    python3 workflowsv2/engagement_state.py <engagement> intake current <id>
    python3 workflowsv2/engagement_state.py <engagement> intake cancel <id>
    python3 workflowsv2/engagement_state.py <engagement> run current <merged dir name>
    python3 workflowsv2/engagement_state.py <engagement> run cancel <merged dir name>
    python3 workflowsv2/engagement_state.py <engagement> stage <stage> <value>

THE ENGAGEMENT COMES FIRST, EXPLICITLY. `new` creates the directory and a
stub engagement.yaml; nothing else creates one, and the intake refuses a
name it does not find. The materials live INSIDE the engagement, at
`target/` unless engagement.yaml says otherwise: the report pins what was
examined, retention is an engagement term, and deleting an engagement
should delete what it examined. `--clone` fills `target/` from a URL or a
local checkout (a local clone hardlinks objects and takes seconds).

THE MODEL (Bruce, 2026-09-03). An engagement holds intakes under
`intakes/<id>/`; one is current. A run — a merged directory under `merged/`,
the materiality ratings and the report — is made on the current intake and
pins it in its `meta.json`; one run per intake is current. Post-delivery
conversations happen over the current run of the current intake. Intakes and
runs can be cancelled or repeated.

THE RULE FOR "CURRENT": an explicit choice if one was made and is not
cancelled; else the most recent not cancelled; else none. A new intake clears
the explicit choice, so it becomes current by being most recent.

STATE IS MARKS, NEVER DELETION. `state.json` in the engagement directory holds
the explicit choices and the cancelled lists. What exists on disk is read from
disk; this file never lists intakes or runs, so it cannot disagree with them.
Engagements with no `state.json` and no `intakes/` behave as before: the
engagement.yaml blocks are the thresholds, and every merged directory is a run
of "no intake".

RETENTION (Bruce, 2026-09-04). Run directories and merged directories are the
record: they live as long as the engagement's `retention:` term says and go
when the engagement goes, never on their own. The world a stage ran in is
scratch once its run directory exists, because the runner copies the working
record there; `workflowsv2/sweep_worlds.py` deletes such worlds after 30
days. Post-delivery and intake worlds are conversations and are kept.

STAGES AND JOBS (2026-09-05, the pilot site). `state.json` also holds
`stages`, one mark per stage of the engagement as the site walks it (STAGES
below, each `{"value", "at", "by"}`), and `jobs`, the record of every
enumeration or chain job the site started: when, which steps, the exit. A
job in state "running" is the engagement's lock: a second job is refused
until it ends. Marks again, never deletion.
"""
from __future__ import annotations

import datetime
import functools
import json
import sys
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
if str(REPO / "src") not in sys.path:
    sys.path.insert(0, str(REPO / "src"))

from utils.file_utils import atomic_write_text                  # noqa: E402

#: ONE WRITER AT A TIME, WITHIN A PROCESS. The site's job thread and its
#: request handlers both read-modify-write state.json; without this a torn
#: read (an empty file mid-write) and a lost update were both observed in
#: tests on 2026-09-05. Across processes the atomic replace in `save` keeps
#: every read whole; two processes writing the same engagement at once is
#: not a case the site produces.
_LOCK = threading.RLock()


def _locked(fn):
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        with _LOCK:
            return fn(*args, **kwargs)
    return wrapper
ENGAGEMENTS = REPO / "workflowsv2" / "claims_audit" / "engagements"
STATE_FILE = "state.json"
INTAKES = "intakes"
MERGED = "merged"
#: What an intake's `--finish` writes: the blocks a run reads.
BLOCKS_FILE = "blocks.yaml"

#: The stages of an engagement, in the order the site walks them. The value
#: of a mark is a short word: "accepted", "done", "ready", "running",
#: "failed", "frozen", "released", "closed".
STAGES = ("created", "letter", "intake", "materials", "enumeration",
          "surface", "chain", "release", "closed")

#: Where the frozen claim surfaces and the client's comments live.
SURFACE = "surface"
#: Where job logs live.
JOBS = "jobs"


def _empty() -> Dict[str, Any]:
    return {"current_intake": None, "current_run": {},
            "cancelled": {"intakes": [], "runs": []},
            "stages": {}, "jobs": []}


def load(eng_dir: Path) -> Dict[str, Any]:
    p = eng_dir / STATE_FILE
    if not p.is_file():
        return _empty()
    st = _empty()
    st.update(json.loads(p.read_text(encoding="utf-8")))
    st["cancelled"] = {**_empty()["cancelled"], **(st.get("cancelled") or {})}
    return st


def save(eng_dir: Path, st: Dict[str, Any]) -> None:
    atomic_write_text(eng_dir / STATE_FILE, json.dumps(st, indent=1) + "\n")


def stamp() -> str:
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")


# ---- the engagement itself ---------------------------------------------------

TARGET = "target"
STUB = """\
# claims-audit engagement: {name}
#
# Created {stamp}. The practice fills `claim_sources:` (the documents in the
# target that carry the seller's claims, by path from the target root) and,
# where the materials are not under target/, `target:`. `transaction:` and
# `thresholds:` come from the current intake's blocks.yaml, not from here.

target: target

claim_sources: []

# Who may open this engagement's pages on the client site, by email.
client_emails: {client_emails}

retention: keep
"""


def target_dir(eng_dir: Path) -> Path:
    """Where the engagement's materials are: `target:` from engagement.yaml
    when it names one (relative to the engagement, else to the repo), else
    `target/` in the engagement. The same rule as load_engagement."""
    import yaml
    cfg_file = eng_dir / "engagement.yaml"
    cfg = (yaml.safe_load(cfg_file.read_text(encoding="utf-8")) or {}) \
        if cfg_file.is_file() else {}
    t = Path(cfg.get("target") or TARGET)
    if t.is_absolute():
        return t
    return (eng_dir / t) if (eng_dir / t).is_dir() else REPO / t


def new_engagement(eng_dir: Path, clone: Optional[str] = None,
                   client_emails: Optional[List[str]] = None,
                   by: Optional[str] = None) -> Path:
    """Create the engagement: its directory, a stub engagement.yaml (with
    the client's emails when given), and — with `clone` — its target/ from
    a git URL or a local checkout. Marks the `created` stage."""
    if eng_dir.exists():
        raise SystemExit(f"engagement '{eng_dir.name}' already exists")
    eng_dir.mkdir(parents=True)
    emails = json.dumps([e.strip() for e in (client_emails or []) if e.strip()])
    (eng_dir / "engagement.yaml").write_text(
        STUB.format(name=eng_dir.name, stamp=stamp(), client_emails=emails),
        encoding="utf-8")
    set_stage(eng_dir, "created", "done", by)
    if clone:
        clone_target(eng_dir, clone)
    return eng_dir


def clone_target(eng_dir: Path, url: str) -> Path:
    """Fill the engagement's target/ from a git URL or a local checkout.
    Refuses when target/ already holds something: the practice removes it
    by hand first, so a run's materials are never replaced by accident."""
    import subprocess
    dest = eng_dir / TARGET
    if dest.exists() and any(dest.iterdir()):
        raise SystemExit(f"{dest} already holds materials; remove it first to clone again")
    r = subprocess.run(["git", "clone", "--quiet", url, str(dest)],
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(f"git clone failed: {r.stderr.strip()}")
    return dest


def _engagement_yaml(eng_dir: Path) -> Dict[str, Any]:
    import yaml
    f = eng_dir / "engagement.yaml"
    return (yaml.safe_load(f.read_text(encoding="utf-8")) or {}) if f.is_file() else {}


def client_emails(eng_dir: Path) -> List[str]:
    """The emails allowed to open this engagement on the client site."""
    return [str(e).strip().lower() for e in (_engagement_yaml(eng_dir).get("client_emails") or [])
            if str(e).strip()]


def claim_sources(eng_dir: Path) -> List[str]:
    return [str(c) for c in (_engagement_yaml(eng_dir).get("claim_sources") or [])]


#: The keys of engagement.yaml the practice page may set. Everything else
#: in the file is kept as it is; the file's comments are not (yaml rewrites).
SETTABLE = ("claim_sources", "client_emails", "target", "retention")


@_locked
def update_engagement(eng_dir: Path, **fields: Any) -> Dict[str, Any]:
    """Set some of SETTABLE in engagement.yaml and return the whole file as
    data. A key given as None is left alone; an empty list is written."""
    import yaml
    bad = [k for k in fields if k not in SETTABLE]
    if bad:
        raise SystemExit(f"not settable: {', '.join(bad)} (have: {', '.join(SETTABLE)})")
    cfg = _engagement_yaml(eng_dir)
    for k, v in fields.items():
        if v is None:
            continue
        if k in ("claim_sources", "client_emails"):
            v = [str(x).strip() for x in v if str(x).strip()]
        else:
            v = str(v).strip()
        cfg[k] = v
    atomic_write_text(eng_dir / "engagement.yaml",
                      yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True))
    return cfg


# ---- stages -----------------------------------------------------------------

@_locked
def set_stage(eng_dir: Path, name: str, value: str, by: Optional[str] = None) -> None:
    if name not in STAGES:
        raise SystemExit(f"no stage '{name}' (have: {', '.join(STAGES)})")
    st = load(eng_dir)
    st.setdefault("stages", {})[name] = {"value": value, "at": stamp(), "by": by}
    save(eng_dir, st)


def stage(eng_dir: Path, name: str) -> Optional[Dict[str, Any]]:
    """The mark for a stage, or None when it has none."""
    return (load(eng_dir).get("stages") or {}).get(name)


def stage_value(eng_dir: Path, name: str) -> Optional[str]:
    m = stage(eng_dir, name)
    return m.get("value") if m else None


# ---- jobs -------------------------------------------------------------------

def jobs(eng_dir: Path) -> List[Dict[str, Any]]:
    return list(load(eng_dir).get("jobs") or [])


def running_job(eng_dir: Path) -> Optional[Dict[str, Any]]:
    """The job in state "running", which is the engagement's lock."""
    for j in jobs(eng_dir):
        if j.get("state") == "running":
            return j
    return None


@_locked
def add_job(eng_dir: Path, kind: str, by: Optional[str] = None,
            log: Optional[str] = None) -> Dict[str, Any]:
    """Record a job as running. Refuses while another job is running: the
    site's buttons must not overlap an enumeration with a chain, or a chain
    with itself."""
    running = running_job(eng_dir)
    if running:
        raise SystemExit(f"job {running['id']} ({running['kind']}) is still running")
    job = {"id": stamp(), "kind": kind, "state": "running", "by": by,
           "started": stamp(), "ended": None, "exit": None, "error": None,
           "log": log, "steps": []}
    st = load(eng_dir)
    if any(j["id"] == job["id"] for j in st["jobs"]):        # two in one second
        job["id"] += "-2"
    st["jobs"].append(job)
    save(eng_dir, st)
    return job


@_locked
def update_job(eng_dir: Path, job_id: str, **fields: Any) -> Dict[str, Any]:
    st = load(eng_dir)
    for j in st["jobs"]:
        if j["id"] == job_id:
            j.update(fields)
            save(eng_dir, st)
            return j
    raise SystemExit(f"no job '{job_id}'")


@_locked
def finish_job(eng_dir: Path, job_id: str, exit_code: int,
               error: Optional[str] = None) -> Dict[str, Any]:
    return update_job(eng_dir, job_id, state="done" if exit_code == 0 else "failed",
                      ended=stamp(), exit=exit_code, error=error)


# ---- intakes ---------------------------------------------------------------

def intakes(eng_dir: Path) -> List[str]:
    """Every intake id on disk, oldest first."""
    d = eng_dir / INTAKES
    return sorted(p.name for p in d.iterdir() if p.is_dir()) if d.is_dir() else []


def current_intake(eng_dir: Path) -> Optional[str]:
    st = load(eng_dir)
    cancelled = set(st["cancelled"]["intakes"])
    explicit = st.get("current_intake")
    if explicit and explicit in intakes(eng_dir) and explicit not in cancelled:
        return explicit
    live = [i for i in intakes(eng_dir) if i not in cancelled]
    return live[-1] if live else None


def intake_dir(eng_dir: Path, intake_id: str) -> Path:
    return eng_dir / INTAKES / intake_id


@_locked
def new_intake(eng_dir: Path) -> str:
    """Create an intake directory and make it current by clearing the
    explicit choice: most recent wins."""
    iid = stamp()
    n = 1
    while intake_dir(eng_dir, iid).exists():          # two in one second
        n += 1
        iid = f"{stamp()}-{n}"
    intake_dir(eng_dir, iid).mkdir(parents=True, exist_ok=False)
    st = load(eng_dir)
    st["current_intake"] = None
    save(eng_dir, st)
    return iid


@_locked
def set_current_intake(eng_dir: Path, intake_id: str) -> None:
    if intake_id not in intakes(eng_dir):
        raise SystemExit(f"no intake '{intake_id}' (have: {', '.join(intakes(eng_dir)) or 'none'})")
    st = load(eng_dir)
    st["current_intake"] = intake_id
    save(eng_dir, st)


@_locked
def cancel_intake(eng_dir: Path, intake_id: str) -> None:
    if intake_id not in intakes(eng_dir):
        raise SystemExit(f"no intake '{intake_id}'")
    st = load(eng_dir)
    if intake_id not in st["cancelled"]["intakes"]:
        st["cancelled"]["intakes"].append(intake_id)
    save(eng_dir, st)


def intake_blocks(eng_dir: Path, intake_id: str) -> Dict[str, Any]:
    """The `transaction` and `thresholds` an intake's --finish wrote, or an
    empty dict when the intake was not finished."""
    import yaml
    p = intake_dir(eng_dir, intake_id) / BLOCKS_FILE
    if not p.is_file():
        return {}
    return yaml.safe_load(p.read_text(encoding="utf-8")) or {}


# ---- runs -------------------------------------------------------------------

def run_intake(run_dir: Path) -> Optional[str]:
    """The intake a merged directory pinned in its meta.json; None when it
    pinned none or predates the pin."""
    p = run_dir / "meta.json"
    if not p.is_file():
        return None
    try:
        return json.loads(p.read_text(encoding="utf-8")).get("intake")
    except (OSError, ValueError):
        return None


def runs(eng_dir: Path) -> List[Path]:
    """Every merged directory holding a meta.json, oldest first."""
    d = eng_dir / MERGED
    return sorted(p for p in d.iterdir() if (p / "meta.json").is_file()) \
        if d.is_dir() else []


def runs_for(eng_dir: Path, intake_id: Optional[str]) -> List[Path]:
    return [r for r in runs(eng_dir) if run_intake(r) == intake_id]


def current_run(eng_dir: Path, intake_id: Optional[str]) -> Optional[Path]:
    st = load(eng_dir)
    cancelled = set(st["cancelled"]["runs"])
    candidates = [r for r in runs_for(eng_dir, intake_id) if r.name not in cancelled]
    explicit = (st.get("current_run") or {}).get(intake_id or "")
    for r in candidates:
        if r.name == explicit:
            return r
    return candidates[-1] if candidates else None


@_locked
def set_current_run(eng_dir: Path, run_name: str) -> None:
    match = [r for r in runs(eng_dir) if r.name == run_name]
    if not match:
        raise SystemExit(f"no run '{run_name}' under {eng_dir / MERGED}")
    st = load(eng_dir)
    st.setdefault("current_run", {})[run_intake(match[0]) or ""] = run_name
    save(eng_dir, st)


@_locked
def cancel_run(eng_dir: Path, run_name: str) -> None:
    if not any(r.name == run_name for r in runs(eng_dir)):
        raise SystemExit(f"no run '{run_name}' under {eng_dir / MERGED}")
    st = load(eng_dir)
    if run_name not in st["cancelled"]["runs"]:
        st["cancelled"]["runs"].append(run_name)
    save(eng_dir, st)


# ---- status -----------------------------------------------------------------

def summary(eng_dir: Path) -> Dict[str, Any]:
    """The engagement's state as data, for the practice page: every intake
    and run on disk with its marks. `status` is the same as text."""
    st = load(eng_dir)
    ci = current_intake(eng_dir)
    pinned = sorted({run_intake(r) for r in runs(eng_dir)} - {None})
    ids = sorted(set(intakes(eng_dir)) | set(pinned))
    out: Dict[str, Any] = {"name": eng_dir.name, "current_intake": ci,
                           "has_engagement_yaml": (eng_dir / "engagement.yaml").is_file(),
                           "has_target": target_dir(eng_dir).is_dir(),
                           "client_emails": client_emails(eng_dir),
                           "claim_sources": claim_sources(eng_dir),
                           "stages": st.get("stages") or {},
                           "job": running_job(eng_dir),
                           "jobs": st.get("jobs") or [],
                           "intakes": [], "runs_without_intake": []}
    for i in ids:
        cr = current_run(eng_dir, i)
        out["intakes"].append({
            "id": i, "current": i == ci,
            "cancelled": i in st["cancelled"]["intakes"],
            "finished": bool(intake_blocks(eng_dir, i)),
            "on_disk": i in intakes(eng_dir),
            "runs": [{"name": r.name, "current": bool(cr and r.name == cr.name),
                      "cancelled": r.name in st["cancelled"]["runs"],
                      "report": (r / "report.md").is_file()}
                     for r in runs_for(eng_dir, i)]})
    cr = current_run(eng_dir, None)
    out["runs_without_intake"] = [
        {"name": r.name, "current": bool(cr and r.name == cr.name),
         "cancelled": r.name in st["cancelled"]["runs"],
         "report": (r / "report.md").is_file()} for r in runs_for(eng_dir, None)]
    return out


def engagements() -> List[str]:
    """Every engagement directory, by name."""
    return sorted(p.name for p in ENGAGEMENTS.iterdir() if p.is_dir()) \
        if ENGAGEMENTS.is_dir() else []


def status(eng_dir: Path) -> str:
    st = load(eng_dir)
    ci = current_intake(eng_dir)
    lines = [f"engagement {eng_dir.name}", "intakes:"]
    for i in intakes(eng_dir):
        marks = []
        if i == ci:
            marks.append("CURRENT")
        if i in st["cancelled"]["intakes"]:
            marks.append("cancelled")
        if not intake_blocks(eng_dir, i):
            marks.append("not finished")
        lines.append(f"  {i}  {' '.join(marks)}")
    if not intakes(eng_dir):
        lines.append("  (none — thresholds come from engagement.yaml)")
    lines.append("runs:")
    seen = 0
    # A run may pin an intake no longer on disk; it is still a run.
    pinned = sorted({run_intake(r) for r in runs(eng_dir)} - {None})
    for i in [None] + sorted(set(intakes(eng_dir)) | set(pinned)):
        rs = runs_for(eng_dir, i)
        if not rs:
            continue
        cr = current_run(eng_dir, i)
        lines.append(f"  on intake {i or '(none)'}:")
        for r in rs:
            marks = []
            if cr and r.name == cr.name:
                marks.append("CURRENT")
            if r.name in st["cancelled"]["runs"]:
                marks.append("cancelled")
            if (r / "report.md").is_file():
                marks.append("report")
            lines.append(f"    {r.name}  {' '.join(marks)}")
            seen += 1
    if not seen:
        lines.append("  (none)")
    stages = st.get("stages") or {}
    if stages:
        lines.append("stages:")
        for name in STAGES:
            m = stages.get(name)
            if m:
                lines.append(f"  {name}  {m['value']}  {m['at']}  {m.get('by') or ''}".rstrip())
    rj = running_job(eng_dir)
    if rj:
        lines.append(f"job running: {rj['id']} ({rj['kind']}) since {rj['started']}")
    return "\n".join(lines)


def main(argv: Optional[List[str]] = None) -> int:
    a = list(sys.argv[1:] if argv is None else argv)
    if len(a) < 2:
        print(__doc__)
        return 2
    eng_dir = ENGAGEMENTS / a[0]
    cmd = a[1:]
    if cmd[0] == "new":
        clone = cmd[2] if len(cmd) == 3 and cmd[1] == "--clone" else None
        if len(cmd) not in (1, 3) or (len(cmd) == 3 and cmd[1] != "--clone"):
            print(__doc__)
            return 2
        new_engagement(eng_dir, clone)
        print(f"created {eng_dir}" + (f"; target cloned from {clone}" if clone else
                                      f"; put the materials in {eng_dir / TARGET}"))
        print(f"next: fill claim_sources: in {eng_dir / 'engagement.yaml'}")
        return 0
    if not eng_dir.is_dir():
        raise SystemExit(f"no engagement '{a[0]}' in {ENGAGEMENTS} — create it "
                         f"with: engagement_state.py {a[0]} new")
    if cmd == ["status"]:
        print(status(eng_dir))
    elif len(cmd) == 3 and cmd[0] == "intake" and cmd[1] == "current":
        set_current_intake(eng_dir, cmd[2])
        print(status(eng_dir))
    elif len(cmd) == 3 and cmd[0] == "intake" and cmd[1] == "cancel":
        cancel_intake(eng_dir, cmd[2])
        print(status(eng_dir))
    elif len(cmd) == 3 and cmd[0] == "run" and cmd[1] == "current":
        set_current_run(eng_dir, cmd[2])
        print(status(eng_dir))
    elif len(cmd) == 3 and cmd[0] == "run" and cmd[1] == "cancel":
        cancel_run(eng_dir, cmd[2])
        print(status(eng_dir))
    elif len(cmd) == 3 and cmd[0] == "stage":
        set_stage(eng_dir, cmd[1], cmd[2], by="cli")
        print(status(eng_dir))
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
