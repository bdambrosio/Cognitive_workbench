#!/usr/bin/env python3
"""Which intake and which run an engagement is currently working from.

    python3 workflowsv2/engagement_state.py <engagement> new [--clone <url or path>]
    python3 workflowsv2/engagement_state.py <engagement> status
    python3 workflowsv2/engagement_state.py <engagement> intake current <id>
    python3 workflowsv2/engagement_state.py <engagement> intake cancel <id>
    python3 workflowsv2/engagement_state.py <engagement> run current <merged dir name>
    python3 workflowsv2/engagement_state.py <engagement> run cancel <merged dir name>

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
"""
from __future__ import annotations

import datetime
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
ENGAGEMENTS = REPO / "workflowsv2" / "claims_audit" / "engagements"
STATE_FILE = "state.json"
INTAKES = "intakes"
MERGED = "merged"
#: What an intake's `--finish` writes: the blocks a run reads.
BLOCKS_FILE = "blocks.yaml"


def _empty() -> Dict[str, Any]:
    return {"current_intake": None, "current_run": {},
            "cancelled": {"intakes": [], "runs": []}}


def load(eng_dir: Path) -> Dict[str, Any]:
    p = eng_dir / STATE_FILE
    if not p.is_file():
        return _empty()
    st = _empty()
    st.update(json.loads(p.read_text(encoding="utf-8")))
    st["cancelled"] = {**_empty()["cancelled"], **(st.get("cancelled") or {})}
    return st


def save(eng_dir: Path, st: Dict[str, Any]) -> None:
    (eng_dir / STATE_FILE).write_text(
        json.dumps(st, indent=1) + "\n", encoding="utf-8")


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


def new_engagement(eng_dir: Path, clone: Optional[str] = None) -> Path:
    """Create the engagement: its directory, a stub engagement.yaml, and —
    with `clone` — its target/ from a git URL or a local checkout."""
    if eng_dir.exists():
        raise SystemExit(f"engagement '{eng_dir.name}' already exists")
    eng_dir.mkdir(parents=True)
    (eng_dir / "engagement.yaml").write_text(
        STUB.format(name=eng_dir.name, stamp=stamp()), encoding="utf-8")
    if clone:
        import subprocess
        r = subprocess.run(["git", "clone", "--quiet", clone, str(eng_dir / TARGET)],
                           capture_output=True, text=True)
        if r.returncode != 0:
            raise SystemExit(f"git clone failed: {r.stderr.strip()}")
    return eng_dir


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


def set_current_intake(eng_dir: Path, intake_id: str) -> None:
    if intake_id not in intakes(eng_dir):
        raise SystemExit(f"no intake '{intake_id}' (have: {', '.join(intakes(eng_dir)) or 'none'})")
    st = load(eng_dir)
    st["current_intake"] = intake_id
    save(eng_dir, st)


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


def set_current_run(eng_dir: Path, run_name: str) -> None:
    match = [r for r in runs(eng_dir) if r.name == run_name]
    if not match:
        raise SystemExit(f"no run '{run_name}' under {eng_dir / MERGED}")
    st = load(eng_dir)
    st.setdefault("current_run", {})[run_intake(match[0]) or ""] = run_name
    save(eng_dir, st)


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
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
