#!/usr/bin/env python3
"""The two jobs the site's buttons start, run as subprocesses of the
existing runners and watched to their end.

    enumerate   one claims_audit run per claim source with --enumerate-only;
                leaves a claims.json per source for the surface page
    chain       per claim source: the audit on the frozen surface, then its
                review; then materiality over every run, against the current
                intake; then the report

ONE JOB AT A TIME PER ENGAGEMENT, MANY ENGAGEMENTS AT ONCE. The lock is the
job record in state.json (engagement_state.add_job refuses while one is
running). There is no queue across engagements: Fireworks carries several
chains at once, and the pilot site has at most a handful (Bruce,
2026-09-05).

EACH STEP IS THE RUNNER'S OWN COMMAND LINE, the same one the practice page
shows for copying. A step that exits non-zero ends the job as failed, with
the step named; later steps do not run; the practice is mailed; the client
never is. Output of every step goes to `<engagement>/jobs/<job id>.log`.

The run directory an audit produces is timestamped, so the chain finds it
after the step by its world name, the way the chain scripts in logs/ did.
"""
from __future__ import annotations

import logging
import re
import subprocess
import sys
import threading
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2 import engagement_state as state               # noqa: E402
from client_ui import mail                                      # noqa: E402

logger = logging.getLogger("client_ui.jobs")

MODEL = "measure/models/fw_glm53flash.yaml"
KINDS = ("enumerate", "chain")

Step = Tuple[str, List[str]]


class JobRunning(Exception):
    """A job is already running for this engagement."""


def slug(claim_source: str) -> str:
    """A claim source path as a world-name fragment: README.md -> readme_md."""
    return re.sub(r"[^a-z0-9]+", "_", claim_source.lower()).strip("_")


def commands(name: str, s: Dict[str, Any]) -> Dict[str, str]:
    """The command lines for this engagement, with the current run filled
    in where one exists. Shown on the practice page to be copied."""
    ci = s.get("current_intake")
    cur = None
    for i in s.get("intakes") or []:
        if i["id"] == ci:
            cur = next((r["name"] for r in i["runs"] if r["current"]), None)
    eng = f"workflowsv2/claims_audit/engagements/{name}"
    return {
        "intake": f"python3 src/client_ui/app.py intake --engagement {name} --model {MODEL} --port 8800",
        "intake_new": f"python3 src/client_ui/app.py intake --engagement {name} --new --model {MODEL} --port 8800",
        "finish": f"python3 workflowsv2/intake/runner.py --engagement {name} --finish",
        "audit": f"python3 workflowsv2/claims_audit/runner.py --engagement {name} --world <fresh world> --claim-source <one of claim_sources> --model {MODEL}",
        "review": f"python3 workflowsv2/audit_review/runner.py --run {eng}/runs/<run dir> --model {MODEL}",
        "materiality": f"python3 workflowsv2/audit_materiality/runner.py --engagement {name} --run {eng}/runs/<run dir> [--run ...] --model {MODEL} --label <label>",
        "report": f"python3 workflowsv2/audit_report/runner.py --merged {eng}/merged/{cur or '<merged dir>'} --model {MODEL}",
        "post": f"python3 src/client_ui/app.py post --engagement {name} --model {MODEL} --port 8801",
    }


# ---- the steps ----------------------------------------------------------------

def _py(script: str, *args: str) -> List[str]:
    return [sys.executable, str(REPO / script), *args]


def _newest(parent: Path, suffix: str) -> Optional[Path]:
    """The most recent directory under `parent` whose name ends in
    `_<suffix>` — how a run or merged directory is found from its world or
    label after the step that made it."""
    if not parent.is_dir():
        return None
    hits = sorted(p for p in parent.iterdir() if p.is_dir() and p.name.endswith("_" + suffix))
    return hits[-1] if hits else None


def surface_file(eng_dir: Path, claim_source: str) -> Path:
    return eng_dir / state.SURFACE / f"{slug(claim_source)}.surface.json"


def latest_enumeration_run(eng_dir: Path, claim_source: str) -> Optional[Path]:
    """The most recent enumeration run of this source that left a
    claims.json, by the world-name fragment enumerate_steps gives it."""
    runs = eng_dir / "runs"
    if not runs.is_dir():
        return None
    frag = f"_enum_{eng_dir.name}_{slug(claim_source)}_"
    hits = sorted(p for p in runs.iterdir()
                  if p.is_dir() and frag in p.name and (p / "claims.json").is_file())
    return hits[-1] if hits else None


def enumerate_steps(eng_dir: Path, model: str, ts: str) -> List[Step]:
    steps: List[Step] = []
    for src in state.claim_sources(eng_dir):
        world = f"enum_{eng_dir.name}_{slug(src)}_{ts}"
        steps.append((f"enumerate {src}", _py(
            "workflowsv2/claims_audit/runner.py", "--engagement", eng_dir.name,
            "--world", world, "--claim-source", src, "--enumerate-only",
            "--model", model)))
    return steps


class Chain:
    """The chain's steps are built as it goes, because each audit's run
    directory is known only after it ran."""

    def __init__(self, eng_dir: Path, model: str, ts: str) -> None:
        self.eng_dir, self.model, self.ts = eng_dir, model, ts
        self.sources = state.claim_sources(eng_dir)
        self.run_dirs: List[Path] = []

    def check(self) -> Optional[str]:
        if not self.sources:
            return "engagement.yaml names no claim_sources"
        missing = [s for s in self.sources if not surface_file(self.eng_dir, s).is_file()]
        if missing:
            return "no frozen surface for " + ", ".join(missing)
        return None

    def steps(self) -> "Iterator[Step]":                       # noqa: F821
        for src in self.sources:
            world = f"audit_{self.eng_dir.name}_{slug(src)}_{self.ts}"
            yield (f"audit {src}", _py(
                "workflowsv2/claims_audit/runner.py", "--engagement", self.eng_dir.name,
                "--world", world, "--claim-source", src,
                "--surface", str(surface_file(self.eng_dir, src)), "--model", self.model))
            run_dir = _newest(self.eng_dir / "runs", world)
            if run_dir is None:
                raise RuntimeError(f"the audit of {src} left no run directory")
            self.run_dirs.append(run_dir)
            yield (f"review {src}", _py(
                "workflowsv2/audit_review/runner.py", "--run", str(run_dir),
                "--model", self.model, "--world", f"review_{world}"))
        label = f"chain_{self.ts}"
        args = ["--engagement", self.eng_dir.name]
        for r in self.run_dirs:
            args += ["--run", str(r)]
        intake = state.current_intake(self.eng_dir)
        if intake:
            args += ["--intake", intake]
        yield ("materiality", _py("workflowsv2/audit_materiality/runner.py",
                                  *args, "--model", self.model, "--label", label))
        merged = _newest(self.eng_dir / state.MERGED, label)
        if merged is None:
            raise RuntimeError("materiality left no merged directory")
        yield ("report", _py("workflowsv2/audit_report/runner.py",
                             "--merged", str(merged), "--model", self.model))


# ---- running ----------------------------------------------------------------

#: Test seam: replaced to run something other than the real runners.
_steps_for: Optional[Callable[[Path, str, str, str], Any]] = None


def _iter_steps(eng_dir: Path, kind: str, model: str, ts: str):
    if _steps_for is not None:
        return _steps_for(eng_dir, kind, model, ts)
    if kind == "enumerate":
        return iter(enumerate_steps(eng_dir, model, ts))
    return Chain(eng_dir, model, ts).steps()


def _precheck(eng_dir: Path, kind: str, model: str, ts: str) -> Optional[str]:
    if _steps_for is not None:
        return None
    if kind == "enumerate":
        return None if state.claim_sources(eng_dir) else "engagement.yaml names no claim_sources"
    return Chain(eng_dir, model, ts).check()


class Job:
    def __init__(self, eng_dir: Path, record: Dict[str, Any], thread: threading.Thread) -> None:
        self.eng_dir, self.record, self.thread = eng_dir, record, thread

    @property
    def id(self) -> str:
        return self.record["id"]

    def wait(self, timeout: Optional[float] = None) -> None:
        self.thread.join(timeout)


def start(eng_dir: Path, kind: str, by: Optional[str] = None,
          model: str = MODEL) -> Job:
    """Start a job for the engagement and return at once; the job runs in a
    thread. Raises JobRunning while another job runs, ValueError for a
    kind that is not one, SystemExit for an engagement not ready for it."""
    if kind not in KINDS:
        raise ValueError(f"no job kind '{kind}' (have: {', '.join(KINDS)})")
    if state.running_job(eng_dir):
        raise JobRunning(state.running_job(eng_dir)["id"])
    ts = state.stamp()
    problem = _precheck(eng_dir, kind, model, ts)
    if problem:
        raise SystemExit(problem)
    (eng_dir / state.JOBS).mkdir(exist_ok=True)
    try:
        record = state.add_job(eng_dir, kind, by=by)
    except SystemExit as e:
        raise JobRunning(str(e))
    log = eng_dir / state.JOBS / f"{record['id']}.log"
    state.update_job(eng_dir, record["id"], log=str(log))
    state.set_stage(eng_dir, "enumeration" if kind == "enumerate" else "chain", "running", by)
    t = threading.Thread(target=_run, args=(eng_dir, kind, record["id"], model, ts, log),
                         name=f"job-{eng_dir.name}-{record['id']}", daemon=True)
    t.start()
    logger.info("job %s (%s) started for %s", record["id"], kind, eng_dir.name)
    return Job(eng_dir, record, t)


def _run(eng_dir: Path, kind: str, job_id: str, model: str, ts: str, log: Path) -> None:
    stage_name = "enumeration" if kind == "enumerate" else "chain"
    steps_done: List[Dict[str, Any]] = []
    error: Optional[str] = None
    code = 0
    with open(log, "a", encoding="utf-8") as out:
        try:
            for label, argv in _iter_steps(eng_dir, kind, model, ts):
                out.write(f"\n=== {label} — {state.stamp()}\n$ {' '.join(argv)}\n")
                out.flush()
                proc = subprocess.Popen(argv, cwd=str(REPO), stdout=out,
                                        stderr=subprocess.STDOUT)
                steps_done.append({"label": label, "pid": proc.pid, "exit": None})
                state.update_job(eng_dir, job_id, steps=steps_done)
                proc.wait()
                steps_done[-1]["exit"] = proc.returncode
                state.update_job(eng_dir, job_id, steps=steps_done)
                if proc.returncode != 0:
                    code, error = proc.returncode, f"{label} exited {proc.returncode}"
                    break
        except Exception as e:                                 # noqa: BLE001
            code, error = 1, f"{type(e).__name__}: {e}"
            logger.exception("job %s failed", job_id)
        out.write(f"\n=== {'failed: ' + error if error else 'done'} — {state.stamp()}\n")
    state.finish_job(eng_dir, job_id, code, error)
    state.set_stage(eng_dir, stage_name, "failed" if error else "done", "job")
    link = f"{mail.site_url()}/p/#{eng_dir.name}"
    if error:
        mail.send(mail.practice_emails(), f"Tuuyi: {kind} failed for {eng_dir.name}",
                  f"The {kind} job for {eng_dir.name} failed: {error}.\n"
                  f"The log is {log}.", link)
    else:
        mail.send(mail.practice_emails(), f"Tuuyi: {kind} finished for {eng_dir.name}",
                  f"The {kind} job for {eng_dir.name} finished. "
                  + ("The claim surface is ready to review and freeze."
                     if kind == "enumerate" else
                     "The report is rendered and waits for your sign-off before release."),
                  link)
    logger.info("job %s (%s) %s", job_id, kind, "failed: " + error if error else "done")
