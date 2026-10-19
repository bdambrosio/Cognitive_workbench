#!/usr/bin/env python3
"""The three jobs the site's buttons start, run as subprocesses of the
existing runners and watched to their end.

    sort        the materials-sorting runner: reads every prose file in the
                target and proposes the claim sources and evidence excludes;
                a person confirms them on the sorting page before enumeration
    enumerate   one claims_audit run per claim source with --enumerate-only,
                then the duplicates pass over all of them, then the reliance
                statement and the tiers; leaves a claims.json per source, with
                its marks, for the surface page
    chain       per claim source: the audit on the frozen surface, then its
                review; then materiality over every run, against the current
                intake; then the report

ONE JOB AT A TIME PER ENGAGEMENT, MANY ENGAGEMENTS AT ONCE. The lock is the
job record in state.json (engagement_state.add_job refuses while one is
running). There is no queue across engagements: Fireworks carries several
chains at once, and the pilot site has at most a handful (Bruce,
2026-09-05).

SOURCES RUN SIDE BY SIDE (Bruce, 2026-10-19). Each claim source's
enumeration, and each source's audit and review, is a lane: its own steps in
order, in its own log, `<job id>_<source>.log`, beside the job's. Up to LANES
lanes run at once, and each runner is given `--parallel PARALLEL`, so a job
has at most LANES x PARALLEL model calls in flight. The steps after the lanes
wait for all of them. These are production jobs; a measurement run is started
by hand with the runners' default of one call at a time.

EACH STEP IS THE RUNNER'S OWN COMMAND LINE, the same one the practice page
shows for copying. A step that exits non-zero ends the job as failed, with
the step named; later steps do not run, lanes not yet started do not start,
and a lane already running finishes the step it is on and stops; the practice is mailed; the client
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
from concurrent.futures import ThreadPoolExecutor
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
#: The duplicates step judges many short pairs; the same model at low
#: reasoning marked no worse by hand review and ran four times faster
#: (2026-09-17: 84 s against 331 s on 304 claims). Bruce, 2026-09-17.
DUPLICATES_MODEL = "measure/models/fw_glm53flash_low.yaml"
KINDS = ("sort", "enumerate", "chain")
#: Claim sources processed at once, and model calls each runner has in
#: flight at once (its --parallel).
LANES = 3
PARALLEL = 4
#: The stage each kind of job marks as running, then done or failed.
STAGE_OF = {"sort": "sorting", "enumerate": "enumeration", "chain": "chain"}

Step = Tuple[str, List[str]]


class Lanes:
    """Sequences of steps that run side by side. Each lane is a name and an
    iterator of steps, run in order in its own subprocesses and log."""

    def __init__(self, lanes: List[Tuple[str, Any]]) -> None:
        self.lanes = lanes


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
        "sort": f"python3 workflowsv2/materials_sorting/runner.py --engagement {name} --model {MODEL}",
        "sort_confirm": f"python3 workflowsv2/materials_sorting/runner.py --engagement {name} --confirm --by <email>",
        "duplicates": f"python3 workflowsv2/claims_audit/duplicates.py --engagement {name} --model {DUPLICATES_MODEL}",
        "reliance": f"python3 workflowsv2/claims_audit/reliance.py --engagement {name} --model {MODEL}",
        "tiers": f"python3 workflowsv2/claims_audit/tiers.py --engagement {name} --model {MODEL} --parallel {PARALLEL}",
        "audit": f"python3 workflowsv2/claims_audit/runner.py --engagement {name} --world <fresh world> --claim-source <one of claim_sources> --model {MODEL} --parallel {PARALLEL}",
        "review": f"python3 workflowsv2/audit_review/runner.py --run {eng}/runs/<run dir> --model {MODEL} --parallel {PARALLEL}",
        "materiality": f"python3 workflowsv2/audit_materiality/runner.py --engagement {name} --run {eng}/runs/<run dir> [--run ...] --model {MODEL} --label <label> --parallel {PARALLEL}",
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


def sort_steps(eng_dir: Path, model: str) -> List[Step]:
    return [("sort the materials", _py(
        "workflowsv2/materials_sorting/runner.py", "--engagement", eng_dir.name,
        "--model", model))]


def enumerate_steps(eng_dir: Path, model: str, ts: str) -> List[Any]:
    steps: List[Any] = []
    # A question source is the practice's own statements, one per line: no
    # model reads it; its draft surface is written straight from the file.
    if any(state.question_kind(eng_dir, s) for s in state.claim_sources(eng_dir)):
        steps.append(("list the question sources' statements", _py(
            "workflowsv2/claims_audit/questions.py", "--engagement", eng_dir.name)))
    lanes = []
    for src in state.claim_sources(eng_dir):
        if state.question_kind(eng_dir, src):
            continue
        world = f"enum_{eng_dir.name}_{slug(src)}_{ts}"
        lanes.append((src, iter([(f"enumerate {src}", _py(
            "workflowsv2/claims_audit/runner.py", "--engagement", eng_dir.name,
            "--world", world, "--claim-source", src, "--enumerate-only",
            "--model", model))])))
    if lanes:
        steps.append(Lanes(lanes))
    # Each source is enumerated alone, so a claim made in two documents is
    # listed twice; this pass marks the repeats before anyone reads the surface.
    steps.append(("mark repeated claims", _py(
        "workflowsv2/claims_audit/duplicates.py", "--engagement", eng_dir.name,
        "--model", DUPLICATES_MODEL)))
    # What the buyer relies on is written once, from the intake and every
    # claim; then each claim is rated against it. A person corrects the
    # statement and any tier on the surface page before the freeze.
    steps.append(("state what the buyer relies on", _py(
        "workflowsv2/claims_audit/reliance.py", "--engagement", eng_dir.name,
        "--model", model)))
    steps.append(("rate the claims into tiers", _py(
        "workflowsv2/claims_audit/tiers.py", "--engagement", eng_dir.name,
        "--model", model, "--parallel", str(PARALLEL))))
    return steps


class Chain:
    """The chain's steps are built as it goes, because each audit's run
    directory is known only after it ran."""

    def __init__(self, eng_dir: Path, model: str, ts: str) -> None:
        self.eng_dir, self.model, self.ts = eng_dir, model, ts
        self.sources = state.claim_sources(eng_dir)
        self.run_dirs: Dict[str, Path] = {}

    def check(self) -> Optional[str]:
        if not self.sources:
            return "engagement.yaml names no claim_sources"
        missing = [s for s in self.sources if not surface_file(self.eng_dir, s).is_file()]
        if missing:
            return "no frozen surface for " + ", ".join(missing)
        if not any(self.tested(s) for s in self.sources):
            return "no frozen claim is in tier 1: there is nothing to test"
        return None

    def tested(self, src: str) -> int:
        """How many claims of this source's frozen surface the audit would
        test: tier 1 and any not rated (TIERS.md §1)."""
        import json
        from workflowsv2.claims_audit.schemas import split_by_tier
        doc = json.loads(surface_file(self.eng_dir, src).read_text(encoding="utf-8"))
        return len(split_by_tier(doc.get("claims") or [])[0])

    def steps(self) -> "Iterator[Step]":                       # noqa: F821
        from workflowsv2.composition import scan as composition
        if composition.enabled(self.eng_dir):
            # An optional, extra-cost part of the engagement: the components
            # the dependency files declare, shown as a report appendix.
            yield ("scan the components", _py(
                "workflowsv2/composition/scan.py", "--engagement", self.eng_dir.name))
        tested = []
        for src in self.sources:
            if not self.tested(src):
                # Every claim of this source was rated tier 2 or 3. An audit
                # of nothing writes no findings and the review of it fails;
                # the report lists these claims from the frozen surface.
                logger.info("%s has no claim to test; no audit and no review", src)
                continue
            tested.append(src)
        yield Lanes([(src, self.source_steps(src)) for src in tested])
        label = f"chain_{self.ts}"
        args = ["--engagement", self.eng_dir.name]
        # In the sources' order, whichever lane finished first.
        for src in tested:
            args += ["--run", str(self.run_dirs[src])]
        intake = state.current_intake(self.eng_dir)
        if intake:
            args += ["--intake", intake]
        yield ("materiality", _py("workflowsv2/audit_materiality/runner.py",
                                  *args, "--model", self.model, "--label", label,
                                  "--parallel", str(PARALLEL)))
        merged = _newest(self.eng_dir / state.MERGED, label)
        if merged is None:
            raise RuntimeError("materiality left no merged directory")
        yield ("report", _py("workflowsv2/audit_report/runner.py",
                             "--merged", str(merged), "--model", self.model))


    def source_steps(self, src: str) -> "Iterator[Step]":       # noqa: F821
        """One source's lane: its audit, then the review of that run."""
        world = f"audit_{self.eng_dir.name}_{slug(src)}_{self.ts}"
        yield (f"audit {src}", _py(
            "workflowsv2/claims_audit/runner.py", "--engagement", self.eng_dir.name,
            "--world", world, "--claim-source", src,
            "--surface", str(surface_file(self.eng_dir, src)), "--model", self.model,
            "--parallel", str(PARALLEL)))
        run_dir = _newest(self.eng_dir / "runs", world)
        if run_dir is None:
            raise RuntimeError(f"the audit of {src} left no run directory")
        self.run_dirs[src] = run_dir
        yield (f"review {src}", _py(
            "workflowsv2/audit_review/runner.py", "--run", str(run_dir),
            "--model", self.model, "--world", f"review_{world}",
            "--parallel", str(PARALLEL)))


# ---- running ----------------------------------------------------------------

#: Test seam: replaced to run something other than the real runners.
_steps_for: Optional[Callable[[Path, str, str, str], Any]] = None


def _iter_steps(eng_dir: Path, kind: str, model: str, ts: str):
    if _steps_for is not None:
        return _steps_for(eng_dir, kind, model, ts)
    if kind == "sort":
        return iter(sort_steps(eng_dir, model))
    if kind == "enumerate":
        return iter(enumerate_steps(eng_dir, model, ts))
    return Chain(eng_dir, model, ts).steps()


def _precheck(eng_dir: Path, kind: str, model: str, ts: str) -> Optional[str]:
    if _steps_for is not None:
        return None
    if kind == "sort":
        return None if state.target_dir(eng_dir).is_dir() else "the engagement has no materials"
    if kind == "enumerate":
        # The claim sources and the evidence excludes are confirmed by a
        # person before anything is enumerated from them.
        if state.stage_value(eng_dir, "sorting") != "confirmed":
            return "the materials sorting is not confirmed"
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
    state.set_stage(eng_dir, STAGE_OF[kind], "running", by)
    t = threading.Thread(target=_run, args=(eng_dir, kind, record["id"], model, ts, log),
                         name=f"job-{eng_dir.name}-{record['id']}", daemon=True)
    t.start()
    logger.info("job %s (%s) started for %s", record["id"], kind, eng_dir.name)
    return Job(eng_dir, record, t)


class _Steps:
    """Runs steps as subprocesses and records them on the job, from the
    job's thread or a lane's; the record and the first failure are shared."""

    def __init__(self, eng_dir: Path, job_id: str) -> None:
        self.eng_dir, self.job_id = eng_dir, job_id
        self.done: List[Dict[str, Any]] = []
        self.code, self.error = 0, None
        self.lock = threading.Lock()

    def fail(self, code: int, error: str) -> None:
        with self.lock:
            if self.error is None:
                self.code, self.error = code, error

    def run(self, label: str, argv: List[str], out) -> bool:
        """One step; False if it failed."""
        out.write(f"\n=== {label} — {state.stamp()}\n$ {' '.join(argv)}\n")
        out.flush()
        proc = subprocess.Popen(argv, cwd=str(REPO), stdout=out,
                                stderr=subprocess.STDOUT)
        with self.lock:
            row = {"label": label, "pid": proc.pid, "exit": None}
            self.done.append(row)
            state.update_job(self.eng_dir, self.job_id, steps=list(self.done))
        proc.wait()
        with self.lock:
            row["exit"] = proc.returncode
            state.update_job(self.eng_dir, self.job_id, steps=list(self.done))
        if proc.returncode != 0:
            self.fail(proc.returncode, f"{label} exited {proc.returncode}")
            return False
        return True

    def lane(self, name: str, steps, log: Path) -> None:
        with open(log, "a", encoding="utf-8") as out:
            try:
                for label, argv in steps:
                    if self.error is not None or not self.run(label, argv, out):
                        break
            except Exception as e:                             # noqa: BLE001
                self.fail(1, f"{name}: {type(e).__name__}: {e}")
                logger.exception("job %s lane %s failed", self.job_id, name)

    def lanes(self, item: Lanes, log: Path, out) -> None:
        out.write(f"\n=== {len(item.lanes)} lane(s), at most {LANES} at once — "
                  f"{state.stamp()}\n")
        logs = []
        for name, _ in item.lanes:
            logs.append(log.with_name(f"{log.stem}_{slug(name)}.log"))
            out.write(f"    {name}: {logs[-1].name}\n")
        out.flush()
        with ThreadPoolExecutor(max_workers=LANES) as pool:
            for (name, steps), lane_log in zip(item.lanes, logs):
                pool.submit(self.lane, name, steps, lane_log)
        out.write(f"=== lanes ended — {state.stamp()}\n")


def _run(eng_dir: Path, kind: str, job_id: str, model: str, ts: str, log: Path) -> None:
    stage_name = STAGE_OF[kind]
    runner = _Steps(eng_dir, job_id)
    with open(log, "a", encoding="utf-8") as out:
        try:
            for item in _iter_steps(eng_dir, kind, model, ts):
                if isinstance(item, Lanes):
                    runner.lanes(item, log, out)
                else:
                    runner.run(item[0], item[1], out)
                if runner.error is not None:
                    break
        except Exception as e:                                 # noqa: BLE001
            runner.fail(1, f"{type(e).__name__}: {e}")
            logger.exception("job %s failed", job_id)
        code, error = runner.code, runner.error
        out.write(f"\n=== {'failed: ' + error if error else 'done'} — {state.stamp()}\n")
    state.finish_job(eng_dir, job_id, code, error)
    # A sorting job that succeeds leaves the runner's own mark, "proposed":
    # the stage is done only when a person confirms.
    if error or kind != "sort":
        state.set_stage(eng_dir, stage_name, "failed" if error else "done", "job")
    link = f"{mail.site_url()}/p/#{eng_dir.name}"
    if error:
        mail.send(mail.practice_emails(), f"Tuuyi: {kind} failed for {eng_dir.name}",
                  f"The {kind} job for {eng_dir.name} failed: {error}.\n"
                  f"The log is {log}.", link)
    else:
        mail.send(mail.practice_emails(), f"Tuuyi: {kind} finished for {eng_dir.name}",
                  f"The {kind} job for {eng_dir.name} finished. "
                  + {"sort": "The proposed claim sources and evidence excludes wait "
                             "for your confirmation on the sorting page.",
                     "enumerate": "The claim surface is ready to review and freeze.",
                     "chain": "The report is rendered and waits for your sign-off "
                              "before release."}[kind],
                  link)
    logger.info("job %s (%s) %s", job_id, kind, "failed: " + error if error else "done")
