#!/usr/bin/env python3
"""Drive one dataroom audit run, unattended.

    python3 workflows/claims_audit/runner.py --world dataroom_1
    python3 workflows/claims_audit/runner.py --world dataroom_2 \
            --model measure/models/nemotron_fp8.yaml --max-turns 20

Loads `workflows/claims_audit/scenario.yaml`, points `inspect_external` at the fixture
corpus, gives the brief as the opening turn, then drives "continue" for as
long as the agent keeps yielding. Writes the reply and the trace next to the
run so `score.py` can read them.

WHY A RUNNER AND NOT --cli. The interactive path needs a human to paste the
brief and press return between legs. A benchmark needs the same input every
time, which a human cannot supply; three runs typed by hand are three
different runs.

WHY autonomy IS OFF. Continuations are driven here rather than by the tick
sensor, so a run is deterministic in its leg count and a stall is visible as
a stall rather than as a missing tick. Concern CREATION happens in-turn and
needs no autonomy; only FIRING is gated.

WHY THE MODEL REPLACES llm_config RATHER THAN MERGING. A merge lets a stale
field from the scenario — an api_key, a reasoning_effort — survive into an
model that never declared one, which is the silent second variable this exists
to avoid. Shape and rationale recovered from the retired bench/common.py.
"""

from __future__ import annotations

import argparse
import datetime
import json
import logging
import re
import shutil
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

HERE = Path(__file__).resolve().parent          # workflows/claims_audit
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

import yaml                                                    # noqa: E402

SCENARIO = HERE / "scenario.yaml"
ENGAGEMENTS = HERE / "engagements"
SOURCE = "User"

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("dataroom.run")
logger.setLevel(logging.INFO)

# DELIVERY IS BY BLOCK (METHOD §16), not by turn boundary. `workflows/blocks.py`
# holds the vocabulary and the reason; this file only drives it.
from workflows import blocks                                   # noqa: E402

# The brief lives with its engagement, in engagements/<name>/brief.md — it is
# what a client says, and it differs per engagement. Its history travels with
# it as HTML comments in that file.
CONTINUE = "continue"

# NO DELIVERABLE IS ASKED FOR ANY MORE. Until 2026-08-27 the runner drove the
# report and the Gap Map as one turn each and knew which was which because it
# had asked — which is exactly how a claim enumeration became a report. Blocks
# announce themselves (METHOD §16), so the runner asks for nothing while they
# arrive and speaks only to name one that has not.


def load_engagement(name: str) -> Dict[str, Any]:
    """Target, brief and run directory for one engagement.

    THE ENGAGEMENT SUPPLIES WHAT THE METHOD MUST NOT INVENT: which tree is
    under audit, and — through the brief — which of its documents carry the
    seller's claims (METHOD §12 step 1). Both differ per engagement and neither
    is knowable to the method's author.
    """
    d = ENGAGEMENTS / name
    cfg_file = d / "engagement.yaml"
    if not cfg_file.is_file():
        have = ", ".join(sorted(x.name for x in ENGAGEMENTS.iterdir()
                                if x.is_dir())) or "none"
        raise SystemExit(f"no engagement '{name}' in {ENGAGEMENTS} (have: {have})")
    cfg = yaml.safe_load(cfg_file.read_text(encoding="utf-8")) or {}
    target = Path(cfg.get("target") or "")
    if not target.is_absolute():
        target = REPO / target
    brief = d / "brief.md"
    if not brief.is_file():
        raise SystemExit(f"engagement '{name}' has no brief.md")
    return {"name": name, "dir": d, "target": target.resolve(),
            "brief": brief, "runs": d / "runs",
            "retention": cfg.get("retention")}


def engagement_state(world: str, agent: str, leg: int, max_legs: int,
                     elapsed_s: float, corpus: Optional[Path] = None) -> str:
    """A ledger of what has happened, appended to each `continue`.

    WHY THE RUNNER CARRIES THIS. The agent's own record of its work decays by
    design — observations cap at 1000 chars once stored, and whole legs drop
    out of the history window. The runner's does not. So the one thing an
    agent structurally cannot hold across a long engagement is exactly the
    thing the process driving it already knows for free.

    STATE, NOT GAPS — the line worth holding. Everything here is something
    that happened: legs taken, documents opened, minutes spent. Nothing here
    says what remains. The runner uses the Gap Map marker as its stopping
    rule, and handing the agent the stopping rule would be feeding the metric
    to the thing being measured. "9 of 9 documents opened" is a ledger entry;
    "you still owe me a Gap Map" is the answer sheet.

    Nothing here is interpreted. Filenames are matched against the corpus
    listing — a directory comparison, not a judgement about what the agent
    meant or intended.
    """
    # Fixture-only. A real target has a claim surface, not a document list,
    # and counting *.md in a 1,100-file repository would report progress
    # against a denominator that means nothing.
    docs = sorted(f.name for f in corpus.glob("*.md")) \
        if corpus and corpus.is_dir() else []
    traces = REPO / "scenarios" / world / agent / "inspect_traces"
    seen = set()
    if traces.is_dir():
        for t in traces.glob("inspect_external_*.txt"):
            try:
                body = t.read_text(errors="replace")
            except OSError as e:
                logger.warning("ledger: unreadable trace %s (%s)", t, e)
                continue
            seen.update(d for d in docs if d in body)
    head = (f"\n\n[engagement state, recorded by the client's process — "
            f"leg {leg} of {max_legs}, {elapsed_s / 60:.0f} min elapsed")
    if docs:
        return head + (f". Data room: {len(seen)} of {len(docs)} documents "
                       f"opened so far.]")
    return head + ".]"


def verify_served_model(model: Dict[str, Any], timeout: float = 10.0
                        ) -> Dict[str, Any]:
    """Ask the server what it is serving, and refuse to run if it is not the
    model we think we are measuring.

    Recovered from the retired bench/common.py on 2026-08-22, the day it was
    needed: both local models declare model:"" (whatever is served), so when the
    box switched from Qwen to Gemma the qwen model would have produced a Gemma
    run labelled Qwen. A comparison whose backend identity rests on
    recollection is not a comparison. The result is recorded verbatim into
    run_meta.json so a row can always name its own backend.
    """
    import urllib.request, urllib.error
    expect = model.get("expects_served_model")
    if not expect:
        return {"checked": False, "reason": "cloud model — nothing to interrogate",
                "served": (model.get("llm_config") or {}).get("model")}
    base = ((model.get("llm_config") or {}).get("vllm_url") or "").rstrip("/")
    if base.endswith("/v1"):
        base = base[:-3]
    url = f"{base}/v1/models"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as fh:
            payload = json.loads(fh.read().decode("utf-8"))
    except (urllib.error.URLError, OSError, ValueError) as e:
        raise SystemExit(
            f"cannot reach {url} to verify the served model ({e}). The local "
            f"backend must be up before a run — a connection refused mid-run "
            f"reads as a mechanism failure in the trace and is not one.")
    served = [m.get("id", "") for m in (payload.get("data") or [])]
    if not any(expect.lower() in s.lower() for s in served):
        raise SystemExit(
            f"model expects a model matching {expect!r} but {url} is serving "
            f"{served}. Restart on the right model, or this row is mislabelled.")
    return {"checked": True, "endpoint": url, "served": served,
            "expected": expect}


def git_rev(path: Path) -> Optional[str]:
    """The commit a tree is at, or None if it is not a repository.

    Used for both the target and this repo. A target that moves under a
    delivered report makes every citation in it resolve against a tree the
    report never saw, and nothing downstream can detect that from a path.
    """
    import subprocess
    try:
        r = subprocess.run(["git", "-C", str(path), "rev-parse", "HEAD"],
                           capture_output=True, text=True, timeout=10)
        return r.stdout.strip() or None if r.returncode == 0 else None
    except Exception as e:                                     # noqa: BLE001
        logger.warning("git rev for %s: %s", path, e)
        return None


def files_read(traces: Path, target: Path) -> Dict[str, str]:
    """Content hashes for the target files this run actually opened.

    A commit pin says the tree moved. This says whether it moved under a file
    the report cites, which is the question a reader of the report has.

    Derived rather than authored: every evidence request already records the
    files it opened, so this reads the record the run produced instead of
    asking the auditor to declare anything.
    """
    import hashlib
    import re as _re
    out: Dict[str, str] = {}
    if not traces.is_dir() or not target.is_dir():
        return out
    names = set()
    for t in sorted(traces.glob("*.txt")):
        try:
            names.update(_re.findall(r'"file":\s*"([^"]+)"', t.read_text(errors="replace")))
        except OSError as e:
            logger.warning("manifest: unreadable trace %s (%s)", t, e)
    for n in sorted(names):
        f = target / n
        if f.is_file():
            try:
                out[n] = hashlib.sha256(f.read_bytes()).hexdigest()[:16]
            except OSError as e:
                logger.warning("manifest: unreadable target file %s (%s)", f, e)
    return out


def build_config(world: str, model_path: Optional[Path],
                 workflow_mode: Optional[bool] = None,
                 temperature: Optional[float] = None,
                 max_tokens: Optional[int] = None,
                 external_repo: Optional[Path] = None
                 ) -> Tuple[str, Dict[str, Any]]:
    from launcher import parse_characters                      # noqa: E402

    scenario = yaml.safe_load(SCENARIO.read_text(encoding="utf-8")) or {}
    scen_llm = dict(scenario.get("llm_config") or {})

    model_llm = None
    if model_path:
        model = yaml.safe_load(Path(model_path).read_text(encoding="utf-8")) or {}
        model_llm = dict(model.get("llm_config") or {})
        if not model_llm:
            raise SystemExit(f"{model_path}: no llm_config block")
        for char in (scenario.get("characters") or {}).values():
            if isinstance(char, dict) and char.get("mode") == "chat":
                char["llm_config"] = dict(model_llm)   # REPLACE, never merge
        scen_llm.update(model_llm)

    world_cfg = dict(scenario.get("world_config") or {})
    world_cfg["world_name"] = world

    chars = parse_characters(scenario, scen_llm, world_cfg,
                             scenario.get("setting", ""),
                             scenario.get("alt_llm_config") or {})
    chat_chars = [(n, c) for n, c in chars if c.get("mode") == "chat"]
    if len(chat_chars) != 1:
        raise SystemExit(f"expected 1 chat character, found {len(chat_chars)}")
    name, cfg = chat_chars[0]
    cfg["autonomy_enabled"] = False
    # The two per-target values, set here rather than by editing the committed
    # scenario, so a run leaves no diff behind.
    cfg["external_repo"] = str(external_repo)
    # WHY A FLAG AND NOT A SECOND SCENARIO. Comparing workflow_mode on
    # against off needs both, and audit.yaml's own header says why a copy is
    # the wrong way to get one: "a second copy is a second source that
    # drifts". One scenario, one override, and run_meta records which side of
    # the comparison a row is on — a row that cannot name its own
    # configuration is not evidence, the same rule the served-model check
    # exists to enforce.
    if workflow_mode is not None:
        cfg["workflow_mode"] = workflow_mode
    if temperature is not None:
        cfg.setdefault("chat", {})["react_temperature"] = temperature
    # Reasoning tokens bill against the SAME budget as the action. At 8192 a
    # thinking model can deliberate until there is no room left to emit the
    # JSON, and the emission comes back truncated or empty — observed on the
    # grader 2026-08-23, silently, three times in eleven.
    if max_tokens is not None:
        cfg.setdefault("chat", {})["react_max_tokens"] = max_tokens
    return name, cfg


def last_exit_reason(world: str, agent: str) -> Optional[str]:
    """Read it off the trace, not the loop. `exit_reason` is a local inside
    _process_user_turn and is never stored on the object — getattr for it
    returns None, which reads as "not a yield" and would end every run after
    one leg."""
    p = (REPO / "scenarios" / world / agent / "memory" /
         "reasoning_trace.jsonl")
    if not p.exists():
        return None
    last = None
    for line in p.open(errors="replace"):
        line = line.strip()
        if line:
            last = line
    if not last:
        return None
    try:
        return json.loads(last).get("exit_reason")
    except json.JSONDecodeError:
        return None


def latest_reply(loop, source: str) -> str:
    turns = loop.store.get_recent_turns(source, limit=4, scope="all")
    for t in reversed(turns):
        if str(t.get("name", "")) != source:
            return str(t.get("text", ""))
    return ""


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engagement", required=True,
                    help="engagement name under engagements/ — supplies the "
                         "target, the brief, and where runs land")
    ap.add_argument("--world", required=True,
                    help="fresh world name; never reuse one")
    ap.add_argument("--model", type=Path, default=None,
                    help="YAML with an llm_config block; replaces the scenario's")
    ap.add_argument("--max-turns", type=int, default=25,
                    help="hard cap on legs (default 25)")
    ap.add_argument("--max-tokens", type=int, default=None,
                    help="override chat.react_max_tokens (scenario default "
                         "8192). Reasoning tokens bill against this budget")
    ap.add_argument("--temperature", type=float, default=None,
                    help="override the action-emission temperature "
                         "(scenario default 0.7)")
    ap.add_argument("--workflow-mode", choices=("on", "off"), default=None,
                    help="override the scenario's workflow_mode; omit to use "
                         "whatever audit.yaml declares")
    args = ap.parse_args()

    if (REPO / "scenarios" / args.world).exists():
        raise SystemExit(
            f"world '{args.world}' already exists — a second run in one world "
            f"inherits the first's conclusions as memories. Pick a fresh name.")

    eng = load_engagement(args.engagement)
    ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    # RUNS BELONG TO THE ENGAGEMENT, not to the workflow and not to a fixture.
    out = eng["runs"] / f"{ts}_{args.world}"
    out.mkdir(parents=True, exist_ok=True)

    arm_doc = (yaml.safe_load(Path(args.model).read_text(encoding="utf-8"))
               if args.model else {})
    served_check = verify_served_model(arm_doc)
    if served_check.get("checked"):
        logger.info("served-model check OK: %s", served_check["served"])

    wf_mode = (None if args.workflow_mode is None
               else args.workflow_mode == "on")
    if not eng["target"].is_dir():
        raise SystemExit(f"engagement '{eng['name']}': target "
                         f"{eng['target']} does not exist")
    name, cfg = build_config(args.world, args.model, wf_mode,
                             args.temperature, args.max_tokens,
                             eng["target"])
    logger.info("world=%s model=%s model=%s", args.world, args.model,
                (cfg.get("llm_config") or {}).get("model") or "(scenario default)")

    from chat.chat_loop import ChatLoop                        # noqa: E402
    from chat.model_params import TOP_P                        # noqa: E402
    loop = ChatLoop(character_name=name, character_config=cfg)

    # PRE-FLIGHT: resolve the model and its temperature before any work. An
    # unconfigured model raises here, in one second, instead of producing a
    # full run that has to be thrown away.
    resolved_model = loop.backend.resolved_model()
    resolved_temperature = (cfg.get("chat") or {}).get("react_temperature")
    if resolved_temperature is None:
        resolved_temperature = loop.backend.temperature_for_model()
    logger.info("resolved model=%s temperature=%s top_p=%s",
                resolved_model, resolved_temperature, TOP_P)

    t0 = time.time()
    legs, error = [], None
    text_first_leg = ''
    # Blocks seen anywhere across the engagement, and how many times the runner
    # had to say one was missing. `blocks_prompted` is the metric that lets the
    # gates exist at all: gating every block means a model that forgets to
    # enumerate is told to, which would have turned campaign m1's three
    # claim-surface failures into successes and flattened the only criterion
    # that discriminated. Counting the prompts keeps the signal and makes it
    # finer than the pass/fail it replaces — "claim surface: 1 prompt" says
    # more than "criterion absent".
    delivered = {n: False for n in blocks.BLOCKS}
    prompted = {n: 0 for n in blocks.BLOCKS}
    transcript = []
    undelivered = list(blocks.BLOCKS)
    try:
        text = eng["brief"].read_text(encoding='utf-8')
        text_first_leg = text
        for i in range(args.max_turns):
            loop._process_user_turn(source=SOURCE, text=text, close=False)
            reply = latest_reply(loop, SOURCE)
            exit_reason = last_exit_reason(args.world, name)
            legs.append({"leg": i + 1, "sent": text[:80],
                         "exit_reason": exit_reason,
                         "reply_chars": len(reply)})
            logger.info("leg %d: exit=%s chars=%d", i + 1, exit_reason,
                        len(reply))
            # A turn that died is not a turn that finished. llm_error and
            # max_iters both end the loop, and treating them as ordinary
            # completion reported error=None on a run whose only output was
            # "[degraded reply — a model call failed mid-loop]". Silence
            # reading as success is the failure mode this whole suite exists
            # to catch; it should not be in the runner.
            if exit_reason in ("llm_error", "crashed"):
                error = f"turn {i + 1} ended {exit_reason} — run is not valid"
                break
            if exit_reason == "max_iters":
                error = (f"turn {i + 1} hit max_iters without answering — "
                         f"run is not valid")
                break

            # DELIVERY IS TESTED, NOT INFERRED. Every block the reply carries
            # counts as delivered, whatever leg it arrived in and however many
            # arrived together.
            transcript.append(reply)
            for n, seen in blocks.present(reply).items():
                delivered[n] = delivered[n] or seen
            legs[-1]["blocks"] = [n for n in blocks.BLOCKS
                                  if blocks.opened(reply, n)]
            undelivered = blocks.missing(delivered)
            if not undelivered:
                break

            # `exit_reason` DECIDES THE MESSAGE, NEVER THE ENDING. This is the
            # one job the yield/respond signal keeps, and it is a continuation
            # job: `yield` means the agent says it is still working, so the
            # runner does not interrupt it to name a block that is legitimately
            # still to come. `respond` means the agent believes it is finished
            # — so if a block is missing, that is the moment to say which, and
            # the moment worth counting. Nothing here ends the engagement; only
            # the blocks or the leg cap do.
            if exit_reason == "yield":
                text = CONTINUE + engagement_state(
                    args.world, name, i + 2, args.max_turns,
                    time.time() - t0, corpus=eng["target"])
                continue
            nxt = undelivered[0]
            prompted[nxt] += 1
            logger.info("leg %d: %s not delivered — prompting (%d)",
                        i + 1, nxt, prompted[nxt])
            text = (CONTINUE + "\n\n" + blocks.rejection(nxt) + "\n"
                    + engagement_state(args.world, name, i + 2, args.max_turns,
                                       time.time() - t0, corpus=eng["target"]))
            continue
        else:
            # THE LEG CAP IS A TERMINAL STATE OF ITS OWN, and it names the
            # block it died on. "Never enumerated" and "produced a report and
            # no Gap Map" are different failures; collapsing them into one
            # outcome loses the only thing that distinguishes them.
            if undelivered:
                error = ("no_deliverable: " + ", ".join(undelivered)
                         + f" not delivered in {args.max_turns} legs")
    except Exception as e:                                     # noqa: BLE001
        error = f"{type(e).__name__}: {e}"
        logger.exception("run failed")
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)
        try:
            loop._persist_to_disk()
        except Exception as e:                                 # noqa: BLE001
            logger.warning("final persist failed: %s", e)

    wall = round(time.time() - t0, 1)

    # Write the deliverables as files. run_meta.json carries the metadata;
    # without this the report exists only inside the trace's raw_response,
    # which is not somewhere a human reads a report from.
    #
    # ASSEMBLED FROM BLOCKS, ACROSS EVERY LEG (§16). Nothing is inferred from
    # which leg a reply arrived in — that inference is what filed a claim
    # enumeration as `b2_glm_2/report.md` on 2026-08-27. A block is taken from
    # wherever it was emitted, and a leg carrying several is not a special
    # case.
    #
    # report.md IS THE REPORT BLOCK PLUS THE LIMITATIONS BLOCK. They are two
    # blocks so that nothing has to nest, and one document because that is what
    # the client receives — ISAE 3000 / AT-C 205 require the limitations to
    # travel with the report, not beside it.
    whole = "\n\n".join(t for t in transcript if t)
    final = latest_reply(loop, SOURCE)
    if final:
        (out / "full_reply.md").write_text(final, encoding="utf-8")
    if whole:
        parts = [b for b in (blocks.span(whole, n)
                             for n in blocks.REPORT_BLOCKS) if b]
        if parts:
            (out / "report.md").write_text("\n\n".join(parts).rstrip() + "\n",
                                           encoding="utf-8")
        else:
            logger.warning("no REPORT block in the engagement — no report.md")
        gap = blocks.content(whole, "GAP MAP")
        if gap:
            (out / "gap_map.md").write_text(gap.strip() + "\n", encoding="utf-8")
        else:
            logger.warning("no GAP MAP block in the engagement — no gap_map.md")

    # THE WORKING RECORD, COPIED OUT (§14). The world is discarded after an
    # engagement; these two are the evidence that the work was systematic, and
    # they are the only evidence of the claims examined that produced no
    # finding. Copied here, beside the deliverables, so they survive the
    # world's deletion.
    #
    # BY THE HARNESS, NOT THE AUDITOR. §12 step 7 deliberately does not ask
    # the agent to produce this: an auditor's own account of its diligence is
    # the weakest evidence of that diligence. The value is that this is a
    # byproduct of doing the work rather than a claim about having done it.
    #
    # HOLDS CLIENT MATERIAL. Verbatim lines from the target's documents are in
    # these traces. Harmless for the synthetic fixture; a retention obligation
    # for a real engagement, which belongs in the engagement letter.
    world_dir = REPO / "scenarios" / args.world / name
    record = out / "working_record"

    # THE INSTRUMENT, COPIED OUT WITH IT. The record shows what the auditor
    # did; these two show what it was told to do. Without them a reader — or a
    # continuation answering questions about this run later — cannot tell
    # which method produced the report, and a method that has moved since
    # supplies the wrong verdict vocabulary silently.
    #
    # THE DELIVERED TEXT, NOT THE FILE. workflows/claims_audit/method/METHOD.md is not what the agent
    # read: the loader strips every section marked for the practice. A commit
    # hash would name the file and still not answer what reached the model, and
    # it needs the repository at that revision to resolve at all. A copy is
    # self-contained and is the artifact itself.
    record.mkdir(parents=True, exist_ok=True)
    try:
        from chat.workflow import load_workflow                # noqa: E402
        wf = (cfg.get("workflow") or "").strip()
        if wf:
            (record / "method_as_delivered.md").write_text(
                load_workflow(REPO / wf), encoding="utf-8")
    except Exception as e:                                     # noqa: BLE001
        logger.warning("working record: method not copied (%s)", e)
    try:
        (record / "brief.md").write_text(text_first_leg, encoding="utf-8")
    except Exception as e:                                     # noqa: BLE001
        logger.warning("working record: brief not copied (%s)", e)

    # THE SCENARIO AS RESOLVED, not the file. The model config replaces a block
    # at runtime, and what shaped the run is the result. omitted_tools, the
    # geofence and workflow_mode all decide what the auditor could do, and
    # "why did you not check X" may have the answer "that tool was not bound".
    try:
        (record / "scenario_as_used.json").write_text(
            json.dumps(cfg, indent=2, default=str), encoding="utf-8")
    except Exception as e:                                     # noqa: BLE001
        logger.warning("working record: scenario not copied (%s)", e)
    for src in (world_dir / "memory" / "reasoning_trace.jsonl",
                world_dir / "inspect_traces"):
        if not src.exists():
            logger.warning("working record: %s absent", src.name)
            continue
        try:
            dst = record / src.name
            record.mkdir(parents=True, exist_ok=True)
            if src.is_dir():
                shutil.copytree(src, dst, dirs_exist_ok=True)
            else:
                shutil.copy2(src, dst)
        except OSError as e:
            logger.warning("working record: could not copy %s (%s)", src, e)
    if record.exists():
        kb = sum(f.stat().st_size for f in record.rglob("*") if f.is_file()) // 1024
        logger.info("working record: %d KB copied to %s", kb, record)

    (out / "run_meta.json").write_text(json.dumps({
        "world": args.world,
        "model_config": str(args.model) if args.model else "(scenario default)",
        "workflow_mode": bool(cfg.get("workflow_mode")),
        "react_temperature": (cfg.get("chat") or {}).get(
            "react_temperature", 0.7),
        "react_max_tokens": (cfg.get("chat") or {}).get(
            "react_max_tokens", 8192),
        # THE SETTINGS THAT ACTUALLY APPLIED, resolved from the model rather
        # than copied from config. A row that cannot name its own sampling
        # settings is not evidence — the rule that already governs the served
        # -model check, extended to the thing that cost a campaign.
        "resolved_model": resolved_model,
        "resolved_temperature": resolved_temperature,
        "top_p": TOP_P,
        "llm_config": cfg.get("llm_config"),
        "served_model_check": served_check,
        "external_repo": cfg.get("external_repo"),
        # A PATH IS NOT A PIN. run_meta used to record where the target was and
        # not what it contained, so a citation could later resolve against a
        # tree the report never saw, silently. The commit says the tree moved;
        # the manifest below says whether it moved under a file this run read.
        "target_rev": git_rev(Path(cfg.get("external_repo") or ".")),
        "harness_rev": git_rev(REPO),
        "files_read": files_read(record / "inspect_traces",
                                 Path(cfg.get("external_repo") or ".")),
        "legs": legs,
        # HOW MUCH THE HARNESS HAD TO CARRY. Gating every block means a model
        # that forgets to enumerate is told to, and campaign m1's three
        # claim-surface failures would have become successes — the only
        # criterion that discriminated, spent. This is what keeps the signal,
        # and it is finer than the pass/fail it replaces: "claim surface: 1
        # prompt" says more than "criterion absent". A model that emits
        # everything unprompted is zero across the board.
        "blocks_prompted": prompted,
        "blocks_delivered": delivered,
        "blocks_closed": {n: blocks.closed(whole, n) for n in blocks.BLOCKS},
        "wall_clock_s": wall,
        "error": error,
        "captured_at_utc": ts,
    }, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\n{len(legs)} legs, {wall}s, error={error}")
    print("blocks: " + ", ".join(
        f"{n}={'ok' if delivered[n] else 'MISSING'}"
        + (f"(+{prompted[n]} prompt{'s' if prompted[n] > 1 else ''})"
           if prompted[n] else "")
        for n in blocks.BLOCKS))
    print(f"deliverables: {out}/report.md, gap_map.md")
    print(f"meta: {out / 'run_meta.json'}")
    # --run, not --world: the run directory is the archive and survives the
    # world's deletion (§14). Scoring is fixture-only; a real engagement has no
    # answer key.
    print(f"score with:  python3 measure/fixtures/dataroom/score.py "
          f"--run {out}")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
