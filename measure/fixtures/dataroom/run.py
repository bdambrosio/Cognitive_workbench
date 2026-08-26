#!/usr/bin/env python3
"""Drive one dataroom audit run, unattended.

    python3 measure/fixtures/dataroom/run.py --world dataroom_1
    python3 measure/fixtures/dataroom/run.py --world dataroom_2 \
            --model measure/models/nemotron_fp8.yaml --max-turns 20

Loads `scenarios/audit.yaml`, points `inspect_external` at the fixture
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

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

import yaml                                                    # noqa: E402

SCENARIO = REPO / "scenarios" / "audit.yaml"
CORPUS = HERE / "corpus"
SOURCE = "User"

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("dataroom.run")
logger.setLevel(logging.INFO)

GAP_MARK = "=== GAP MAP ==="
# Whitespace-tolerant, for the same reason score.py's is: a marker split by a
# line wrap is still a marker. Only used for the salvage path now.
_GAP_RE = re.compile(r"\s+".join(re.escape(t) for t in GAP_MARK.split()))

# THE BRIEF IS THE ENGAGEMENT, NOT THE METHOD. It used to open "Read
# METHOD.md" and then restate the deliverable contract in full, because the
# method was something the agent fetched with a tool and the contract had
# nowhere else to live. Both are now in the workflow document, loaded verbatim
# into the static system prompt (scenarios/audit.yaml `workflow:`), so what is
# left here is only what a client would actually say: who the target is, what
# is wanted, and the protocol for driving legs.
#
# Dropped with the contract: the paragraph telling the agent to quote figures
# verbatim. That is §5, and §5 is now guaranteed present on every turn — which
# is precisely what it was not when that paragraph was added.
#
# Dropped 2026-08-24: "Recap as the method says. Stop and tell me if you find a
# delta." Both named procedures METHOD.md has since removed as unperformable —
# there is no channel to the client mid-engagement — and §12 step 4 now says
# the opposite. A client brief instructing behaviour the method forbids is a
# contradiction the agent has to resolve at runtime, and it was doing so
# silently. The brief must not outlive the method it refers to.
#
# Added 2026-08-25: the claim sources. §2 defines a claim as something the
# SELLER asserts to the buyer, which makes doc3-doc8 evidence rather than
# claims — but deciding that per run is a judgement, and three models made it
# three ways. Measured: counts of 62, 67 and 273 on identical materials, then
# 44, 21 and 321 under one definition and 66, 108 and none under another. The
# engagement names the sources instead; §12 step 1 says so. This is the client
# speaking, not the method, which is why it lives in the brief.
BRIEF = """The target is the data room bound to inspect_external: nine
documents for a small SaaS business called flowmetrics, offered for sale by a
seller named Dave. I am the buyer's side. Audit it.

The claim sources are doc1 (the listing), doc2 (the tech stack description)
and doc9 (the technical claims). Those are the documents Dave asserts things
in. The other six are evidence.

Enumerate the claim surface first and close it as the method says, then work
the priority order straight through.

Work in as many legs as you need — end a leg with `yield` and I will say
continue. End with the report; I will ask for the Gap Map after it.
"""

CONTINUE = "continue"

# THE SECOND DELIVERABLE IS ASKED FOR, NOT DETECTED. §16 puts the report in one
# turn and the Gap Map in the next, so the runner knows which is which because
# it drove them. Nothing has to be parsed out of prose.
# The leading sentence is a STABLE SENTINEL score.py matches to find the Gap
# Map leg. Matching it is safe in a way matching the model's output is not:
# this is the harness's own text, emitted verbatim by the harness. Change it
# and score.py's GAP_MAP_REQUEST_MARK must change with it.
GAP_MAP_REQUEST = ("The report is received. Now the Gap Map, per §15. The whole "
                   "reply is the Gap Map — no preamble, no restatement of the "
                   "report.")


def engagement_state(world: str, agent: str, leg: int, max_legs: int,
                     elapsed_s: float, fixture: bool = True) -> str:
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
    docs = sorted(f.name for f in CORPUS.glob("*.md")) if CORPUS.is_dir() else []
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
    if fixture:
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
    cfg["external_repo"] = str(external_repo or CORPUS)
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
    ap.add_argument("--world", required=True,
                    help="fresh world name; never reuse one")
    ap.add_argument("--model", type=Path, default=None,
                    help="YAML with an llm_config block; replaces the scenario's")
    ap.add_argument("--max-turns", type=int, default=25,
                    help="hard cap on legs (default 25)")
    ap.add_argument("--external-repo", type=Path, default=None,
                    help="audit a real target instead of the fixture corpus. "
                         "Scoring against the answer key is meaningless for "
                         "one — score.py is for the fixture")
    ap.add_argument("--brief-file", type=Path, default=None,
                    help="engagement brief for a real target; required with "
                         "--external-repo, since the built-in brief names "
                         "flowmetrics")
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

    ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    out = HERE / "results" / f"{ts}_{args.world}"
    out.mkdir(parents=True, exist_ok=True)

    arm_doc = (yaml.safe_load(Path(args.model).read_text(encoding="utf-8"))
               if args.model else {})
    served_check = verify_served_model(arm_doc)
    if served_check.get("checked"):
        logger.info("served-model check OK: %s", served_check["served"])

    wf_mode = (None if args.workflow_mode is None
               else args.workflow_mode == "on")
    if args.external_repo and not args.brief_file:
        raise SystemExit(
            "--external-repo needs --brief-file: the built-in brief "
            "names flowmetrics and its nine documents, and handing "
            "that to an agent pointed at a different target is the "
            "kind of mismatch that produces a confident report about "
            "the wrong thing.")
    name, cfg = build_config(args.world, args.model, wf_mode,
                             args.temperature, args.max_tokens,
                             args.external_repo)
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
    gap_map_sent = False
    report_text = ''
    try:
        text = (args.brief_file.read_text(encoding='utf-8')
                if args.brief_file else BRIEF)
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
            # THE RUNNER DRIVES; THE SCORER JUDGES. This loop used to continue
            # until `=== GAP MAP ===` appeared in a reply, because "done is a
            # deliverable, not an exit reason": on 2026-08-22 an model wrote "I
            # will now begin working the priority order", ended the turn, and
            # was scored as complete. But detecting completeness by matching a
            # string in prose is what that check actually was, and score.py
            # already has the criteria — a 241-word report with zero findings
            # fails the finding parse, fails Tier 1, fails the threshold. So a
            # premature `respond` now produces a legibly bad run instead of a
            # silent pass, and this loop stops making the judgement.
            if gap_map_sent:
                break                       # that reply IS the Gap Map (§16)
            if exit_reason == "yield":
                text = CONTINUE + engagement_state(
                    args.world, name, i + 2, args.max_turns,
                    time.time() - t0, fixture=args.external_repo is None)
                continue
            # SALVAGE. An model may emit both documents in one turn whatever the
            # brief says. Splitting that is strictly better than spending a leg
            # asking for a Gap Map it has already written.
            if _GAP_RE.search(reply):
                logger.info("leg %d carried both documents — no Gap Map leg needed",
                            i + 1)
                break
            report_text = reply
            text, gap_map_sent = GAP_MAP_REQUEST, True
            continue
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
    # TWO TURNS, TWO FILES (§16). The runner asked for each, so it knows which
    # reply is which and nothing is parsed out of prose. The marker path
    # survives only as salvage, for an model that emitted both in one turn.
    final = latest_reply(loop, SOURCE)
    if final:
        (out / "full_reply.md").write_text(final, encoding="utf-8")
        hit = _GAP_RE.search(final)
        if gap_map_sent:
            # The last reply is the Gap Map; the one before it is the report.
            (out / "report.md").write_text(
                (report_text or "").rstrip() + "\n", encoding="utf-8")
            (out / "gap_map.md").write_text(final.strip() + "\n", encoding="utf-8")
            if not report_text:
                logger.warning("Gap Map leg ran but no report leg was captured")
        elif hit:
            (out / "report.md").write_text(
                final[:hit.start()].rstrip() + "\n", encoding="utf-8")
            (out / "gap_map.md").write_text(
                final[hit.end():].strip() + "\n", encoding="utf-8")
        else:
            (out / "report.md").write_text(final, encoding="utf-8")
            logger.warning("run ended without a Gap Map leg or marker — "
                           "Gap Map not produced")

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
    # THE DELIVERED TEXT, NOT THE FILE. audit/METHOD.md is not what the agent
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
        "wall_clock_s": wall,
        "error": error,
        "captured_at_utc": ts,
    }, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\n{len(legs)} legs, {wall}s, error={error}")
    print(f"deliverables: {out}/report.md, gap_map.md")
    print(f"meta: {out / 'run_meta.json'}")
    print(f"score with:  python3 measure/fixtures/dataroom/score.py "
          f"--world {args.world}")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
