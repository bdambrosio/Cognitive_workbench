"""Merge the reviewed runs of one engagement, then rate each finding's
materiality.

    python3 workflowsv2/audit_materiality/runner.py --engagement dataroom-fixture \\
        --run <doc1 run> --run <doc2 run> --run <doc9 run> \\
        --model measure/models/or_glm53flash.yaml [--batch 20] [--label x]

Two parts, in order:

  1. MERGE (merge.py). Mechanical: the named runs' findings, each with its
     review outcome and the citation problems the output check recorded,
     concatenated. No deduplication — the claim surface is human-owned.
  2. RATE. Schema-constrained calls over batches of findings, per
     MATERIALITY.md: `materiality` for findings with a verdict about the
     claim, `exposure` for `unverifiable` ones, never in one total. No tools,
     no legs: the finding and the transaction are the whole input.

Output, under engagements/<name>/merged/<ts>_<label>/: merged.json,
materiality.json, materiality.md (for a person: decisive first), meta.json,
issues.jsonl.

WHY THE ENGAGEMENT AND NOT ONLY THE RUNS. The transaction — buyer, purpose,
price basis, structure — is an engagement fact (`transaction:` in
engagement.yaml) and the scale in MATERIALITY.md §3 is relative to it.
"""
from __future__ import annotations

import argparse
import datetime
import json
import logging
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

import yaml                                                    # noqa: E402

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("audit_materiality.run")
logger.setLevel(logging.INFO)

from chat.workflow import load_workflow                        # noqa: E402
from workflowsv2 import issues                                 # noqa: E402
from workflowsv2.audit_materiality import merge as merging     # noqa: E402
from workflowsv2.audit_materiality import schemas              # noqa: E402
from workflowsv2.audit_report.render import _from_knowledge     # noqa: E402
# The review's rendering of a finding, so the rater reads the same text the
# reviewer did. The review runner configures logging at import without
# force=True, which is a no-op after the basicConfig above.
from workflowsv2.audit_review.runner import _finding_text      # noqa: E402
from workflowsv2.claims_audit.runner import load_engagement    # noqa: E402
from workflowsv2.emit import emit, run_tasks, set_parallel     # noqa: E402

SCENARIO = HERE / "scenario.yaml"
METHOD_PATH = "workflowsv2/audit_materiality/method/MATERIALITY.md"
STAGE = "audit_materiality"


def build_config(out: Path, world: str, model_path: Optional[Path]
                 ) -> Tuple[str, Dict[str, Any]]:
    """Same shape and the same reasons as the review runner's: the model config
    REPLACES `llm_config` rather than merging into it."""
    from launcher import parse_characters                      # noqa: E402
    scenario = yaml.safe_load(SCENARIO.read_text(encoding="utf-8")) or {}
    scen_llm = dict(scenario.get("llm_config") or {})
    if model_path:
        doc = yaml.safe_load(Path(model_path).read_text(encoding="utf-8")) or {}
        llm = dict(doc.get("llm_config") or {})
        if not llm:
            raise SystemExit(f"{model_path}: no llm_config block")
        for ch in (scenario.get("characters") or {}).values():
            if isinstance(ch, dict) and ch.get("mode") == "chat":
                ch["llm_config"] = dict(llm)
        scen_llm.update(llm)
    world_cfg = dict(scenario.get("world_config") or {})
    world_cfg["world_name"] = world
    chars = parse_characters(scenario, scen_llm, world_cfg,
                             scenario.get("setting", ""),
                             scenario.get("alt_llm_config") or {})
    chat = [(n, c) for n, c in chars if c.get("mode") == "chat"]
    if len(chat) != 1:
        raise SystemExit(f"expected 1 chat character, found {len(chat)}")
    name, cfg = chat[0]
    cfg["autonomy_enabled"] = False
    cfg["external_repo"] = str(out)
    cfg["inspect_repo"] = str(out)
    return name, cfg


def _rating_text(f: Dict[str, Any]) -> str:
    """One merged finding as the rater reads it: the review's rendering, then
    the review outcome and any citation problem, per MATERIALITY.md §2."""
    claim = {f["claim_id"]: {"quote": f.get("quote"), "lines": f.get("lines"),
                             "statement": f.get("statement")}}
    body = _finding_text({"claim_id": f["claim_id"],
                          "adjudication": f.get("adjudication") or {},
                          "evidence": f.get("evidence") or []}, claim)
    body = body.replace(f"--- claim {f['claim_id']}",
                        f"--- {f['claim_source']} claim {f['claim_id']}", 1)
    body += "\n" + _facts(f)
    rv = f.get("review") or {}
    line = rv.get("outcome", "unreviewed")
    if rv.get("adverse_observations"):
        line += " (" + ", ".join(rv["adverse_observations"]) + ")"
    body += f"\n    review    : {line}"
    if f.get("citation_problems"):
        body += "\n    citation problems: " + " | ".join(f["citation_problems"])
    return body


ASKED_WORDS = {"buyer": "the buyer asked this; the seller did not say it",
               "standard": "the practice asked this; the seller did not say it",
               "practice": "the practice asked this; the seller did not say it"}


def _facts(f: Dict[str, Any]) -> str:
    """Two facts the record holds that an `in_short` must respect
    (MATERIALITY §5b, §8): who made the statement, and whether the finding
    rests on what a library does, known rather than shown."""
    who = ASKED_WORDS.get(f.get("asked_by"), "the seller's claim")
    lib = ("yes: say so in words, \"from the library's documented behaviour\""
           if _from_knowledge(f) else "no")
    return f"    said by   : {who}\n    rests on knowledge of a library: {lib}"


def rate(loop, method_text: str, transaction: Optional[str],
         rateable: List[Dict[str, Any]], exposable: List[Dict[str, Any]],
         max_tokens: int, batch: int,
         thresholds: Optional[str] = None,
         seed: Optional[int] = None,
         reliance: Optional[str] = None) -> Dict[str, Any]:
    """The ratings, in batches. Same runner-decides-the-batch pattern as the
    review's emit_parts: one schema per part, concatenated by merge_parts.

    A batch holds one kind of finding (MATERIALITY §7): the `materiality`
    batches first, then the `exposure` ones, each asked for its own array.
    With `seed`, the findings are shuffled before batching: a rating is made
    in the company of its batch-mates, and a replicate that keeps the same
    company measures only sampling noise."""
    if seed is not None:
        import random
        rng = random.Random(seed)
        rateable = list(rateable); rng.shuffle(rateable)
        exposable = list(exposable); rng.shuffle(exposable)
    head = _head(transaction, thresholds, reliance)

    def groups_of(xs):
        return ([xs] if not batch else
                [xs[i:i + batch] for i in range(0, len(xs), batch)]) if xs else []

    plan = ([("materiality", "ratings", "exposures", g)
             for g in groups_of(rateable)]
            + [("exposure", "exposures", "ratings", g)
               for g in groups_of(exposable)])
    parts, calls, asks = [], [], []
    for n, (field, array, other, g) in enumerate(plan, 1):
        body = (f"Rate these {len(g)} findings for `{field}`, per MATERIALITY "
                f"\u00a73 to \u00a7{'5b' if field == 'materiality' else '5'}. Each carries its claim source and claim "
                f"id; return both as given.\n\n"
                + "\n\n".join(_rating_text(f) for f in g)
                + f"\n\nEmit `{array}` for exactly these findings, and an "
                  f"empty `{other}`.")
        asks.append(lambda lp, body=body, g=g: emit(
            lp, method_text, head + body,
            schemas.ratings_schema(sorted({f["claim_source"] for f in g})),
            max_tokens))
    # The batches are independent; with --parallel they are asked at once.
    for n, ((field, array, other, g), out) in enumerate(
            zip(plan, run_tasks(loop, asks)), 1):
        calls.append({"part": f"{array}[{n}/{len(plan)}]", "findings": len(g),
                      **{k: v for k, v in out.items() if k not in ("raw", "obj")}})
        if out["obj"] is not None:
            parts.append(out["obj"])
        else:
            logger.warning("batch %d/%d did not parse: %s", n, len(plan),
                           out["parse_error"])
    return {"obj": schemas.merge_parts(parts), "calls": calls}


def _observation_text(i: int, u: Dict[str, Any]) -> str:
    """One unclaimed observation as the rater reads it: its number, the claim
    source whose run recorded it, the note and its one evidence item."""
    e = u.get("evidence") or {}
    if e.get("form") == "derived":
        where = "; ".join(f"{b.get('document')} {b.get('lines')}: {b.get('quote')}"
                          for b in e.get("basis") or [] if isinstance(b, dict))
        ev = f"derived from {where} — {e.get('derivation')} — {e.get('consequence')}"
    else:
        ev = f"{e.get('document')} {e.get('lines')}: {e.get('quote')} — {e.get('shows')}"
    return f"--- observation {i} (recorded with {u.get('claim_source')})\n    note    : {u.get('note')}\n    evidence: {ev}"


def rate_observations(loop, method_text: str, head: str, unclaimed: List[Dict[str, Any]],
                      max_tokens: int) -> Dict[str, Any]:
    """The unclaimed observations, grouped by the fact they record and rated
    once per group (MATERIALITY §4). One call; asked once more when the
    groups do not cover every observation exactly once. Returns the groups
    (empty when both answers fail the check), the check and the calls."""
    body = (f"Here are {len(unclaimed)} observations the review recorded that no claim "
            f"asserts, numbered. Group the observations that record the same fact in "
            f"the target, and rate each group per MATERIALITY \u00a74 to \u00a75b.\n\n"
            + "\n\n".join(_observation_text(i, u) for i, u in enumerate(unclaimed, 1))
            + f"\n\nEmit `observations`: every number from 1 to {len(unclaimed)} in exactly one group.")
    calls, check = [], {"ok": False, "problems": ["not asked"]}
    for attempt in (1, 2):
        out = emit(loop, method_text, head + body, schemas.observations_schema(), max_tokens)
        calls.append({"part": f"observations[{attempt}]", "observations": len(unclaimed),
                      **{k: v for k, v in out.items() if k not in ("raw", "obj")}})
        obj = out["obj"] or {}
        check = schemas.check_observations(obj, len(unclaimed))
        if check["ok"]:
            return {"groups": obj["observations"], "check": check, "calls": calls}
        logger.warning("observation groups, answer %d: %s", attempt, "; ".join(check["problems"]))
    return {"groups": [], "check": check, "calls": calls}


def _in_short_items(ratings: Dict[str, Any], merged: Dict[str, Any]
                    ) -> List[Tuple[str, Dict[str, Any], str]]:
    """(key, the row that carries the `in_short`, what the check reads) for
    every rated gap and every group of observations that has one."""
    by_key = {f"{f['claim_source']}#{f['claim_id']}": f for f in merged["findings"]}
    unclaimed = merged.get("unclaimed") or []
    out = []
    for r in ratings.get("ratings") or []:
        k = f"{r.get('claim_source')}#{r.get('claim_id')}"
        f = by_key.get(k)
        if f and (r.get("in_short") or "").strip():
            out.append((k, r, f"--- {k}\n    gap       : {(f.get('adjudication') or {}).get('gap')}\n"
                              f"{_facts(f)}\n    in_short  : {r['in_short']}"))
    for n, g in enumerate(ratings.get("observations") or [], 1):
        if (g.get("in_short") or "").strip():
            members = [unclaimed[m - 1] for m in g.get("members") or [] if 1 <= m <= len(unclaimed)]
            knows = any((u.get("evidence") or {}).get("from_knowledge") is True for u in members)
            out.append((f"observation {n}", g,
                        f"--- observation {n}\n    note      : "
                        + " / ".join(str(u.get("note")) for u in members)
                        + "\n    said by   : nobody: the seller claimed nothing about it"
                        + f"\n    rests on knowledge of a library: {'yes' if knows else 'no'}"
                        + f"\n    in_short  : {g['in_short']}"))
    return out


def check_in_short(loop, method_text: str, ratings: Dict[str, Any], merged: Dict[str, Any],
                   max_tokens: int) -> Dict[str, Any]:
    """MATERIALITY §8. Every `in_short` that ships is checked against the
    gap it was written from, in one call. Those that fail are written again,
    in one call, with the check's reason, and checked once more; any that
    fail again lose their `in_short`, and the report shows the claim's
    statement and verdict in its place. A check that does not give every
    sentence one verdict is asked once more; if it fails again, every
    `in_short` is withdrawn: nothing unchecked reaches the summary.
    Returns {checks, rewritten, dropped, calls}."""
    items = _in_short_items(ratings, merged)
    calls: List[Dict[str, Any]] = []

    def ask(part, body, schema):
        out = emit(loop, method_text, body, schema, max_tokens)
        calls.append({"part": part, "items": len(items),
                      **{k: v for k, v in out.items() if k not in ("raw", "obj")}})
        return out["obj"] or {}

    def checked(subset, part):
        keys = [k for k, _, _ in subset]
        body = ("Check each `in_short` below against the gap it was written from, per "
                "MATERIALITY \u00a78. You did not write them.\n\n"
                + "\n\n".join(t for _, _, t in subset)
                + f"\n\nEmit `checks`: one verdict for each of the {len(keys)} keys.")
        for attempt in (1, 2):
            got = schemas.verdicts_of(ask(f"{part}[{attempt}]", body,
                                          schemas.in_short_checks_schema(keys)), keys)
            if got is not None:
                return got
        return None

    result: Dict[str, Any] = {"checks": {}, "rewritten": [], "dropped": [], "calls": calls}
    if not items:
        return result

    def drop(row, key, why):
        row.pop("in_short", None)
        row["in_short_dropped"] = why
        result["dropped"].append(key)

    first = checked(items, "in_short_check")
    if first is None:
        for k, row, _ in items:
            drop(row, k, "the check did not give a verdict on every sentence")
        return result
    result["checks"] = first
    failed = [(k, row, t) for k, row, t in items if first[k]["verdict"] != "holds"]
    if not failed:
        return result
    keys = [k for k, _, _ in failed]
    body = ("Each `in_short` below failed the check of MATERIALITY \u00a78 for the reason given. "
            "Write it again per \u00a75b, stating nothing the gap or the note does not state.\n\n"
            + "\n\n".join(f"{t}\n    check     : {first[k]['verdict']}: \"{first[k]['words']}\" — {first[k]['why']}"
                          for k, _, t in failed)
            + f"\n\nEmit `rewrites`: one for each of the {len(keys)} keys.")
    new = {r.get("key"): r.get("in_short") for r in
           ask("in_short_rewrite", body, schemas.in_short_rewrites_schema(keys)).get("rewrites") or []
           if (r.get("in_short") or "").strip()}
    for k, row, _ in failed:
        if k in new:
            row["in_short"] = new[k]
            result["rewritten"].append(k)
        else:
            drop(row, k, f"failed the check ({first[k]['verdict']}) and was not written again")
    again = [x for x in _in_short_items(ratings, merged) if x[0] in result["rewritten"]]
    second = checked(again, "in_short_recheck") if again else {}
    for k, row, _ in again:
        v = (second or {}).get(k)
        result["checks"][k] = v or {"verdict": "unchecked"}
        if not v or v["verdict"] != "holds":
            drop(row, k, f"failed the check again ({v['verdict'] if v else 'no verdict'})")
    return result


def _head(transaction: Optional[str], thresholds: Optional[str],
          reliance: Optional[str]) -> str:
    """What every rating call is read against: the transaction, the buyer's
    thresholds and the reliance statement."""
    return ("The transaction, as the engagement states it:\n\n"
            + (transaction.strip() if transaction else
               "(The engagement states nothing about the transaction. Rate "
               "against a buyer paying a price that assumes every claim "
               "holds, per MATERIALITY \u00a72.)")
            + "\n\nThe buyer's thresholds, as the engagement records them:\n\n"
            + (thresholds.strip() if thresholds else
               "(The engagement records no thresholds. Rate as MATERIALITY "
               "\u00a73 defines the values.)")
            # The same account of the buyer the claims were rated into tiers
            # against (claims_audit/reliance.py), where the engagement has one.
            + ("\n\nThe reliance statement:\n\n" + reliance.strip() if reliance else "")
            + "\n\n")


def _table(findings: List[Dict[str, Any]], rated: Dict[str, Dict[str, Any]],
           field: str, unrated: str) -> List[str]:
    """One table, highest value first. `unrated` labels a finding this table
    holds that carries no rating."""
    order = {m: i for i, m in enumerate(reversed(schemas.MATERIALITY))}
    rows = []
    for f in findings:
        r = rated.get(f"{f['claim_source']}#{f['claim_id']}")
        m = r.get(field) if r else unrated
        rows.append((order.get(m, 99), f, r, m))
    rows.sort(key=lambda x: (x[0], x[1]["claim_source"], x[1]["claim_id"]))
    lines = [f"| {field} | source | id | verdict | disposition | review | "
             f"citations | claim | basis |",
             "|---|---|---|---|---|---|---|---|---|"]
    for _, f, r, m in rows:
        rv = f.get("review") or {}
        review = rv.get("outcome", "unreviewed")
        if rv.get("adverse_observations"):
            review += " (" + ", ".join(rv["adverse_observations"]) + ")"
        cites = f"{len(f.get('citation_problems') or [])} problem(s)" \
            if f.get("citation_problems") else "ok"
        adj = f.get("adjudication") or {}
        q = (f.get("quote") or "").replace("|", "\\|").replace("\n", " ")
        b = ((r or {}).get("basis") or "").replace("|", "\\|").replace("\n", " ")
        lines.append(f"| {m} | {f['claim_source']} | {f['claim_id']} | "
                     f"{adj.get('verdict')} | {adj.get('unresolved_because') or ''} "
                     f"| {review} | {cites} | {q[:160]} | {b} |")
    return lines


def render(merged: Dict[str, Any], ratings: Dict[str, Any]) -> str:
    """materiality.md: what a person reads. Two tables — what the audit
    showed, rated for materiality, then the unsettled claims ranked by
    exposure — and `real` findings last. Highest value first in each."""
    by_m = {f"{r.get('claim_source')}#{r.get('claim_id')}": r
            for r in ratings.get("ratings") or []}
    by_e = {f"{r.get('claim_source')}#{r.get('claim_id')}": r
            for r in ratings.get("exposures") or []}
    fig = merged["figures"]
    rf = ratings.get("figures") or {}
    lines = ["# Materiality", "",
             f"{fig['runs']} run(s), {fig['findings']} findings, "
             f"{fig['reviewed']} reviewed, {fig['unreviewed']} unreviewed. "
             f"Materiality: "
             + (", ".join(f"{k} {v}" for k, v in
                          sorted((rf.get("materiality") or {}).items()))
                or "none")
             + ". Exposure: "
             + (", ".join(f"{k} {v}" for k, v in
                          sorted((rf.get("exposure") or {}).items())) or "none")
             + ".", ""]
    if fig.get("restated_quotes"):
        lines += ["## Restated across claim sources", ""]
        for x in fig["restated_quotes"]:
            lines.append("- " + "; ".join(f"{i['claim_source']} #{i['claim_id']} "
                                          f"{i['verdict']}" for i in x["in"])
                         + f" — \"{(x['quote'] or '')[:120]}\"")
        lines.append("")
    shown = [f for f in merged["findings"] if schemas.rateable(f)]
    unsettled = [f for f in merged["findings"] if schemas.exposable(f)]
    real = [f for f in merged["findings"]
            if not schemas.rateable(f) and not schemas.exposable(f)]
    lines += ["## What the audit showed, by materiality", ""]
    lines += _table(shown, by_m, "materiality", "not rated")
    lines += ["", "## Unsettled claims, by exposure", "",
              "Not a mark against the seller: the audit showed nothing "
              "against these claims. `disposition` says why each is "
              "unsettled; `not_examined` means the searches named a file "
              "the engagement did not open.", ""]
    lines += _table(unsettled, by_e, "exposure", "not rated")
    lines += ["", "## Claims that hold", ""]
    lines += _table(real, {}, "materiality", "real")
    if merged.get("questions"):
        lines += ["", "## Questions for the seller", ""]
        lines += [f"- ({q['claim_source']}"
                  + (f", claim {q['claim_id']}" if q.get("claim_id") is not None else "")
                  + f") {q['question']}"
                  for q in merged["questions"]]
    if merged.get("unclaimed"):
        lines += ["", "## Unclaimed observations", ""]
        unclaimed = merged["unclaimed"]
        for g in ratings.get("observations") or [{"members": [i]} for i in range(1, len(unclaimed) + 1)]:
            rem = g.get("remedy") or {}
            lines.append(f"- {g.get('materiality', 'not rated')}: "
                         + " / ".join(f"[{m}] ({unclaimed[m - 1]['claim_source']}) {unclaimed[m - 1].get('note')}"
                                      for m in g["members"])
                         + (f" — {g['basis']}" if g.get("basis") else "")
                         + (f" — remedy ({rem.get('band')}): {rem.get('change')}" if rem else ""))
    return "\n".join(lines) + "\n"


def replicate(loop, method_text: str, eng: Dict[str, Any],
              rateable: List[Dict[str, Any]], exposable: List[Dict[str, Any]],
              max_tokens: int, batch: int, first: Dict[str, Any],
              log: Dict[str, Any]) -> Dict[str, Any]:
    """THE REPLICATES (Bruce, 2026-09-03). A second pass over everything in
    shuffled batches; every finding the two passes rate differently is rated
    three more times, each in freshly shuffled company, and combined by
    schemas.combine: a majority of four or five ships with its count, three
    to two ships the plurality marked borderline. The spread this makes
    visible is the rater's own, on fixed inputs: two rating runs on one
    ChatterMate record moved four ratings between them."""
    by_key = {f"{f['claim_source']}#{f['claim_id']}": f
              for f in rateable + exposable}
    seeds = [1, 2, 3, 4]
    samples: Dict[str, Dict[str, List[Dict[str, Any]]]] = {
        "materiality": {}, "exposure": {}}

    def take(obj, field, array):
        for r in obj.get(array) or []:
            samples[field].setdefault(f"{r.get('claim_source')}#{r.get('claim_id')}", []).append(r)

    take(first["obj"], "materiality", "ratings")
    take(first["obj"], "exposure", "exposures")
    calls = list(first["calls"])
    second = rate(loop, method_text, eng.get("transaction"), rateable, exposable,
                  max_tokens, batch, thresholds=eng.get("thresholds"), seed=seeds[0],
                  reliance=eng.get("reliance"))
    calls += second["calls"]; log["passes"] = 2; log["seeds"].append(seeds[0])
    take(second["obj"], "materiality", "ratings")
    take(second["obj"], "exposure", "exposures")

    cm = schemas.contested("materiality", first["obj"].get("ratings") or [],
                           second["obj"].get("ratings") or [])
    ce = schemas.contested("exposure", first["obj"].get("exposures") or [],
                           second["obj"].get("exposures") or [])
    log["escalated"] = cm + ce
    if cm or ce:
        logger.info("replicates: %d contested (%d materiality, %d exposure) "
                    "— rating each three more times", len(cm) + len(ce), len(cm), len(ce))
        r_sub = [by_key[k] for k in cm if k in by_key]
        e_sub = [by_key[k] for k in ce if k in by_key]
        # The three further samples are independent of each other; with
        # --parallel they are rated at once.
        mores = run_tasks(loop, [
            lambda lp, seed=seed: rate(
                lp, method_text, eng.get("transaction"), r_sub, e_sub,
                max_tokens, batch, thresholds=eng.get("thresholds"), seed=seed,
                reliance=eng.get("reliance"))
            for seed in seeds[1:]])
        for seed, more in zip(seeds[1:], mores):
            calls += more["calls"]; log["passes"] += 1; log["seeds"].append(seed)
            take(more["obj"], "materiality", "ratings")
            take(more["obj"], "exposure", "exposures")

    def combined(field):
        out = []
        for key, rows in samples[field].items():
            out.append(schemas.combine(field, rows))
        return out

    obj = {"ratings": combined("materiality"), "exposures": combined("exposure")}
    log["borderline"] = sorted(f"{r['claim_source']}#{r['claim_id']}"
                               for field, arr in (("materiality", "ratings"), ("exposure", "exposures"))
                               for r in obj[arr] if r.get("borderline"))
    return {"obj": obj, "calls": calls}


def to_rate(merged: Dict[str, Any]) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """(rateable, exposable) findings for the model. A statement the buyer
    rated at intake keeps the buyer's rating (INTAKE.md §3). An unsettled one
    is left out; a gap is still sent, for its remedy and its plain account
    (MATERIALITY §5a, §5b), and
    add_buyer_ratings puts the buyer's rating in place of the model's."""
    fs = merged["findings"]
    return ([f for f in fs if schemas.rateable(f)],
            [f for f in fs if schemas.exposable(f) and not f.get("buyer_rating")])


def add_buyer_ratings(ratings: Dict[str, Any], merged: Dict[str, Any]) -> None:
    """Record the buyer's own rating for each finding it covers: as its
    materiality when the finding shows a gap, its exposure when unsettled.
    The model's row for such a gap is replaced, keeping only its remedy and
    its plain account."""
    for f in merged["findings"]:
        if not f.get("buyer_rating"):
            continue
        field, array = (("materiality", "ratings") if schemas.rateable(f) else
                        ("exposure", "exposures") if schemas.exposable(f) else (None, None))
        if not field:
            continue
        rows = ratings.setdefault(array, [])
        mine = [r for r in rows if (r.get("claim_source"), r.get("claim_id"))
                == (f["claim_source"], f["claim_id"])]
        for r in mine:
            rows.remove(r)
        row = {"claim_source": f["claim_source"], "claim_id": f["claim_id"],
               field: f["buyer_rating"], "by": "buyer",
               "basis": f"The buyer rated this question {f['buyer_rating'].replace('_', ' ')} "
                        f"at intake; the buyer's own rating stands."}
        for k in ("remedy", "in_short", "in_short_dropped"):
            if mine and mine[0].get(k) is not None:
                row[k] = mine[0][k]
        rows.append(row)


def _sha(text: Optional[str]) -> Optional[str]:
    import hashlib
    return hashlib.sha256(text.encode("utf-8")).hexdigest() if text else None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engagement", required=True,
                    help="engagement name under claims_audit/engagements/")
    ap.add_argument("--run", action="append", required=True, type=Path,
                    help="a reviewed audit run directory; repeat per claim source")
    ap.add_argument("--model", type=Path, default=None,
                    help="YAML with an llm_config block; replaces the scenario's")
    ap.add_argument("--batch", type=int, default=20,
                    help="findings per rating call; 0 rates all at once")
    ap.add_argument("--label", default=None,
                    help="suffix for the output directory (default: model stem)")
    ap.add_argument("--intake", default=None,
                    help="intake id whose transaction and thresholds the "
                         "ratings are read against (default: the "
                         "engagement's current intake)")
    ap.add_argument("--max-tokens", type=int, default=None)
    ap.add_argument("--single", action="store_true",
                    help="one rating pass; the default rates twice and "
                         "escalates a disagreement to five samples")
    ap.add_argument("--parallel", type=int, default=1,
                    help="rating calls in flight at once (default 1). The "
                         "site's production jobs raise it; a measurement run "
                         "keeps 1")
    args = ap.parse_args()
    set_parallel(args.parallel)

    eng = load_engagement(args.engagement, intake=args.intake)
    from workflowsv2.claims_audit import reliance as reliance_statement
    statement = reliance_statement.load(eng["dir"])
    eng["reliance"] = reliance_statement.render(statement) if statement else None
    ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    label = args.label or (args.model.stem if args.model else "default")
    out = eng["dir"] / "merged" / f"{ts}_{label}"
    out.mkdir(parents=True, exist_ok=False)
    fh = logging.FileHandler(out / "run.log", encoding="utf-8")
    fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(fh)
    # One line per model call into the same log, as the audit runner does,
    # so a stage's calls and tokens can be counted from its own record.
    _ul = logging.getLogger("chat.backend.usage")
    _ul.setLevel(logging.INFO)
    _ul.addHandler(fh)

    # ---- part one: merge ----------------------------------------------------
    merged = merging.merge(args.run)
    merged["engagement"] = eng["name"]
    (out / "merged.json").write_text(
        json.dumps(merged, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    fig = merged["figures"]
    logger.info("merged %d run(s): %d findings, %d reviewed, %d unreviewed",
                fig["runs"], fig["findings"], fig["reviewed"], fig["unreviewed"])
    for r in merged["runs"]:
        if not r["reviewed"]:
            issues.note(out, stage=STAGE, code="unreviewed_run",
                        text=f"{r['claim_source']}: no review — its findings "
                             f"are rated with outcome `unreviewed`",
                        severity="check")
    for x in fig["restated_quotes"]:
        issues.note(out, stage=STAGE, code="restated_quote",
                    text="one quote in several claim sources: "
                         + "; ".join(f"{i['claim_source']} #{i['claim_id']} "
                                     f"{i['verdict']}" for i in x["in"]),
                    severity="check")
    if not eng.get("transaction"):
        issues.note(out, stage=STAGE, code="no_transaction",
                    text="no `transaction:` block in the intake or "
                         "engagement.yaml; rated "
                         "against a buyer paying a price that assumes every "
                         "claim holds", severity="check")
    if not eng.get("thresholds"):
        issues.note(out, stage=STAGE, code="no_thresholds",
                    text="no `thresholds:` block in the intake or "
                         "engagement.yaml; ratings "
                         "are read against MATERIALITY \u00a73 alone, not "
                         "against what the buyer said would change the price",
                    severity="check")

    # ---- part two: rate -----------------------------------------------------
    rateable, exposable = to_rate(merged)
    from chat.chat_loop import ChatLoop                        # noqa: E402
    world = f"materiality_{ts}_{label}"[:60]
    name, cfg = build_config(out, world, args.model)
    loop = ChatLoop(character_name=name, character_config=cfg)
    method_text = load_workflow(REPO / METHOD_PATH)
    max_tokens = args.max_tokens or int((cfg.get("chat") or {})
                                        .get("react_max_tokens", 32768))
    t0 = datetime.datetime.now(datetime.timezone.utc)
    error, result = None, {"obj": {"ratings": [], "exposures": []}, "calls": []}
    observed: Dict[str, Any] = {"groups": [], "check": None}
    in_short: Dict[str, Any] = {"checks": {}, "rewritten": [], "dropped": [], "calls": []}
    replicates: Dict[str, Any] = {"passes": 0, "escalated": [], "seeds": []}
    try:
        result = rate(loop, method_text, eng.get("transaction"), rateable,
                      exposable, max_tokens, args.batch,
                      thresholds=eng.get("thresholds"),
                      reliance=eng.get("reliance"))
        replicates["passes"] = 1
        if not args.single:
            result = replicate(loop, method_text, eng, rateable, exposable,
                               max_tokens, args.batch, result, replicates)
        if merged.get("unclaimed"):
            observed = rate_observations(
                loop, method_text, _head(eng.get("transaction"), eng.get("thresholds"),
                                         eng.get("reliance")),
                merged["unclaimed"], max_tokens)
            result["calls"] += observed["calls"]
        # The plain accounts are checked once the ratings are final, while
        # the model is still open (MATERIALITY §8).
        add_buyer_ratings(result["obj"], merged)
        if observed["groups"]:
            result["obj"]["observations"] = observed["groups"]
        in_short = check_in_short(loop, method_text, result["obj"], merged, max_tokens)
        result["calls"] += in_short["calls"]
        for c in result["calls"]:
            if c["response_format_dropped"]:
                error = (f"the route dropped "
                         f"{', '.join(c['response_format_dropped'])} on "
                         f"{c['part']} — the ratings were not constrained")
            elif c["parse"] == "unparseable":
                error = f"{c['part']} did not parse"
    except Exception as e:                                     # noqa: BLE001
        error = f"{type(e).__name__}: {e}"
        logger.exception("rating failed")
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)

    ratings = result["obj"]
    add_buyer_ratings(ratings, merged)                 # again, should rating have failed before it
    ratings["in_short_checks"] = in_short["checks"]
    for k in in_short["dropped"]:
        issues.note(out, stage=STAGE, code="in_short_dropped",
                    text=f"{k}: the plain account failed the check and is not shown; the report "
                         f"gives the claim's statement and verdict instead (MATERIALITY \u00a78)",
                    severity="check")
    # THE OBSERVATIONS, GROUPED AND RATED. Absent when the groups failed
    # their check twice; the report then lists the observations unrated.
    if observed["groups"]:
        ratings["observations"] = observed["groups"]
    elif observed["check"] is not None:
        for p in observed["check"]["problems"]:
            issues.note(out, stage=STAGE, code="observations_check", text=p,
                        severity="check")
    check = schemas.check_ratings(ratings, merged)
    for p in check["problems"]:
        issues.note(out, stage=STAGE, code="ratings_check", text=p,
                    severity="blocking")
    ratings["figures"] = {k: check["figures"][k]
                          for k in ("materiality", "exposure")}

    for key in replicates.get("borderline") or []:
        issues.note(out, stage=STAGE, code="borderline_rating",
                    text=f"{key}: the replicates split — the shipped rating "
                         f"is the plurality, marked borderline; every basis "
                         f"is in materiality.json. The practice decides.",
                    severity="check")
    # A consequential rating on a finding the review did not uphold, or whose
    # citation failed the mechanical check, is for a person to read.
    by_key = {f"{f['claim_source']}#{f['claim_id']}": f for f in merged["findings"]}
    for field, r in ([("materiality", r) for r in ratings.get("ratings") or []]
                     + [("exposure", r) for r in ratings.get("exposures") or []]):
        f = by_key.get(f"{r.get('claim_source')}#{r.get('claim_id')}")
        if not f or r.get(field) == "not_material":
            continue
        rv = f.get("review") or {}
        if rv.get("outcome") == "does_not_hold" or f.get("citation_problems"):
            why = []
            if rv.get("outcome") == "does_not_hold":
                why.append("review: does not hold ("
                           + ", ".join(rv.get("adverse_observations") or []) + ")")
            if f.get("citation_problems"):
                why.append(f"{len(f['citation_problems'])} citation problem(s)")
            issues.note(out, stage=STAGE, code="rated_but_not_upheld",
                        text=f"{r['claim_source']} #{r['claim_id']} {field} "
                             f"{r[field]}; " + "; ".join(why),
                        severity="check")

    (out / "materiality.json").write_text(
        json.dumps(ratings, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    (out / "materiality.md").write_text(render(merged, ratings), encoding="utf-8")
    wall = round((datetime.datetime.now(datetime.timezone.utc) - t0).total_seconds(), 1)
    (out / "meta.json").write_text(json.dumps({
        "engagement": eng["name"], "world": world,
        "model_config": str(args.model) if args.model else None,
        "resolved_model": loop.backend.resolved_model(),
        "runs": [{k: r[k] for k in ("dir", "claim_source", "harness_rev",
                                    "resolved_model", "reviewed")}
                 for r in merged["runs"]],
        "batch": args.batch, "rateable": len(rateable),
        "replicates": replicates,
        # THE RUN PINS ITS INTAKE. The ratings were read against this
        # intake's blocks; the report stage reads the same ones back by this
        # id, and the hashes say whether the text has changed since.
        "intake": eng.get("intake_id"),
        "thresholds_recorded": bool(eng.get("thresholds")),
        "thresholds_sha256": _sha(eng.get("thresholds")),
        "transaction_sha256": _sha(eng.get("transaction")),
        "reliance_sha256": _sha(eng.get("reliance")),
        "exposable": len(exposable),
        "calls": result["calls"], "ratings_check": check,
        "observations_check": observed["check"],
        "in_short_check": {"verdicts": {v: sum(1 for c in in_short["checks"].values()
                                               if c.get("verdict") == v)
                                        for v in schemas.CHECK_VERDICTS + ("unchecked",)},
                           "rewritten": in_short["rewritten"], "dropped": in_short["dropped"]},
        "wall_clock_s": wall, "error": error, "parallel": args.parallel,
        "transient_events": getattr(loop, "transient_events", None),
        "captured_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    }, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\n{len(result['calls'])} call(s), {wall}s, error={error}")
    print(f"merged: {fig['findings']} findings from {fig['runs']} run(s); "
          f"rateable {len(rateable)}, exposable {len(exposable)}")
    print(f"ratings: {check['figures']}")
    print(f"ratings check: {'clean' if check['ok'] else str(len(check['problems'])) + ' problem(s)'}")
    print(f"out: {out}/materiality.md")
    return 1 if (error or not check["ok"]) else 0


if __name__ == "__main__":
    raise SystemExit(main())
