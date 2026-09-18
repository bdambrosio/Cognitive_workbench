#!/usr/bin/env python3
"""Mark the claims that repeat a claim already listed, across an engagement's
claim sources, after every source is enumerated and before any surface is
frozen.

    python3 workflowsv2/claims_audit/duplicates.py --engagement <name> --model <model yaml>

WHY A SEPARATE PASS. Each claim source is enumerated in its own run, and
`restates` can only name a claim of the same document. On chhoto with six
sources (2026-09-17) 24 of the project page's 25 claims repeated the README's,
and each would have been audited, reviewed and reported a second time. The
rule for "the same assertion" is in method/DUPLICATES.md; this file is the
plumbing.

WHAT IT CHANGES. The latest enumeration run of each source, in the order
engagement.yaml lists them. A claim found to repeat an earlier one gets
`same_as: {"source", "id", "statement"}` in that run's claims.json; a claim
found narrower than another, in either document, gets `within: {...}` naming
the wider one (DUPLICATES.md §3, approved 2026-09-17). The whole result is
written to `<engagement>/surface/duplicates.json`. Nothing is removed. The
surface page shows a `same_as` claim struck out beside the earlier statement,
left out unless the practice keeps it, and a `within` claim with the wider
statement beside it, still in: its verdict can be read from the wider claim's
finding only once the report derives it (not built), so until then it is
audited like any other and the mark is information.

ONLY CLOSE PAIRS ARE JUDGED (Bruce, 2026-09-17). Comparing each new claim
with every earlier one had the model reason through thousands of pairs, about
40 tokens of reasoning each: on tuuyi.com's ten pages (304 claims) the pass
took 17 minutes. Each statement is embedded (BAAI/bge-small-en-v1.5, the model
the resource manager uses, on the CPU: 304 statements in under a second). A
new claim is shown the earlier claims whose similarity to it is at least
FLOOR, the NEAREST most similar of them when there are more; a new claim with
none is not sent to the model at all. The embedding only decides what the
model is shown. Whether two claims are the same assertion is the model's
judgement under DUPLICATES.md, never a similarity figure.

THE SETTING LEANS TOWARDS MISSING A REPEAT (Bruce). A missed repeat stays on
the surface as its own claim and is tested twice: extra work, nothing lost.
A false merge removes a claim from the review unless the practice notices it
on the surface page. Measured against the pairs the every-pair pass found, at
0.75 and three: 337 pairs judged in place of about 45,000 on tuuyi.com and 167
in place of about 15,000 on chhoto, with 49 of 70 and 22 of 25 of the known
pairs among those judged. The repeats not shown are worded differently ("the
review includes an independent check" / "a checking process separate from the
process that produced the findings", 0.70).

WHAT WAS TRIED AND DID NOT BEAT THIS, the same day, against the same pairs.
bge-base and bge-large in place of bge-small: 48 and 45 of 70. A label for
each claim (its subject and the property asserted, written by the model),
embedded in place of the statement: 38 of 70 at 0.75 while judging 545 pairs,
because claims that share a subject ("review scope: not a penetration test",
"review scope: not a code-quality review") score above real repeats whose
subjects were named differently. Labels written with the running list of
subjects in view, grouped by identical subject: 34 of 53 on tuuyi.com and 16
of 17 on chhoto, judging 483 and 164 pairs, after two to four minutes of
labelling. A site's claims share one vocabulary, and no cheap representation
tried separates its repeats from its neighbours better than the statements do.

A CLAIM ABOUT BEHAVIOUR IN OPERATION IS COMPARED ONLY WITH OTHERS OF ITS
KIND. The behaviour split marks those rows (`property` "behaviour in
operation"), so this is read from the data. It keeps a behaviour row from
being folded into the mechanism claim it was split from, which the first
versions did.

A SOURCE THAT ALREADY HAS A DRAFT OR A FROZEN SURFACE IS NOT TOUCHED, as a
new claim. The practice has edited it, and the page reads the draft in
preference to the enumeration. Its claims still count as earlier claims for
the sources after it.

A PAIRING IS KEPT ONLY IF IT POINTS BACKWARDS: to a source listed earlier, or
to a smaller id in the same source, and to a claim that is not itself marked.
A pairing to a marked claim is followed to the claim that one points at.
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
import types
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2 import engagement_state as state               # noqa: E402
from workflowsv2.emit import emit                               # noqa: E402
from workflowsv2.claims_audit.decompose import backend_from_model  # noqa: E402
from chat.workflow import load_workflow                        # noqa: E402
from utils.file_utils import atomic_write_text                  # noqa: E402

logger = logging.getLogger("claims_audit.duplicates")

METHOD_PATH = HERE / "method" / "DUPLICATES.md"
# New claims per emission, and its token budget. At 8,192 tokens, four
# emissions of thirteen on 2026-09-17 spent the whole budget reasoning and
# returned nothing; the model works through the comparisons aloud.
BATCH = 30
MAX_TOKENS = 32768
NEAREST = 5                      # earlier claims shown for one new claim, at most
FLOOR = 0.70                     # cosine similarity below which none is shown
EMBEDDER = "BAAI/bge-small-en-v1.5"
Key = Tuple[str, int]
RELATIONS = ("same", "new_within_earlier", "earlier_within_new")


def schema() -> Dict[str, Any]:
    pair = {"type": "object", "properties": {
        "claim_id": {"type": "integer", "minimum": 1},
        "other_source": {"type": "string"},
        "other_id": {"type": "integer", "minimum": 1},
        "relation": {"type": "string", "enum": list(RELATIONS)}},
        "required": ["claim_id", "other_source", "other_id", "relation"]}
    return {"type": "object", "properties": {
        "pairs": {"type": "array", "items": pair}}, "required": ["pairs"]}


def is_behaviour(claim: Dict[str, Any]) -> bool:
    from workflowsv2.claims_audit.behaviour_split import PROPERTY
    return claim.get("implied_by") is not None and claim.get("property") == PROPERTY


def embed(statements: Sequence[str]):
    """Unit-length embeddings of the statements, one row each."""
    from sentence_transformers import SentenceTransformer
    try:
        model = SentenceTransformer(EMBEDDER, local_files_only=True, device="cpu")
    except Exception as e:                                     # noqa: BLE001
        logger.info("embedding model not in the local cache (%s); downloading", e)
        model = SentenceTransformer(EMBEDDER, device="cpu")
    return model.encode(list(statements), normalize_embeddings=True, show_progress_bar=False)


def nearest(row, allowed: Sequence[int]) -> List[int]:
    """Indices from `allowed`, the at most NEAREST most similar to the claim
    whose similarity row is `row`, none below FLOOR, most similar first."""
    ranked = sorted((i for i in allowed if row[i] >= FLOOR), key=lambda i: -row[i])
    return ranked[:NEAREST]


def propose(backend, source: str, new: Sequence[Tuple[Dict[str, Any], Sequence[Tuple[str, Dict[str, Any]]]]],
            max_tokens: int = MAX_TOKENS) -> Dict[str, Any]:
    """One emission under DUPLICATES.md. `new` is the claims of `source` to be
    judged, each with its shortlist of earlier (source, claim) pairs. Returns
    {"pairs": [...], "parse", "parse_error"}; the pairs are as emitted, not
    yet checked."""
    blocks = []
    for c, shortlist in new:
        listed = "\n".join(f"      [{s}] {e['id']}. {e.get('statement')}" for s, e in shortlist)
        blocks.append(f"  New claim {c['id']}. {c.get('statement')}\n    Earlier claims nearest to it:\n{listed}")
    user = (f"The new claims are from `{source}`. Each is followed by the earlier "
            f"claims nearest to it in wording, each with its document in brackets "
            f"and its id.\n\n" + "\n\n".join(blocks) + "\n\n"
            f"Emit the pairs per DUPLICATES.md §4.")
    out = emit(types.SimpleNamespace(backend=backend), load_workflow(METHOD_PATH),
               user, schema(), max_tokens)
    obj = out.get("obj") if isinstance(out.get("obj"), dict) else {}
    return {"pairs": [p for p in obj.get("pairs") or [] if isinstance(p, dict)],
            "parse": out.get("parse"), "parse_error": out.get("parse_error")}


def accept(pairs: Sequence[Dict[str, Any]], source: str, order: Sequence[str],
           claims: Dict[Key, Dict[str, Any]], same: Dict[Key, Key],
           within: Dict[Key, Key]) -> Tuple[Dict[Key, Key], Dict[Key, Key]]:
    """The pairings that are kept, as two maps: {claim: the earlier claim it
    is the same as} and {narrower claim: the wider claim it is within}.
    `same` and `within` are what earlier batches marked.

    A `same` pairing points backwards, to a source listed earlier or a
    smaller id in this one, and is followed through any claim already marked
    the same as another. A `within` pairing points either way (the narrower
    claim can be the earlier one); the wider claim is followed through a
    `same` mark, a claim marked the same as another absorbs nothing, and a
    claim has at most one mark of its own. A claim about a document is about
    its own file (DUPLICATES.md says so; the model still paired "this
    document is MIT-licensed" across three files on 2026-09-17), so the
    enumerator's `about` tag refuses such a pair in either relation."""
    out_same: Dict[Key, Key] = {}
    out_within: Dict[Key, Key] = {}

    def taken(k: Key) -> bool:
        return k in same or k in within or k in out_same or k in out_within

    for p in pairs:
        try:
            new: Key = (source, int(p["claim_id"]))
            other: Key = (str(p["other_source"]), int(p["other_id"]))
            relation = str(p["relation"])
        except (KeyError, TypeError, ValueError):
            continue
        other = same.get(other, out_same.get(other, other))
        if new not in claims or other not in claims or new == other or relation not in RELATIONS:
            continue
        if new[0] != other[0] and "document" in (claims[new].get("about"), claims[other].get("about")):
            continue
        if relation == "same":
            backwards = (order.index(other[0]) < order.index(source) if other[0] != source
                         else other[1] < new[1]) if other[0] in order else False
            if backwards and not taken(new) and not taken(other):
                out_same[new] = other
            continue
        narrower, wider = (new, other) if relation == "new_within_earlier" else (other, new)
        if taken(narrower) or wider in same or wider in out_same:
            continue
        out_within[narrower] = wider
    return out_same, out_within


def _touched(eng_dir: Path, source: str) -> bool:
    from client_ui.jobs import surface_file
    frozen = surface_file(eng_dir, source)
    return frozen.is_file() or frozen.with_name(frozen.name.replace(".surface.json", ".draft.json")).is_file()


def run(eng_dir: Path, model_yaml: Path) -> Dict[str, Any]:
    from client_ui import jobs
    order = state.claim_sources(eng_dir)
    backend = backend_from_model(model_yaml)
    runs: Dict[str, Path] = {}
    claims: Dict[Key, Dict[str, Any]] = {}
    for src in order:
        r = jobs.latest_enumeration_run(eng_dir, src)
        if r is None:
            raise SystemExit(f"{src} has not been enumerated")
        runs[src] = r
        for c in json.loads((r / "claims.json").read_text(encoding="utf-8")).get("claims") or []:
            claims[(src, int(c["id"]))] = c
    keys: List[Key] = [k for src in order for k in claims if k[0] == src]
    index = {k: i for i, k in enumerate(keys)}
    vectors = embed([str(claims[k].get("statement") or "") for k in keys])
    sim = vectors @ vectors.T
    marked: Dict[Key, Key] = {}          # claim -> the earlier claim it is the same as
    within: Dict[Key, Key] = {}          # narrower claim -> the wider claim
    emissions: List[Dict[str, Any]] = []
    skipped: List[str] = []
    for n, src in enumerate(order):
        if _touched(eng_dir, src):
            skipped.append(src)
            logger.info("%s has a draft or a frozen surface; its claims are left as they are", src)
            continue
        # Earlier claims: every claim of a source listed before this one, and
        # the claims of this source with a smaller id. A marked claim is never
        # offered; the claim it points at is.
        judged = []
        for k in (k for k in keys if k[0] == src):
            allowed = [index[e] for e in keys[:index[k]] if e not in marked
                       and is_behaviour(claims[e]) == is_behaviour(claims[k])]
            shortlist = [(keys[i][0], claims[keys[i]]) for i in nearest(sim[index[k]], allowed)]
            if shortlist:
                judged.append((claims[k], shortlist))
        for i in range(0, len(judged), BATCH):
            group = judged[i:i + BATCH]
            batch = [c for c, _ in group]
            out = propose(backend, src, group)
            if out["parse"] not in ("parsed", "repaired"):
                # No answer is not "no duplicates". Stop before anything is
                # marked, so a half-compared surface is never shown as done.
                raise SystemExit(f"{src} claims {batch[0]['id']}-{batch[-1]['id']}: the "
                                 f"comparison returned nothing usable ({out['parse']}: "
                                 f"{out['parse_error']}); no claim was marked")
            got_same, got_within = accept(out["pairs"], src, order, claims, marked, within)
            marked.update(got_same)
            within.update(got_within)
            emissions.append({"source": src, "claims": [c["id"] for c in batch],
                              "proposed": len(out["pairs"]), "same": len(got_same),
                              "within": len(got_within),
                              "parse": out["parse"], "parse_error": out["parse_error"]})
            logger.info("%s claims %s-%s: %d proposed, %d same, %d within", src,
                        batch[0]["id"], batch[-1]["id"], len(out["pairs"]),
                        len(got_same), len(got_within))
    for src in order:
        if src in skipped:
            continue
        f = runs[src] / "claims.json"
        doc = json.loads(f.read_text(encoding="utf-8"))
        for c in doc.get("claims") or []:
            k = (src, int(c["id"]))
            c.pop("same_as", None)
            c.pop("within", None)
            for key, m in (("same_as", marked), ("within", within)):
                o = m.get(k)
                if o is not None:
                    c[key] = {"source": o[0], "id": o[1],
                              "statement": claims[o].get("statement")}
        atomic_write_text(f, json.dumps(doc, indent=1, ensure_ascii=False) + "\n")
    record = {"at": state.stamp(), "model": backend.resolved_model(), "order": order,
              "runs": {s: r.name for s, r in runs.items()}, "skipped": skipped,
              "emissions": emissions,
              "pairs": [{"source": n[0], "id": n[1], "statement": claims[n].get("statement"),
                         "similarity": round(float(sim[index[n], index[o]]), 3),
                         "relation": rel,
                         "other": {"source": o[0], "id": o[1],
                                   "statement": claims[o].get("statement")}}
                        for rel, m in (("same", marked), ("within", within))
                        for n, o in sorted(m.items(), key=lambda kv: (order.index(kv[0][0]), kv[0][1]))]}
    d = eng_dir / state.SURFACE
    d.mkdir(exist_ok=True)
    atomic_write_text(d / "duplicates.json", json.dumps(record, indent=1, ensure_ascii=False) + "\n")
    return record


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engagement", required=True)
    ap.add_argument("--model", type=Path, required=True)
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    eng_dir = state.ENGAGEMENTS / args.engagement
    if not eng_dir.is_dir():
        raise SystemExit(f"no engagement '{args.engagement}'")
    rec = run(eng_dir, args.model)
    per: Dict[str, int] = {}
    for p in rec["pairs"]:
        per[p["source"]] = per.get(p["source"], 0) + 1
    print(f"{len(rec['pairs'])} claims repeat an earlier one: "
          + (", ".join(f"{s} {n}" for s, n in per.items()) or "none"))
    print(eng_dir / state.SURFACE / "duplicates.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
