"""Rate each claim by what its being false would change for this buyer,
before the surface is frozen.

    python3 workflowsv2/claims_audit/tiers.py --engagement <name> --model <yaml> [--source <claim source>]

One schema-constrained emission under TIERS.md per batch of claims from one
claim source. The call gets the engagement's `transaction` and `thresholds`
(the current intake's blocks, else engagement.yaml's), the engagement's
reliance statement where `reliance.py` has written one, and the claims; it
has no tools and no access to the target.

WHICH CLAIMS. For each claim source: the frozen surface if there is one, else
the draft, else the latest enumeration run's claims.json. A claim marked
`same_as` an earlier one is not rated (TIERS.md §2); a claim marked `within`
a wider one is.

WHAT IS WRITTEN. Each rated claim gets `tier` and `tier_basis` in the file it
was read from, the way the duplicates pass marks `same_as`: the surface page
shows them, a person changes any tier before the freeze, the freeze carries
them, and the audit tests tier 1 only. Two things are left as they are: a
frozen surface, which only a person unfreezes, and a claim whose tier a
person set (`tier_by`). `surface/tiers.json` is the record: per claim source,
every rating, and the ids the call did not rate. A claim the call did not
rate has no tier and is tested; it is never given a tier by default, because
a missing answer is not tier 3.
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
import types
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2 import engagement_state as state                  # noqa: E402
from workflowsv2.emit import emit                                   # noqa: E402
from workflowsv2.claims_audit.decompose import backend_from_model   # noqa: E402
from chat.workflow import load_workflow                            # noqa: E402
from utils.file_utils import atomic_write_text                    # noqa: E402

logger = logging.getLogger("tiers")

METHOD_PATH = HERE / "method" / "TIERS.md"
BATCH = 15
RECORD = "tiers.json"


def schema() -> Dict[str, Any]:
    item = {"type": "object", "properties": {
        "claim_id": {"type": "integer", "minimum": 1},
        "tier": {"type": "integer", "enum": [1, 2, 3]},
        "basis": {"type": "string"}},
        "required": ["claim_id", "tier", "basis"]}
    return {"type": "object", "properties": {
        "tiers": {"type": "array", "items": item}},
        "required": ["tiers"]}


def claims_file(eng_dir: Path, source: str) -> Optional[Path]:
    """The frozen surface, else the draft, else the latest enumeration."""
    from client_ui import jobs
    frozen = jobs.surface_file(eng_dir, source)
    draft = frozen.with_name(frozen.name.replace(".surface.json", ".draft.json"))
    for f in (frozen, draft):
        if f.is_file():
            return f
    r = jobs.latest_enumeration_run(eng_dir, source)
    return (r / "claims.json") if r is not None else None


def propose(backend, transaction: str, thresholds: str, source: str,
            claims: Sequence[Dict[str, Any]], reliance: str = "",
            max_tokens: int = 32768) -> Dict[str, Any]:
    """One emission for one batch. Returns {"tiers": [...], "unrated": [ids],
    "parse", "parse_error", "raw"}; an entry naming an id not in `claims`,
    a second entry for one id, or an entry with an empty basis is dropped."""
    listed = "\n".join(f"  {c.get('id')}. about: {c.get('about')}\n"
                       f"      quote: {c.get('quote')}\n"
                       f"      statement: {c.get('statement')}" for c in claims)
    user = (f"The transaction:\n\n{transaction.strip() or '(the engagement states none)'}\n\n"
            f"The buyer's thresholds:\n\n{thresholds.strip() or '(the engagement records none)'}\n\n"
            + (f"The reliance statement:\n\n{reliance.strip()}\n\n" if reliance.strip() else "") +
            f"The claims, from the claim source {source}:\n\n{listed}\n\n"
            f"Emit one tier for every claim per TIERS.md §7.")
    out = emit(types.SimpleNamespace(backend=backend), load_workflow(METHOD_PATH),
               user, schema(), max_tokens)
    obj = out.get("obj") if isinstance(out.get("obj"), dict) else {}
    ids = [int(c.get("id")) for c in claims]
    got: Dict[int, Dict[str, Any]] = {}
    for t in obj.get("tiers") or []:
        if not isinstance(t, dict):
            continue
        try:
            cid, tier = int(t.get("claim_id")), int(t.get("tier"))
        except (TypeError, ValueError):
            continue
        basis = str(t.get("basis") or "").strip()
        if cid in ids and cid not in got and tier in (1, 2, 3) and basis:
            got[cid] = {"claim_id": cid, "tier": tier, "basis": basis}
    return {"tiers": [got[i] for i in ids if i in got],
            "unrated": [i for i in ids if i not in got],
            "parse": out.get("parse"), "parse_error": out.get("parse_error"),
            "raw": out.get("raw")}


def mark(claims_path: Path, tiers: Sequence[Dict[str, Any]]) -> int:
    """Write `tier` and `tier_basis` onto the claims of one draft or
    enumeration file. A claim whose tier a person set is left alone. Returns
    how many claims were marked."""
    by_id = {t["claim_id"]: t for t in tiers}
    doc = json.loads(claims_path.read_text(encoding="utf-8"))
    n = 0
    for c in doc.get("claims") or []:
        t = by_id.get(c.get("id"))
        if t is None or c.get("tier_by"):
            continue
        c["tier"], c["tier_basis"] = t["tier"], t["basis"]
        n += 1
    atomic_write_text(claims_path, json.dumps(doc, indent=1, ensure_ascii=False) + "\n")
    return n


def run(eng_name: str, model_yaml: Path, only: Optional[str] = None) -> Dict[str, Any]:
    from workflowsv2.claims_audit.runner import load_engagement
    eng = load_engagement(eng_name)
    eng_dir = eng["dir"]
    sources = [s for s in state.claim_sources(eng_dir) if only in (None, s)]
    if not sources:
        raise SystemExit(f"{only} is not a claim source of {eng_name}")
    backend = backend_from_model(model_yaml)
    from workflowsv2.claims_audit import reliance
    statement = reliance.load(eng_dir)
    if not statement:
        logger.info("%s has no reliance statement; rating on the transaction and thresholds alone", eng_name)
    out_path = eng_dir / state.SURFACE / RECORD
    record = (json.loads(out_path.read_text(encoding="utf-8")) if out_path.is_file()
              else {"sources": {}})
    for src in sources:
        f = claims_file(eng_dir, src)
        if f is None:
            raise SystemExit(f"{src} has not been enumerated")
        claims = [c for c in json.loads(f.read_text(encoding="utf-8")).get("claims") or []
                  if not c.get("same_as")]
        tiers: List[Dict[str, Any]] = []
        unrated: List[int] = []
        for i in range(0, len(claims), BATCH):
            batch = claims[i:i + BATCH]
            got = propose(backend, str(eng.get("transaction") or ""),
                          str(eng.get("thresholds") or ""), src, batch,
                          reliance.render(statement) if statement else "")
            if got["parse"] not in ("parsed", "repaired"):
                raise SystemExit(f"{src} claims {batch[0]['id']}-{batch[-1]['id']}: the rating "
                                 f"returned nothing usable ({got['parse']}: {got['parse_error']})")
            tiers += got["tiers"]
            unrated += got["unrated"]
            logger.info("%s claims %s-%s: %d rated, %d unrated", src, batch[0]["id"],
                        batch[-1]["id"], len(got["tiers"]), len(got["unrated"]))
        marked = mark(f, tiers) if not f.name.endswith(".surface.json") else 0
        if f.name.endswith(".surface.json"):
            logger.info("%s is frozen; its claims are left as they are", src)
        record["sources"][src] = {"at": state.stamp(), "model": backend.resolved_model(),
                                  "marked": marked,
                                  "claims_from": str(f.relative_to(eng_dir)),
                                  "reliance_statement": statement.get("at"),
                                  "tiers": tiers, "unrated": unrated}
    (eng_dir / state.SURFACE).mkdir(exist_ok=True)
    atomic_write_text(out_path, json.dumps(record, indent=1, ensure_ascii=False) + "\n")
    return record


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engagement", required=True)
    ap.add_argument("--model", type=Path, required=True)
    ap.add_argument("--source", default=None,
                    help="rate one claim source only, by its path as engagement.yaml lists it")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    rec = run(args.engagement, args.model, args.source)
    for src, r in rec["sources"].items():
        n = {t: sum(1 for x in r["tiers"] if x["tier"] == t) for t in (1, 2, 3)}
        print(f"{src}: tier 1 {n[1]}, tier 2 {n[2]}, tier 3 {n[3]}, unrated {len(r['unrated'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
