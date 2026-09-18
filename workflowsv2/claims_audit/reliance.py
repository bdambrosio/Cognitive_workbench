"""Write down what this buyer relies on the offering for, once per engagement,
after enumeration and before the claims are rated into tiers.

    python3 workflowsv2/claims_audit/reliance.py --engagement <name> --model <yaml>

One schema-constrained emission under RELIANCE.md. The call gets everything
the buyer said (the current intake's whole form; where there is no intake,
the engagement's `transaction` and `thresholds`) and every claim's statement,
by claim source. It has no tools and no access to the target.

WHY A SEPARATE TEXT. The buyer's thresholds are in business language and the
claims are in technical language. Without this text every tier call worked
out for itself what the buyer would use and could work around, differently
each time, and sometimes by inventing facts about the buyer. Written once, it
is one page a person can read and correct, and every tier call reads the same
account (TIERS.md §2).

WHAT IS WRITTEN. `surface/reliance.json` in the engagement: `use`, `items[]`,
and when and by which model it was written. A person corrects it by editing
that file. Running this again overwrites it, corrections included.
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
import types
from pathlib import Path
from typing import Any, Dict, List

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

logger = logging.getLogger("reliance")

METHOD_PATH = HERE / "method" / "RELIANCE.md"
RECORD = "reliance.json"
RELIANCE = ("depends", "uses", "does_not_use")
SOURCES = ("buyer", "inference")


def schema() -> Dict[str, Any]:
    item = {"type": "object", "properties": {
        "item": {"type": "string"},
        "reliance": {"type": "string", "enum": list(RELIANCE)},
        "if_it_failed": {"type": "string"},
        "source": {"type": "string", "enum": list(SOURCES)},
        "buyer_words": {"type": "string"}},
        "required": ["item", "reliance", "if_it_failed", "source", "buyer_words"]}
    return {"type": "object", "properties": {
        "use": {"type": "string"},
        "items": {"type": "array", "items": item}},
        "required": ["use", "items"]}


def buyer_said(eng: Dict[str, Any]) -> str:
    """The current intake's whole form; else the engagement's two blocks."""
    intake_id = state.current_intake(eng["dir"])
    form = state.intake_dir(eng["dir"], intake_id) / "intake.json" if intake_id else None
    if form is not None and form.is_file():
        return "The intake form:\n\n" + form.read_text(encoding="utf-8").strip()
    return ("The transaction:\n\n" + (str(eng.get("transaction") or "").strip() or "(the engagement states none)")
            + "\n\nThe buyer's thresholds:\n\n"
            + (str(eng.get("thresholds") or "").strip() or "(the engagement records none)"))


def inventory(eng_dir: Path) -> str:
    """Every claim's statement, by claim source; repeats of an earlier claim
    left out, as the tier stage leaves them out."""
    from workflowsv2.claims_audit.tiers import claims_file
    parts: List[str] = []
    for src in state.claim_sources(eng_dir):
        f = claims_file(eng_dir, src)
        if f is None:
            raise SystemExit(f"{src} has not been enumerated")
        claims = [c for c in json.loads(f.read_text(encoding="utf-8")).get("claims") or []
                  if not c.get("same_as")]
        parts.append(f"{src} ({len(claims)} claims):\n"
                     + "\n".join(f"  - {c.get('statement')}" for c in claims))
    return "\n\n".join(parts)


def propose(backend, said: str, claims_text: str, max_tokens: int = 16384) -> Dict[str, Any]:
    """One emission. Returns {"use", "items", "parse", "parse_error", "raw"};
    an item with no name, or a value outside the schema's, is dropped."""
    user = (f"{said}\n\nThe claims:\n\n{claims_text}\n\n"
            f"Emit the reliance statement per RELIANCE.md §5.")
    out = emit(types.SimpleNamespace(backend=backend), load_workflow(METHOD_PATH),
               user, schema(), max_tokens)
    obj = out.get("obj") if isinstance(out.get("obj"), dict) else {}
    items = []
    for x in obj.get("items") or []:
        if not isinstance(x, dict):
            continue
        name = str(x.get("item") or "").strip()
        if name and x.get("reliance") in RELIANCE and x.get("source") in SOURCES:
            items.append({"item": name, "reliance": x["reliance"],
                          "if_it_failed": str(x.get("if_it_failed") or "").strip(),
                          "source": x["source"],
                          "buyer_words": str(x.get("buyer_words") or "").strip()})
    return {"use": str(obj.get("use") or "").strip(), "items": items,
            "parse": out.get("parse"), "parse_error": out.get("parse_error"),
            "raw": out.get("raw")}


def render(record: Dict[str, Any]) -> str:
    """The statement as the tier call reads it."""
    lines = [str(record.get("use") or "").strip(), ""]
    for x in record.get("items") or []:
        rests = (f"the buyer said: \"{x['buyer_words']}\"" if x.get("source") == "buyer"
                 else "the practice's inference")
        lines.append(f"- {x['item']} — {x['reliance']} ({rests}). "
                     f"If it failed: {x.get('if_it_failed') or '(not stated)'}")
    return "\n".join(lines).strip()


def load(eng_dir: Path) -> Dict[str, Any]:
    """The engagement's reliance statement, or an empty dict when it has none."""
    p = eng_dir / state.SURFACE / RECORD
    return json.loads(p.read_text(encoding="utf-8")) if p.is_file() else {}


def run(eng_name: str, model_yaml: Path) -> Dict[str, Any]:
    from workflowsv2.claims_audit.runner import load_engagement
    eng = load_engagement(eng_name)
    eng_dir = eng["dir"]
    backend = backend_from_model(model_yaml)
    got = propose(backend, buyer_said(eng), inventory(eng_dir))
    if got["parse"] not in ("parsed", "repaired") or not got["items"]:
        raise SystemExit(f"the reliance statement came back unusable ({got['parse']}: "
                         f"{got['parse_error']}); nothing was written")
    record = {"at": state.stamp(), "model": backend.resolved_model(),
              "use": got["use"], "items": got["items"]}
    (eng_dir / state.SURFACE).mkdir(exist_ok=True)
    atomic_write_text(eng_dir / state.SURFACE / RECORD,
                      json.dumps(record, indent=1, ensure_ascii=False) + "\n")
    return record


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engagement", required=True)
    ap.add_argument("--model", type=Path, required=True)
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    rec = run(args.engagement, args.model)
    n = {r: sum(1 for x in rec["items"] if x["reliance"] == r) for r in RELIANCE}
    print(f"{len(rec['items'])} items: depends {n['depends']}, uses {n['uses']}, "
          f"does_not_use {n['does_not_use']}; "
          f"{sum(1 for x in rec['items'] if x['source'] == 'inference')} inferred")
    print(state.ENGAGEMENTS / args.engagement / state.SURFACE / RECORD)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
