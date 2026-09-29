"""The `nidana` action: the consultation agent's lookup into the knowledge
base, registered on the ChatLoop the way claims_audit/record.py registers
`claim`. CONSULT.md §4 requires every quotation to come from here."""
from __future__ import annotations

import logging
from types import SimpleNamespace
from typing import Any, Dict, List

logger = logging.getLogger("ayur_consult.record")

TOOL_NAME = "nidana"
MAX_IDS = 6
DESCRIPTION = (
    "look up Madhava Nidana in the knowledge base by id: a verse (MN.2.6) "
    "returns its Devanagari, transliteration and the translation of its "
    "passage; a passage (MN.2.4-5, one section of the chapter) returns its "
    "section heading, its place in the Sanskrit edition, each verse in "
    "Devanagari and IAST, the translation and the ids of its commentary; a "
    "whole commentary (MN.2.4-5:mk for the Madhukośa, MN.2.4-5:at for the "
    "Ātaṅkadarpaṇa) returns its Sanskrit and English, segment by segment; a "
    "commentary segment (MN.2.4-5:mk3) returns that segment's Sanskrit and "
    "English; "
    "a disease (jvara) returns its names, variants and cited "
    "features; a feature (F.yawning) returns its description and the "
    "diseases that cite it. Up to 6 ids per call.")
ARGS = {"ids": "list of ids, e.g. [\"MN.2.6\", \"jvara\", \"F.yawning\"]"}


def lookup(kb, ids: List[str]) -> Dict[str, Any]:
    if not isinstance(ids, list) or not ids:
        return {"status": "error", "text": "give `ids` as a list, e.g. [\"MN.2.6\"]"}
    if len(ids) > MAX_IDS:
        return {"status": "error", "text": f"at most {MAX_IDS} ids per call"}
    parts = []
    for i in ids:
        i = str(i).strip()
        if kb.verse(i):
            parts.append(kb.render_verse(i))
        elif kb.segment(i):
            g = kb.segment(i)
            parts.append(f"{i} ({g['commentary']} on {g['passage']}):\n{g['sa']}\n"
                         f"English: {g['en'] or '(untranslated)'}")
        elif kb.commentary(i):
            c = kb.commentary(i)
            parts.append(f"{i} ({c['commentary']} on {c['passage']}; edition "
                         f"{c.get('edition') or 'not recorded'}):\n" + "\n".join(
                             f"[{g['id']}] {g['sa']}\n    English: {g['en'] or '(untranslated)'}"
                             for g in c["segments"]))
        elif i in kb.passages:
            ps = kb.passages[i]
            verses = "\n".join(f"{v}: {kb.verse(v)['sa']}\n    {kb.verse(v).get('iast') or ''}"
                               for v in ps["verses"] if kb.verse(v))
            coms = [f"{i}:{short} ({len(v)} segments)" for name, short in
                    (("madhukosha", "mk"), ("atankadarpana", "at"))
                    for v in [(ps.get("commentary") or {}).get(name)] if isinstance(v, list) and v]
            parts.append(f"{i} — section: {ps.get('section')}; edition: "
                         f"{ps.get('edition') or 'not recorded'}\n"
                         f"Sanskrit:\n{verses or '(no verses)'}\n"
                         f"Translation: {ps.get('translation') or '(none)'}\n"
                         f"Commentaries: {', '.join(coms) or 'none'}")
        elif kb.disease(i):
            d = kb.disease(i)
            feats = "\n".join(
                f"  - {f['feature']} ({(kb.feature(f['feature']) or {}).get('en', '')}): "
                f"{f['role']}, {f['weight']}, variant {f.get('variant') or 'all'}, "
                f"{'/'.join(f['verses']) or 'UNCITED'}" for f in d.get("features", []))
            parts.append(f"{i}: {d.get('names')}\nvariants: "
                         f"{[v['id'] for v in d.get('variants') or []] or 'none'}\nfeatures:\n{feats}")
        elif kb.feature(i):
            f = kb.feature(i)
            where = [f"{d['id']} ({x['role']}, {'/'.join(x['verses']) or 'uncited'})"
                     for d in kb.diseases.values() for x in d.get("features", []) if x["feature"] == i]
            parts.append(f"{i}: {f.get('en')} — {f.get('clinical')}; Sanskrit {f.get('sa')}; "
                         f"ask: {f.get('lay_question')}\ncited for: {', '.join(where) or 'nothing'}")
        else:
            parts.append(f"{i}: not in the knowledge base")
    return {"status": "ok", "text": "\n\n".join(parts)}


def register(loop: Any, kb) -> None:
    loop._discovered_tools[TOOL_NAME] = {
        "description": DESCRIPTION, "args": dict(ARGS), "module_path": None, "body": ""}
    loop._tool_module_cache[TOOL_NAME] = SimpleNamespace(
        react_invoke=lambda args, **_kw: lookup(kb, (args or {}).get("ids")))
    logger.info("nidana lookup: %d verses, %d diseases, %d features",
                len(kb.verses), len(kb.diseases), len(kb.lexicon))
