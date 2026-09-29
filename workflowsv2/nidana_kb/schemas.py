"""The shapes of the clinical layer: the extraction schema (EXTRACT.md §5)
and the check every extracted feature must pass before it is kept."""
from __future__ import annotations

from typing import Any, Dict, List, Tuple

ROLES = ("nidana", "purvarupa", "rupa", "upashaya", "anupashaya", "samprapti",
         "upadrava", "arishta", "sadhyata")
WEIGHTS = ("cardinal", "supporting")
DOSHAS = ("vata", "pitta", "kapha")
OBSERVABLE = ("reported", "examined", "wearable", "lab")

_STR = {"type": "string"}
_STRS = {"type": "array", "items": _STR}


def extract_schema() -> Dict[str, Any]:
    feature = {"type": "object", "properties": {
        "feature": _STR, "role": {"type": "string", "enum": list(ROLES)},
        "variant": _STR, "weight": {"type": "string", "enum": list(WEIGHTS)},
        "verses": _STRS, "quote": _STR},
        "required": ["feature", "role", "variant", "weight", "verses", "quote"]}
    variant = {"type": "object", "properties": {
        "id": _STR, "doshas": {"type": "array", "items": {"type": "string", "enum": list(DOSHAS)}},
        "label": _STR}, "required": ["id", "doshas", "label"]}
    disease = {"type": "object", "properties": {
        "id": _STR,
        "names": {"type": "object", "properties": {"sa": _STR, "iast": _STR, "en": _STR},
                  "required": ["sa", "iast", "en"]},
        "variants": {"type": "array", "items": variant},
        "features": {"type": "array", "items": feature}},
        "required": ["id", "names", "variants", "features"]}
    new_feature = {"type": "object", "properties": {
        "id": _STR, "en": _STR, "clinical": _STR, "sa": _STRS, "lay_question": _STR,
        "observable": {"type": "array", "items": {"type": "string", "enum": list(OBSERVABLE)}}},
        "required": ["id", "en", "clinical", "sa", "lay_question", "observable"]}
    return {"type": "object", "properties": {
        "diseases": {"type": "array", "items": disease},
        "new_features": {"type": "array", "items": new_feature},
        "notes": _STRS},
        "required": ["diseases", "new_features", "notes"]}


def check_feature(f: Dict[str, Any], disease: Dict[str, Any], kb, known_features: set
                  ) -> Tuple[Dict[str, Any], List[str]]:
    """A feature as it will be kept, and what was wrong with it. A verse id
    that does not resolve, or whose text does not hold the quote, is removed
    from the feature; a feature left with no verse is kept with status
    `uncited`, which the consultation does not use."""
    problems: List[str] = []
    verses = []
    for vid in f.get("verses") or []:
        res = kb.resolve_citation(vid, f.get("quote") or None)
        if res["ok"]:
            verses.append(vid)
        else:
            problems.append(res["why"])
    variant_ids = {v["id"] for v in disease.get("variants") or []}
    variant = f.get("variant") or None
    if variant and variant not in variant_ids:
        problems.append(f"variant {variant} is not a variant of {disease['id']}")
        variant = None
    if f.get("feature") not in known_features:
        problems.append(f"feature {f.get('feature')} is neither in the lexicon nor described")
    kept = {"feature": f.get("feature"), "role": f.get("role"), "variant": variant,
            "weight": f.get("weight"), "verses": verses, "quote_iast": f.get("quote") or "",
            "status": "draft" if verses else "uncited"}
    return kept, problems
