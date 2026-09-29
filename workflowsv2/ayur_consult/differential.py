"""The differential and the next questions, computed from the case's findings
and the knowledge base. No model call: the ranking is arithmetic a reviewer
can recompute by hand, and CONSULT.md tells the agent to report it, not to
make its own.

SCORING. Each finding the practitioner recorded is matched to a lexicon
feature (session.py does the matching). For each disease variant in the
knowledge base, a present finding that is one of the variant's cited features
adds that feature's weight; an absent finding subtracts part of it. The
weight is the feature's own weight (cardinal or supporting) times its role's
weight (a sign of the manifest disease counts more than a cause). A variant
caused by a doṣa the practitioner has confirmed as currently aggravated
(vikriti) gets a small addition. Only variants with at least one present
finding are listed. Every weight is in WEIGHTS below.

NEXT QUESTIONS. Among the features of the leading variants that the case has
not recorded either way, the ones whose answer would best separate the
hypotheses: each leading variant, and "none of these" (a disease the findings
do not yet point to, or none). A feature held by some leaders and not others
separates them; a feature all the leaders share still separates them from
"none of these", so it confirms the disease without telling its forms apart.
"""
from __future__ import annotations

from typing import Any, Dict, Iterable, List, Optional

#: The arithmetic, in one place. `role` values are EXTRACT.md §2's roles;
#: roles not listed count zero (samprapti and sadhyata are not findings).
WEIGHTS: Dict[str, Any] = {
    "weight": {"cardinal": 2.0, "supporting": 1.0},
    "role": {"rupa": 1.0, "purvarupa": 0.8, "nidana": 0.5, "upashaya": 0.5,
             "anupashaya": 0.5, "upadrava": 0.6},
    # An absent finding subtracts this share of the feature's weight.
    "absent": {"cardinal": 1.0, "supporting": 0.25},
    # Added once when the variant's doṣa is among the confirmed vikriti.
    "vikriti_match": 0.5,
    "top_k": 5,
    # The score mass given to "none of these" when choosing questions.
    "none_mass": 1.0,
    "questions": 3,
}


def _feature_weight(f: Dict[str, Any]) -> float:
    return (WEIGHTS["weight"].get(f.get("weight"), 1.0)
            * WEIGHTS["role"].get(f.get("role"), 0.0))


def _usable(features: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """The cited features that count, each feature once: a feature the
    disease lists generally and the variant lists again (as its cardinal
    sign) counts at the higher of its two weights, not twice."""
    best: Dict[str, Dict[str, Any]] = {}
    for f in features:
        if f.get("status") == "uncited" or not f.get("verses") or _feature_weight(f) <= 0:
            continue
        cur = best.get(f["feature"])
        if cur is None or _feature_weight(f) > _feature_weight(cur):
            best[f["feature"]] = f
    return list(best.values())


def findings_by_feature(findings: List[Dict[str, Any]]) -> Dict[str, str]:
    """feature id → present | absent | unclear. A feature recorded both
    present and absent (the practitioner corrected themselves) takes the
    later entry."""
    out: Dict[str, str] = {}
    for f in findings:
        fid = f.get("feature_id")
        if fid:
            out[fid] = f.get("status") or "unclear"
    return out


def rank(variants: List[Dict[str, Any]], findings: List[Dict[str, Any]],
         vikriti: Optional[List[str]] = None) -> List[Dict[str, Any]]:
    """Every variant with a present finding, highest score first. Each entry:
    id, disease, names, doshas, score, coverage (share of the variant's
    weight found present), supporting and against (feature, role, weight,
    verses), and red_flags (arishta features found present)."""
    seen = findings_by_feature(findings)
    vik = set(vikriti or [])
    out = []
    for v in variants:
        support, against, red = [], [], []
        score, total, present_w = 0.0, 0.0, 0.0
        for f in v["features"]:
            if f.get("role") == "arishta" and seen.get(f["feature"]) == "present" and f.get("verses"):
                red.append({"feature": f["feature"], "verses": f["verses"]})
        for f in _usable(v["features"]):
            w = _feature_weight(f)
            total += w
            st = seen.get(f["feature"])
            item = {"feature": f["feature"], "role": f["role"], "weight": f.get("weight"),
                    "verses": f["verses"]}
            if st == "present":
                score += w
                present_w += w
                support.append(item)
            elif st == "absent":
                score -= w * WEIGHTS["absent"].get(f.get("weight"), 0.25)
                against.append(item)
        if not support and not red:
            continue
        if vik & set(v.get("doshas") or []):
            score += WEIGHTS["vikriti_match"]
        out.append({"id": v["id"], "disease": v["disease"], "names": v.get("names", {}),
                    "label": v.get("label") or v["id"],
                    "doshas": v.get("doshas", []), "score": round(score, 3),
                    "coverage": round(present_w / total, 3) if total else 0.0,
                    "supporting": support, "against": against, "red_flags": red})
    out.sort(key=lambda r: (-r["score"], -r["coverage"], r["id"]))
    return out


def next_questions(ranked: List[Dict[str, Any]], variants: List[Dict[str, Any]],
                   findings: List[Dict[str, Any]], n: Optional[int] = None
                   ) -> List[Dict[str, Any]]:
    """The features to ask about next: not yet recorded, held by some of the
    leading variants, ranked by how evenly the leaders' score mass divides on
    them (p·(1−p), p the share of mass whose variant holds the feature),
    times the feature's weight; the mass includes "none of these". Each entry:
    feature, value, held_by and not_held_by (leading variant ids), verses."""
    n = n or WEIGHTS["questions"]
    top = ranked[:WEIGHTS["top_k"]]
    if not top:
        return []
    by_id = {v["id"]: v for v in variants}
    seen = findings_by_feature(findings)
    mass = {r["id"]: max(r["score"], 0.0) + 0.1 for r in top}   # every leader counts a little
    total = sum(mass.values()) + WEIGHTS["none_mass"]
    feats: Dict[str, Dict[str, Any]] = {}
    for r in top:
        for f in _usable(by_id[r["id"]]["features"]):
            if f["feature"] in seen:
                continue
            e = feats.setdefault(f["feature"], {"held_by": set(), "w": 0.0, "verses": set()})
            e["held_by"].add(r["id"])
            e["w"] = max(e["w"], _feature_weight(f))
            e["verses"] |= set(f["verses"])
    scored = []
    for fid, e in feats.items():
        p = sum(mass[v] for v in e["held_by"]) / total
        value = p * (1 - p) * e["w"]
        scored.append({"feature": fid, "value": round(value, 4),
                       "held_by": sorted(e["held_by"]),
                       "not_held_by": sorted(set(mass) - e["held_by"]),
                       "verses": sorted(e["verses"])})
    scored.sort(key=lambda s: (-s["value"], s["feature"]))
    return scored[:n]
