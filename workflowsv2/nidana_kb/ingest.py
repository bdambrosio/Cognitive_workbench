#!/usr/bin/env python3
"""Read a translated Madhava Nidana document into the knowledge base's text
layer, one file per chapter.

    python3 workflowsv2/nidana_kb/ingest.py <doc.docx> [--kb <dir>] [--replace]
    python3 workflowsv2/nidana_kb/ingest.py <doc.docx> --dry-run

WHAT THE DOCUMENT MUST LOOK LIKE is in TRANSLATION_FORMAT.md beside this file,
which is what the translation team is asked to follow. This parser reads that
format, and also the earlier sampler the team sent (a Purva Rupa compilation
across chapters), which labels less and so is read by script instead: a
paragraph written mostly in Devanagari is verse text, a Latin paragraph
carrying `|| n ||` is its transliteration. Both are reading a document
format, not classifying meaning.

WHAT IT WRITES. `<kb>/text/chNN.json` for every chapter the document holds:
the verses (Devanagari, IAST, source reference) and the passages (a run of
verses and the one translation that follows them, with its word list). A
passage whose translation has no verse before it is kept, with verses [], and
flagged `unanchored`: it is content, but it cannot be cited.

WHAT IT CHECKS, all in code: every verse has a numeral; the Devanagari and the
IAST numerals pair up; every verse has IAST and sits in a passage with a
translation. Each failure is a flag on the verse or passage and a line in
`<kb>/issues.jsonl`. It does not judge whether a verse is correct Sanskrit;
that is the reviewer's job (review.py).

A chapter already in the knowledge base from a different document is not
overwritten without --replace.
"""
from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import re
import sys
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2 import issues                                  # noqa: E402
from workflowsv2.nidana_kb import kb as kbmod                   # noqa: E402

STAGE = "ingest"
W = "{http://schemas.openxmlformats.org/wordprocessingml/2006/main}"

#: The paragraph labels of TRANSLATION_FORMAT.md, in the order a verse block
#: uses them. A label is the paragraph's first word(s) followed by a colon.
LABELS = ("verse", "source", "iast", "padaccheda", "words", "translation",
          "meaning", "madhukosha", "atankadarpana", "note", "edition")
#: The two commentaries a passage may carry, by label, and the short name a
#: commentary segment's id uses (MN.2.9-10:mk3).
COMMENTARIES = ("madhukosha", "atankadarpana")
COMMENTARY_ID = {"madhukosha": "mk", "atankadarpana": "at"}
#: A commentary paragraph numbered as a segment: "[3] …". The Devanagari and
#: the English of segment 3 carry the same number (compose.py).
SEGMENT_RE = re.compile(r"^\[(\d+)\]\s*(.*)$", re.S)
#: The label word, an optional parenthesis ("Meaning (Shloka 6):"), then the
#: colon; so an English sentence that starts "Note that … :" is not a label.
LABEL_RE = re.compile(r"^\s*(" + "|".join(LABELS) + r")\s*(?:\([^)]{0,40}\))?\s*:\s*", re.I)
CHAPTER_RE = re.compile(r"\bchapter\s+(\d+)\b", re.I)
DEVA_DIGITS = str.maketrans("०१२३४५६७८९", "0123456789")
#: A verse ends with its number between double dandas: ॥ ४ ॥ or || 4 ||.
#: A half verse split across sections is numbered 14a and 14b.
DEVA_END_RE = re.compile(r"(?:॥|\|\|)\s*([०-९0-9]+[ab]?)\s*(?:॥|\|\|)")
IAST_END_RE = re.compile(r"\|\|\s*([0-9]+[ab]?)\s*\|\|")
#: A source reference in the sampler: a parenthesised Devanagari abbreviation.
DEVA_CITE_RE = re.compile(r"^\(\s*[ऀ-ॿ][ऀ-ॿ\s.?०-९]*\)$")
WORD_ITEM_RE = re.compile(r"^(.{1,80}?)\s+[—–-]\s+(.+)$")


# ---- reading the docx -----------------------------------------------------

def paragraphs(path: Path) -> List[Dict[str, Any]]:
    """Every paragraph of the document body, in order: its style, its text
    and whether any of its text is bold. Table cells are paragraphs too."""
    with zipfile.ZipFile(path) as z:
        root = ET.fromstring(z.read("word/document.xml"))
    out = []
    for p in root.find(W + "body").iter(W + "p"):
        ppr = p.find(W + "pPr")
        style = ""
        if ppr is not None and ppr.find(W + "pStyle") is not None:
            style = ppr.find(W + "pStyle").get(W + "val") or ""
        text = "".join(t.text or "" for t in p.iter(W + "t"))
        out.append({"style": style, "text": text.strip()})
    return out


def heading_level(style: str) -> Optional[int]:
    m = re.match(r"(?i)heading\s*(\d)$", style or "")
    return int(m.group(1)) if m else None


def devanagari_share(text: str) -> float:
    letters = [c for c in text if c.isalpha()]
    if not letters:
        return 0.0
    return sum(1 for c in letters if "ऀ" <= c <= "ॿ") / len(letters)


def split_verses(text: str, pattern: re.Pattern) -> List[Tuple[str, str]]:
    """A paragraph may hold several verses; each ends at its numeral. Returns
    (numeral, verse text including its ending). Text after the last numeral,
    or a paragraph with none, comes back with numeral ''."""
    out, start = [], 0
    for m in pattern.finditer(text):
        out.append((m.group(1).translate(DEVA_DIGITS), text[start:m.end()].strip()))
        start = m.end()
    rest = text[start:].strip(" |।॥")
    if rest:
        out.append(("", text[start:].strip()))
    return out


# ---- the parse ------------------------------------------------------------

def _new_passage(chapter: int, section: str) -> Dict[str, Any]:
    return {"chapter": chapter, "section": section, "verses": [], "padaccheda": "",
            "words": [], "translation": [], "commentary": {k: [] for k in COMMENTARIES},
            "notes": [], "flags": [], "edition": ""}


def parse(paras: List[Dict[str, Any]]) -> Dict[int, Dict[str, Any]]:
    """The document as chapters: {chapter: {"title", "verses", "passages",
    "skipped"}}. Text before the first chapter heading, and after a heading
    that names no chapter, is counted in `skipped` and not kept."""
    chapters: Dict[int, Dict[str, Any]] = {}
    chapter: Optional[int] = None
    chapter_level: Optional[int] = None
    section = ""
    passage: Optional[Dict[str, Any]] = None
    mode = ""                      # the label the current run of paragraphs sits under
    skipped = 0

    def ch() -> Dict[str, Any]:
        return chapters[chapter]

    def close() -> None:
        nonlocal passage
        if passage and (passage["verses"] or passage["translation"] or passage["words"]
                        or any(passage["commentary"].values())):
            ch()["passages"].append(passage)
        passage = None

    def current() -> Dict[str, Any]:
        nonlocal passage
        if passage is None:
            passage = _new_passage(chapter, section)
        return passage

    for para in paras:
        text, level = para["text"], heading_level(para["style"])
        if not text:
            continue
        if level is not None:
            m = CHAPTER_RE.search(text)
            if m:
                if chapter is not None:
                    close()
                chapter, chapter_level, section, mode = int(m.group(1)), level, "", ""
                chapters.setdefault(chapter, {"title": text, "verses": [], "passages": [],
                                              "skipped": 0})
                continue
            if chapter is not None and chapter_level is not None and level > chapter_level:
                close()
                section, mode = text.rstrip(":"), ""
                continue
            if chapter is not None:          # a heading at or above the chapter's level ends it
                close()
            chapter, chapter_level, mode = None, None, ""
            continue
        if chapter is None:
            skipped += 1
            continue

        lab = LABEL_RE.match(text)
        label = lab.group(1).lower() if lab else ""
        body = text[lab.end():].strip() if lab else text

        if label == "verse" or (not label and devanagari_share(body) > 0.6
                                and not DEVA_CITE_RE.match(body)
                                and mode not in ("padaccheda",) + COMMENTARIES):
            # Verse text. After a translation it starts a new passage.
            if passage and (passage["translation"] or passage["words"]):
                close()
            pieces = split_verses(body, DEVA_END_RE)
            tail = None
            if len(pieces) > 1 and not pieces[-1][0]:
                # Text after the paragraph's last numeral: the sampler puts
                # some numerals mid-verse. Kept with the verse before it.
                tail = pieces.pop()[1]
            for n, v in pieces:
                current()["verses"].append({"n": n, "sa": v, "iast": "", "cites": "", "flags": []})
            if tail:
                last = current()["verses"][-1]
                last["sa"] += " " + tail
                last["flags"].append({"code": "text_after_numeral",
                                      "text": "the paragraph continues after this verse's numeral"})
            mode = "verse"
            continue
        if label == "source" or (not label and DEVA_CITE_RE.match(body)
                                 and mode not in COMMENTARIES):
            if passage and passage["verses"]:
                # A source line under a run of verses applies to each of them
                # that has none yet.
                for v in passage["verses"]:
                    v["cites"] = v["cites"] or body.strip("()").strip()
            continue
        if label == "iast" or (not label and IAST_END_RE.search(body) and mode in ("verse", "iast")):
            pieces = split_verses(body, IAST_END_RE)
            p = current()
            for n, v in pieces:
                target = next((x for x in p["verses"] if x["n"] == n and not x["iast"]), None) if n else None
                if target is None:
                    target = next((x for x in p["verses"] if not x["iast"]), None)
                if target is None:
                    p["flags"].append({"code": "iast_without_verse", "text": v[:80]})
                    continue
                if n and target["n"] and n != target["n"]:
                    p["flags"].append({"code": "numeral_mismatch",
                                       "text": f"verse ॥{target['n']}॥ paired with IAST ||{n}||"})
                target["iast"] = v
            mode = "iast"
            continue
        if label == "padaccheda":
            current()["padaccheda"] = (current()["padaccheda"] + " " + body).strip()
            mode = "padaccheda"
            continue
        if label == "words":
            mode = "words"
            if body:
                current()["words"].extend(_word_items(body))
            continue
        if label in ("translation", "meaning"):
            mode = "translation"
            if body:
                current()["translation"].append(body)
            continue
        if label in COMMENTARIES:
            mode = label
            if body:
                _commentary_para(current()["commentary"][label], body)
            continue
        if label == "note":
            current()["notes"].append(body)
            continue
        if label == "edition":
            # "<file> §<n>": the section of the team's edition this passage
            # comes from (compose.py writes it under each section heading).
            current()["edition"] = body
            continue

        # An unlabelled paragraph continues whatever label it sits under.
        # In the sampler a "Meaning:" is followed by "Term — gloss" lines.
        p = current()
        if mode in COMMENTARIES:
            _commentary_para(p["commentary"][mode], body)
        elif mode in ("words", "translation") and WORD_ITEM_RE.match(body):
            p["words"].extend(_word_items(body))
        elif mode == "padaccheda":
            p["padaccheda"] += " " + body
        else:
            if mode in ("verse", "iast"):
                # English straight after verses with no label: the sampler
                # does this; treat it as the translation.
                mode = "translation"
            if WORD_ITEM_RE.match(body) and not p["translation"]:
                p["words"].extend(_word_items(body))
                mode = "words"
            else:
                p["translation"].append(body)
                mode = "translation"
    if chapter is not None:
        close()
    for c in chapters.values():
        c["skipped"] = skipped
    return chapters


def _commentary_para(segs: List[Dict[str, Any]], text: str) -> None:
    """Add one commentary paragraph to its segments. A numbered paragraph
    ([k]) in Devanagari opens segment k and in English is its translation.
    An unnumbered Devanagari paragraph opens a segment without a number; an
    unnumbered English one continues the last segment's translation."""
    m = SEGMENT_RE.match(text)
    k, body = (int(m.group(1)), m.group(2).strip()) if m else (None, text.strip())
    if devanagari_share(body) > 0.5:
        segs.append({"k": k, "sa": body, "en": ""})
        return
    seg = next((x for x in reversed(segs) if x["k"] == k), None) if k is not None else None
    if seg is None:
        if not segs or k is not None:
            segs.append({"k": k, "sa": "", "en": ""})
        seg = segs[-1]
    seg["en"] = (seg["en"] + "\n\n" + body).strip()


def _word_items(text: str) -> List[Dict[str, str]]:
    out = []
    for line in re.split(r"\n+", text):
        m = WORD_ITEM_RE.match(line.strip())
        if m:
            out.append({"term": m.group(1).strip(), "en": m.group(2).strip()})
    return out


# ---- assembling the chapter files ------------------------------------------

def assemble(chapter: int, raw: Dict[str, Any], source: Dict[str, Any]) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    """The chapter file and the problems found, each problem also a flag on
    the verse or passage it concerns."""
    verses: List[Dict[str, Any]] = []
    passages: List[Dict[str, Any]] = []
    problems: List[Dict[str, Any]] = []
    seen: Dict[str, int] = {}
    for i, p in enumerate(raw["passages"], 1):
        ids = []
        for v in p["verses"]:
            if not v["n"]:
                problems.append({"code": "verse_without_numeral", "where": f"MN.{chapter}",
                                 "text": v["sa"][:80]})
                continue
            vid = f"MN.{chapter}.{v['n']}"
            flags = list(v.get("flags") or [])
            if vid in seen:
                # The edition repeats a number (a sub-passage numbered from 1,
                # or a misprint). Kept, not dropped, under a second id.
                seen[vid] += 1
                vid = f"{vid}r{seen[vid]}"
                flags.append({"code": "duplicate_verse",
                              "text": "this verse number appears earlier in the chapter; "
                                      "kept as a repeat"})
            else:
                seen[vid] = 1
            if not v["iast"]:
                flags.append({"code": "missing_iast", "text": "no transliteration"})
            verses.append({"id": vid, "n": v["n"], "section": p["section"], "sa": v["sa"],
                           "iast": v["iast"], "cites": v["cites"], "flags": flags,
                           "status": "draft", "reviewed_by": None, "reviewed_at": None})
            ids.append(vid)
            for f in flags:
                problems.append({"code": f["code"], "where": vid, "text": f["text"]})
        pid = (f"MN.{chapter}.{ids[0].rsplit('.', 1)[1]}" +
               (f"-{ids[-1].rsplit('.', 1)[1]}" if len(ids) > 1 else "")) if ids else f"MN.{chapter}.u{i}"
        flags = list(p["flags"])
        if not ids:
            flags.append({"code": "unanchored",
                          "text": "translation with no verse before it; it cannot be cited"})
        if ids and not p["translation"] and not p["words"]:
            flags.append({"code": "missing_translation", "text": "verses with no translation"})
        for f in flags:
            problems.append({"code": f["code"], "where": pid, "text": f["text"]})
        passages.append({"id": pid, "section": p["section"], "verses": ids,
                         "padaccheda": p["padaccheda"], "words": p["words"],
                         "translation": "\n".join(p["translation"]),
                         "commentary": {k: [dict(x, id=f"{pid}:{COMMENTARY_ID[k]}{x['k'] or j + 1}")
                                             for j, x in enumerate(v)]
                                         for k, v in p["commentary"].items() if v} or None,
                         "notes": p["notes"], "edition": p.get("edition") or None,
                         "flags": flags, "status": "draft", "reviewed_by": None,
                         "reviewed_at": None})
    doc = {"chapter": chapter, "title": raw["title"], "source": source,
           "verses": verses, "passages": passages}
    return doc, problems


def ingest(doc_path: Path, kb_dir: Path, replace: bool = False,
           dry_run: bool = False) -> Dict[str, Any]:
    data = doc_path.read_bytes()
    source = {"doc": doc_path.name, "sha256": hashlib.sha256(data).hexdigest(),
              "ingested_at": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")}
    chapters = parse(paragraphs(doc_path))
    if not chapters:
        raise SystemExit(f"{doc_path}: no heading names a chapter ('Chapter N'); nothing read")
    report: Dict[str, Any] = {"chapters": {}, "problems": 0, "written": [], "refused": []}
    for n, raw in sorted(chapters.items()):
        doc, problems = assemble(n, raw, source)
        report["chapters"][n] = {"verses": len(doc["verses"]), "passages": len(doc["passages"]),
                                 "problems": problems}
        report["problems"] += len(problems)
        if dry_run:
            continue
        path = kbmod.text_path(kb_dir, n)
        if path.is_file() and not replace:
            old = json.loads(path.read_text(encoding="utf-8"))
            if old.get("source", {}).get("doc") != source["doc"]:
                report["refused"].append(n)
                continue
        kbmod.write_json(path, doc)
        report["written"].append(n)
        for pr in problems:
            issues.note(kb_dir, stage=STAGE, code=pr["code"], severity="check",
                        text=f"{pr['where']}: {pr['text']}", doc=source["doc"])
    if not dry_run:
        kbmod.update_manifest(kb_dir, source, report["written"])
    return report


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("doc", type=Path, help="the translated chapter document (.docx)")
    ap.add_argument("--kb", type=Path, default=None,
                    help=f"knowledge base directory (default $AYUR_KB or {kbmod.DEFAULT_KB})")
    ap.add_argument("--replace", action="store_true",
                    help="overwrite a chapter already ingested from a different document")
    ap.add_argument("--dry-run", action="store_true", help="parse and report; write nothing")
    args = ap.parse_args()
    kb_dir = kbmod.kb_root(args.kb)
    if not args.dry_run:
        kb_dir.mkdir(parents=True, exist_ok=True)
    rep = ingest(args.doc, kb_dir, replace=args.replace, dry_run=args.dry_run)
    for n, c in rep["chapters"].items():
        print(f"chapter {n:>2}: {c['verses']:>3} verses, {c['passages']:>3} passages, "
              f"{len(c['problems'])} problem(s)")
        for pr in c["problems"]:
            print(f"      {pr['code']:<22} {pr['where']}: {pr['text'][:70]}")
    if rep["refused"]:
        print(f"not written (already ingested from another document; use --replace): "
              f"{', '.join(map(str, rep['refused']))}")
    if not args.dry_run:
        print(f"wrote {len(rep['written'])} chapter file(s) under {kb_dir / 'text'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
