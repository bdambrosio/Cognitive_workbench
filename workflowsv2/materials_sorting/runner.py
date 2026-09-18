#!/usr/bin/env python3
"""Sort the seller's materials, after they are marked ready and before
enumeration: propose the engagement's claim sources and evidence excludes,
and apply them once a person has confirmed.

    python3 workflowsv2/materials_sorting/runner.py --engagement <name> --model <model yaml>
    python3 workflowsv2/materials_sorting/runner.py --engagement <name> --confirm --by <email>
            [--claim-source <path> ...] [--exclude <path> ...]

The procedure this carries out, and the three engagements it was drawn from,
are in docs/claim-source-selection.md.

WHAT IS CODE AND WHAT IS THE MODEL. The listings are code, because they are
the part a person skips: every prose file in the target, the files inside
archives, the files and hosts the prose links to, the repository's hosting
description. They are deliberately too wide. What KIND each file is (a
description, an instrument, product text, neither) is decided by reading it,
one model call per file under method/SORTING.md, never by its name. The two
lists then follow from the kinds by rule (`propose`), and a person confirms
or changes them. Nothing is extracted and engagement.yaml is not touched until
`--confirm`.

WHAT THIS VERSION DOES NOT DO. It does not fetch anything outside the target
except the hosting description. Outside hosts are listed with the number of
links to them, as the client's choice; whoever prepares the engagement fetches
and snapshots the ones chosen, by hand, per the procedure. It does not decide
regions of a web page: an HTML claim source is extracted as its title, its
meta description and its main content, and the text of the dropped header,
navigation and footer is printed in the record so a person sees what was lost.

THE RECORD lives at `<engagement>/sorting/`: `selection.json` (the listing,
every model answer, the proposal, the confirmation) and `SELECTION.md`, the
same thing for reading. It is outside target/, which the seller can see.
"""
from __future__ import annotations

import argparse
import io
import json
import logging
import re
import subprocess
import sys
import urllib.request
import zipfile
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

import types                                                     # noqa: E402

from workflowsv2 import engagement_state as state               # noqa: E402
from workflowsv2.emit import emit                               # noqa: E402
from workflowsv2.claims_audit.decompose import backend_from_model  # noqa: E402
from chat.workflow import load_workflow                        # noqa: E402
from utils.doc_extract import html_to_markdown, pdf_to_markdown  # noqa: E402
from utils.file_utils import atomic_write_text                  # noqa: E402

logger = logging.getLogger("materials_sorting")

METHOD_PATH = HERE / "method" / "SORTING.md"
SORTING = "sorting"
EXTRACTED = "claim_sources"          # under target/: where extracted text goes
KINDS = ("description", "instrument", "product_text", "neither")
TEXT_WORDS = 1500                    # words of a file the model is shown
MIN_WORDS = 20                       # below this a file is listed as near-empty
MAX_HOSTS = 15                       # outside hosts printed in SELECTION.md
# Words of a description per enumerated claim, for the estimate a person sees
# before choosing claim sources. Measured on 2026-09-17 with GLM-5.3-Flash:
# chhoto's six sources gave 14 to 18 words per claim (326 claims from 4,966
# words), this site's how-it-works page 21. An estimate of the enumeration's
# output, duplicates between documents included.
WORDS_PER_CLAIM = 15

# A LISTING RULE, NOT A CLASSIFICATION. Files of these types are source code,
# configuration, data or media; they stay evidence and are not shown to the
# model. The record counts what was passed over by type, so the rule is
# visible. Everything else that decodes as text is read.
PASSED_OVER = {
    ".py", ".pyi", ".js", ".mjs", ".cjs", ".ts", ".tsx", ".jsx", ".vue", ".rs", ".go",
    ".java", ".kt", ".c", ".h", ".cc", ".cpp", ".hpp", ".cs", ".rb", ".php", ".swift",
    ".sh", ".bash", ".ps1", ".sql", ".css", ".scss", ".less", ".lua", ".ex", ".exs",
    ".json", ".yaml", ".yml", ".toml", ".ini", ".cfg", ".env", ".lock", ".xml", ".csv",
    ".tsv", ".map", ".snap", ".ipynb",
    ".png", ".jpg", ".jpeg", ".gif", ".webp", ".ico", ".svg", ".svgz", ".bmp", ".mp4",
    ".mp3", ".wav", ".webm", ".woff", ".woff2", ".ttf", ".otf", ".eot",
    ".gz", ".tar", ".tgz", ".7z", ".jar", ".whl", ".so", ".dll", ".exe", ".bin", ".db",
    ".sqlite", ".pyc"}

_MD_LINK = re.compile(r"\]\(\s*<?([^)\s>]+)")
_HREF = re.compile(r"""(?:href|src)\s*=\s*["']([^"']+)["']""", re.I)
_URL = re.compile(r"https?://[^\s)>\]\"'`]+")


# ---- the listings -----------------------------------------------------------

def _tracked(target: Path) -> List[str]:
    """Paths of the files in the target, by git when it is a repository."""
    try:
        r = subprocess.run(["git", "-C", str(target), "ls-files", "-z"],
                           capture_output=True, timeout=60)
        if r.returncode == 0 and r.stdout:
            return sorted(x for x in r.stdout.decode("utf-8", "replace").split("\0") if x)
    except Exception as e:                                     # noqa: BLE001
        logger.warning("git ls-files in %s: %s", target, e)
    return sorted(str(p.relative_to(target)) for p in target.rglob("*")
                  if p.is_file() and ".git" not in p.relative_to(target).parts)


def _text_of(name: str, data: bytes) -> Optional[str]:
    """The readable text of one file's bytes, or None when it has none."""
    ext = Path(name).suffix.lower()
    try:
        if ext == ".pdf":
            return pdf_to_markdown(data)
        text = data.decode("utf-8")
    except Exception as e:                                     # noqa: BLE001
        logger.info("no text from %s: %s", name, e)
        return None
    if ext in (".html", ".htm"):
        from bs4 import BeautifulSoup
        soup = BeautifulSoup(text, "html.parser")
        head = [f"Page title: {soup.title.get_text(strip=True)}"] if soup.title else []
        meta = soup.find("meta", attrs={"name": "description"})
        if meta and meta.get("content"):
            head.append(f"Page description: {meta['content']}")
        return "\n\n".join(head + [html_to_markdown(text)])
    return text


def list_candidates(target: Path) -> Tuple[List[Dict[str, Any]], Dict[str, int]]:
    """Every file in the target, and inside its zip archives, that holds
    prose. Returns the candidates and a count of the files passed over, by
    type. A candidate: {path, archive, words, text, raw}."""
    out: List[Dict[str, Any]] = []
    passed: Counter = Counter()

    def consider(path: str, data: bytes, archive: Optional[str]) -> None:
        ext = Path(path).suffix.lower()
        if ext in PASSED_OVER:
            passed[ext] += 1
            return
        text = _text_of(path, data)
        if text is None:
            passed["(not text)"] += 1
            return
        out.append({"path": path, "archive": archive, "words": len(text.split()),
                    "text": text, "raw": data.decode("utf-8", "replace")})

    for rel in _tracked(target):
        if rel.split("/")[0] == EXTRACTED:
            continue
        f = target / rel
        if not f.is_file():
            continue
        if f.suffix.lower() == ".zip":
            try:
                with zipfile.ZipFile(f) as z:
                    for info in z.infolist():
                        if not info.is_dir():
                            consider(info.filename, z.read(info), rel)
            except Exception as e:                             # noqa: BLE001
                logger.warning("archive %s not read: %s", rel, e)
                passed["(unreadable archive)"] += 1
            continue
        consider(rel, f.read_bytes(), None)
    return out, dict(passed)


def find_links(target: Path, candidates: List[Dict[str, Any]]
               ) -> Tuple[Dict[str, List[str]], Dict[str, Dict[str, Any]]]:
    """What the prose links to: files in the target, by the documents that
    link to them, and outside hosts, with the number of links and the
    documents they are in. Archive members are read for hosts only."""
    linked: Dict[str, List[str]] = {}
    hosts: Dict[str, Dict[str, Any]] = {}
    for c in candidates:
        raw = c["raw"]
        for url in _URL.findall(raw):
            host = url.split("/")[2].lower()
            h = hosts.setdefault(host, {"links": 0, "in": []})
            h["links"] += 1
            if c["path"] not in h["in"]:
                h["in"].append(c["path"])
        if c["archive"]:
            continue
        base = (target / c["path"]).parent
        for ref in _MD_LINK.findall(raw) + _HREF.findall(raw):
            ref = ref.split("#")[0].split("?")[0]
            if not ref or "://" in ref or ref.startswith(("mailto:", "/", "data:")):
                continue
            try:
                dest = (base / ref).resolve().relative_to(target.resolve())
            except ValueError:
                continue
            if (target / dest).is_file() and str(dest) != c["path"]:
                linked.setdefault(str(dest), [])
                if c["path"] not in linked[str(dest)]:
                    linked[str(dest)].append(c["path"])
    return linked, hosts


def hosting_metadata(target: Path) -> Optional[Dict[str, Any]]:
    """The repository's description, homepage, topics and wiki flag from its
    host, when the origin is on GitHub. None when there is no such origin or
    the host does not answer; the record then says it was not read."""
    try:
        r = subprocess.run(["git", "-C", str(target), "remote", "get-url", "origin"],
                           capture_output=True, text=True, timeout=10)
        m = re.search(r"github\.com[:/]([^/\s]+)/([^/\s]+?)(?:\.git)?\s*$", r.stdout or "")
        if not m:
            return None
        url = f"https://api.github.com/repos/{m.group(1)}/{m.group(2)}"
        with urllib.request.urlopen(url, timeout=20) as resp:
            d = json.load(io.TextIOWrapper(resp, encoding="utf-8"))
        return {"repository": d.get("full_name"), "description": d.get("description"),
                "homepage": d.get("homepage"), "topics": d.get("topics") or [],
                "wiki": bool(d.get("has_wiki"))}
    except Exception as e:                                     # noqa: BLE001
        logger.warning("hosting metadata for %s: %s", target, e)
        return None


# ---- the model's part -------------------------------------------------------

def schema() -> Dict[str, Any]:
    kind = {"type": "string", "enum": list(KINDS)}
    part = {"type": "object", "properties": {
        "lines": {"type": "array", "items": {"type": "integer", "minimum": 1},
                  "minItems": 2, "maxItems": 2},
        "kind": kind, "says": {"type": "string"}},
        "required": ["lines", "kind", "says"]}
    return {"type": "object", "properties": {
        "kind": kind, "parts": {"type": "array", "items": part},
        "reason": {"type": "string"}},
        "required": ["kind", "parts", "reason"]}


def decide_kind(backend, c: Dict[str, Any], linked_from: List[str],
                max_tokens: int = 4096) -> Dict[str, Any]:
    """One emission under SORTING.md for one candidate."""
    lines = c["text"].splitlines()
    shown, words = [], 0
    for n, line in enumerate(lines, 1):
        shown.append(f"{n:>4}|{line}")
        words += len(line.split())
        if words >= TEXT_WORDS:
            shown.append(f"[the file continues: {c['words']} words in all, "
                         f"{TEXT_WORDS} shown]")
            break
    where = (f"inside the archive `{c['archive']}` in the repository" if c["archive"]
             else "in the repository")
    links = ("Documents in the materials that link to it: "
             + ", ".join(f"`{x}`" for x in linked_from) + ".") if linked_from else \
        "No document in the materials links to it."
    user = (f"The file: `{c['path']}`, found {where}. {links}\n\n"
            f"Its text, with line numbers:\n\n" + "\n".join(shown) + "\n\n"
            f"Emit the answer per SORTING.md §5.")
    out = emit(types.SimpleNamespace(backend=backend), load_workflow(METHOD_PATH),
               user, schema(), max_tokens)
    obj = out.get("obj") if isinstance(out.get("obj"), dict) else {}
    kind = obj.get("kind") if obj.get("kind") in KINDS else None
    parts = [p for p in obj.get("parts") or []
             if isinstance(p, dict) and p.get("kind") in KINDS and p.get("kind") != kind]
    return {"kind": kind, "parts": parts, "reason": str(obj.get("reason") or "").strip(),
            "parse": out.get("parse"), "parse_error": out.get("parse_error")}


# ---- the proposal -----------------------------------------------------------

def _slug(path: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "_", path).strip("_")


def source_path(row: Dict[str, Any]) -> str:
    """The claim-source path a description becomes: itself when the runner
    can read it as it is, else the file its text is extracted to."""
    ext = Path(row["path"]).suffix.lower()
    if row["archive"] or ext in (".html", ".htm", ".pdf"):
        origin = (row["archive"] + "_" if row["archive"] else "") + row["path"]
        return f"{EXTRACTED}/{_slug(origin)}.md"
    return row["path"]


def estimated_claims(words: int) -> int:
    return max(1, round(words / WORDS_PER_CLAIM))


def propose(rows: List[Dict[str, Any]]) -> Dict[str, List[str]]:
    """The two lists, from the kinds. Every description in the target is a
    proposed claim source and is excluded as evidence, whether or not a person
    keeps it as a claim source. An archive member cannot be excluded by path
    and is not cited by path either. A file whose larger part is an
    instrument stays evidence; its descriptive parts are named in the record
    for the brief."""
    sources = [source_path(r) for r in rows if r["kind"] == "description"]
    excludes = [r["path"] for r in rows if r["kind"] == "description" and not r["archive"]]
    return {"claim_sources": sources, "evidence_excludes": excludes + [EXTRACTED + "/"]}


def _words(text: str) -> List[str]:
    """Words, with markdown markers and punctuation standing alone left out,
    so two renderings of one text count the same."""
    return re.findall(r"\w[\w'’.-]*", text)


def _visible_words(name: str, data: bytes) -> Optional[int]:
    """The words a reader sees in an HTML file's kept regions, counted
    without the extractor, as the figure its output is checked against. None
    for other types: there is no second reading of a PDF to compare with."""
    if Path(name).suffix.lower() not in (".html", ".htm"):
        return None
    from bs4 import BeautifulSoup
    soup = BeautifulSoup(data.decode("utf-8", "replace"), "html.parser")
    head = (soup.title.get_text(" ") if soup.title else "") + " "
    meta = soup.find("meta", attrs={"name": "description"})
    head += (meta.get("content") or "") if meta else ""
    root = soup.find("main") or soup.body or soup
    for el in root.find_all(["script", "style", "nav", "header", "footer", "video",
                             "audio", "form", "noscript", "template"]):
        el.decompose()
    return len(_words(head + " " + root.get_text(" ")))


def _dropped_regions(html: str) -> str:
    from bs4 import BeautifulSoup
    soup = BeautifulSoup(html, "html.parser")
    bits = []
    for tag in ("header", "nav", "footer"):
        for el in soup.find_all(tag):
            t = " ".join(el.get_text(" ").split())
            if t and t not in bits:
                bits.append(t)
    return " / ".join(bits)


def render(sel: Dict[str, Any]) -> str:
    """SELECTION.md, for reading. selection.json is the record."""
    L: List[str] = [f"# Selection record: `{sel['engagement']}`", ""]
    conf = sel.get("confirmed")
    L += [f"Status: **{'CONFIRMED ' + conf['at'] + ' by ' + str(conf['by']) if conf else 'PROPOSED ' + sel['at'] + ', not yet confirmed'}**. "
          f"Prepared by `workflowsv2/materials_sorting/runner.py` with {sel['model']}; "
          f"the procedure is `docs/claim-source-selection.md`.", ""]
    L += ["## 1. What intake and the engagement said", "",
          "The brief, whole:", "", "```", sel["intake_words"] or "(no brief)", "```", "",
          "`claim_sources:` in engagement.yaml before this step: "
          + (", ".join(f"`{x}`" for x in sel["before"]["claim_sources"]) or "none"), "",
          "`evidence_excludes:` before this step: "
          + (", ".join(f"`{x}`" for x in sel["before"]["evidence_excludes"]) or "none")
          + ("" if sel["before"]["excludes_explicit"] else " (defaulted to the claim sources)"), ""]
    L += ["## 2. Files that hold prose, and their kind", "",
          f"Target at commit {sel['target_rev'] or '(not a repository)'}. "
          f"{len(sel['rows'])} files read. Passed over without reading, by type: "
          + (", ".join(f"{k} {v}" for k, v in sorted(sel["passed_over"].items())) or "none") + ".", "",
          "| File | Words | Linked from | Kind | Claim source | Claims, estimated | Excluded as evidence | Reason |",
          "|---|---|---|---|---|---|---|---|"]
    prop = sel["proposal"]
    for r in sel["rows"]:
        name = f"`{r['path']}`" + (f" in `{r['archive']}`" if r["archive"] else "")
        src = f"`{source_path(r)}`" if source_path(r) in prop["claim_sources"] else ""
        exc = "yes" if r["path"] in prop["evidence_excludes"] else ""
        kind = r["kind"] or "NOT DECIDED (the model's answer did not parse)"
        if r["parts"]:
            kind += ", mixed"
        est = f"about {estimated_claims(r['words'])}" if src else ""
        L.append(f"| {name} | {r['words']} | {', '.join(r['linked_from']) or ''} | {kind} | "
                 f"{src} | {est} | {exc} | {r['reason'].replace('|', '/')} |")
    mixed = [r for r in sel["rows"] if r["parts"]]
    if mixed:
        L += ["", "### Mixed files", "",
              "Parts whose kind differs from the file's. Where a file kept as evidence "
              "has a part that is a description, the brief should name that part as the "
              "seller's own statement. Decide each one."]
        for r in mixed:
            for p in r["parts"]:
                L.append(f"- `{r['path']}` lines {p['lines'][0]}-{p['lines'][1]}, "
                         f"{p['kind']}: {p['says']}")
    near = [r for r in sel["rows"] if r["words"] < MIN_WORDS]
    if near:
        L += ["", "Near-empty files (under " + str(MIN_WORDS) + " words), not sent to the model: "
              + ", ".join(f"`{r['path']}`" for r in near) + "."]
    L += ["", "## 3. Outside the target: the client's choice", "",
          "Nothing here was fetched. Each is the seller's text a buyer could have "
          "read; including one adds its claims to the review and to the fee.", ""]
    md = sel.get("hosting")
    if md:
        L += [f"- Hosting description of `{md['repository']}`: \"{md['description']}\"; "
              f"homepage {md['homepage'] or 'none'}; topics: {', '.join(md['topics']) or 'none'}; "
              f"wiki {'enabled' if md['wiki'] else 'not enabled'}."]
    else:
        L += ["- Hosting description: not read (no GitHub origin, or the host did not answer)."]
    # Hosts are listed when a description links to them; which of them are
    # the seller's own is for the person reading to say.
    described = {r["path"] for r in sel["rows"] if r["kind"] == "description"}
    hosts = sorted(((host, h) for host, h in sel["hosts"].items() if described & set(h["in"])),
                   key=lambda kv: -kv[1]["links"])
    for host, h in hosts[:MAX_HOSTS]:
        L.append(f"- `{host}`: {h['links']} link(s), in {', '.join('`' + x + '`' for x in h['in'][:4])}"
                 + (" and others" if len(h["in"]) > 4 else ""))
    if len(hosts) > MAX_HOSTS:
        L.append(f"- and {len(hosts) - MAX_HOSTS} more hosts with fewer links, in selection.json")
    L += ["", "## 4. Web pages: what extraction drops", ""]
    pages = [r for r in sel["rows"] if r.get("dropped")]
    if not pages:
        L.append("No HTML file is proposed as a claim source.")
    for r in pages:
        L.append(f"- `{r['path']}`: kept the title, the meta description and the main "
                 f"content. Dropped header, navigation and footer text: \"{r['dropped']}\"")
    words = {source_path(r): r["words"] for r in sel["rows"]}
    est = {x: estimated_claims(words[x]) for x in prop["claim_sources"] if x in words}
    L += ["", "## 5. Proposed lists, and how they differ from before", "",
          "Every description is excluded as evidence. Which descriptions are claim "
          "sources is a choice: each one adds its claims to the review, to its "
          "length and to the fee. The estimates are of what enumeration will list, "
          f"at one claim per {WORDS_PER_CLAIM} words; claims repeated between documents "
          "are counted in each and are folded after enumeration.", "",
          "Claim sources:",
          *[f"- `{x}`" + (f": about {est[x]} claims" if x in est else "") for x in prop["claim_sources"]],
          f"- in all: about {sum(est.values())} claims" if est else "- in all: nothing to estimate", "",
          "Evidence excludes:", *[f"- `{x}`" for x in prop["evidence_excludes"]], ""]
    for key, label in (("claim_sources", "claim sources"), ("evidence_excludes", "evidence excludes")):
        new = [x for x in prop[key] if x not in sel["before"][key]]
        gone = [x for x in sel["before"][key] if x not in prop[key]]
        L.append(f"Not named before, proposed now ({label}): " + (", ".join(f"`{x}`" for x in new) or "none") + ".")
        L.append(f"Named before, not proposed now ({label}): " + (", ".join(f"`{x}`" for x in gone) or "none")
                 + ". These are kept unless removed at confirmation.")
    if conf:
        L += ["", "## 6. Confirmation", "",
              f"Confirmed by {conf['by']} at {conf['at']}.", "",
              "Claim sources as confirmed:", *[f"- `{x}`" for x in conf["claim_sources"]], "",
              "Evidence excludes as confirmed:", *[f"- `{x}`" for x in conf["evidence_excludes"]], "",
              "Changed from the proposal: " + (", ".join(conf["changes"]) or "nothing") + ".", "",
              "Extraction check (words in the extracted file, less its two heading labels, "
              "against the words a reader sees in the original, counted separately):",
              *([f"- `{e['source']}` from `{e['from']}`: {e['words']} extracted, "
                 + (f"{e['original_words']} in the original" if e["original_words"] is not None
                    else "no second count for this file type; read it")
                 for e in conf["extracted"]] or ["- nothing needed extracting"])]
    return "\n".join(L) + "\n"


# ---- the two commands -------------------------------------------------------

def sort_materials(eng_dir: Path, model_yaml: Path) -> Dict[str, Any]:
    from workflowsv2.claims_audit.runner import git_rev
    target = state.target_dir(eng_dir)
    if not target.is_dir():
        raise SystemExit(f"{eng_dir.name}: no materials at {target}")
    backend = backend_from_model(model_yaml)
    candidates, passed = list_candidates(target)
    linked, hosts = find_links(target, candidates)
    rows = []
    for c in candidates:
        row = {"path": c["path"], "archive": c["archive"], "words": c["words"],
               "linked_from": linked.get(c["path"], []) if not c["archive"] else [],
               "kind": "neither", "parts": [], "reason": "near-empty", "parse": None}
        if c["words"] >= MIN_WORDS:
            logger.info("reading %s (%d words)", c["path"], c["words"])
            row.update(decide_kind(backend, c, row["linked_from"]))
        if row["kind"] == "description" and Path(c["path"]).suffix.lower() in (".html", ".htm"):
            row["dropped"] = _dropped_regions(c["raw"])
        rows.append(row)
    # The brief carries what intake said about the claim sources. It is short
    # and its wording varies, so the record shows it whole.
    brief = eng_dir / "brief.md"
    intake_words = brief.read_text(encoding="utf-8").strip() if brief.is_file() else ""
    before_sources = state.claim_sources(eng_dir)
    proposal = propose(rows)
    for key, had in (("claim_sources", before_sources),
                     ("evidence_excludes", state.evidence_excludes(eng_dir))):
        proposal[key] += [x for x in had if x not in proposal[key]]
    sel = {"engagement": eng_dir.name, "at": state.stamp(),
           "model": backend.resolved_model(), "target_rev": git_rev(target),
           "intake_words": intake_words,
           "before": {"claim_sources": before_sources,
                      "evidence_excludes": state.evidence_excludes(eng_dir),
                      "excludes_explicit": "evidence_excludes" in state._engagement_yaml(eng_dir)},
           "passed_over": passed, "rows": rows, "hosts": hosts,
           "hosting": hosting_metadata(target), "proposal": proposal, "confirmed": None}
    write(eng_dir, sel)
    state.set_stage(eng_dir, SORTING, "proposed", "job")
    return sel


def write(eng_dir: Path, sel: Dict[str, Any]) -> None:
    d = eng_dir / SORTING
    d.mkdir(exist_ok=True)
    atomic_write_text(d / "selection.json", json.dumps(sel, indent=1, ensure_ascii=False))
    atomic_write_text(d / "SELECTION.md", render(sel))


def load(eng_dir: Path) -> Optional[Dict[str, Any]]:
    f = eng_dir / SORTING / "selection.json"
    return json.loads(f.read_text(encoding="utf-8")) if f.is_file() else None


def confirm(eng_dir: Path, by: str, claim_sources: Optional[List[str]] = None,
            evidence_excludes: Optional[List[str]] = None) -> Dict[str, Any]:
    """Apply the proposal, or the lists given in its place, to engagement.yaml;
    extract the claim sources that are not plain text; record who confirmed
    and what they changed; mark the stage."""
    sel = load(eng_dir)
    if sel is None:
        raise SystemExit(f"{eng_dir.name}: the materials have not been sorted yet")
    prop = sel["proposal"]
    sources = list(claim_sources) if claim_sources is not None else list(prop["claim_sources"])
    excludes = list(evidence_excludes) if evidence_excludes is not None else list(prop["evidence_excludes"])
    target = state.target_dir(eng_dir)
    by_source = {source_path(r): r for r in sel["rows"]}
    extracted = []
    for s in sources:
        row = by_source.get(s)
        if row is None or s == row["path"]:
            if not (target / s).is_file():
                raise SystemExit(f"claim source '{s}' is not a file in the target")
            continue
        if row["archive"]:
            with zipfile.ZipFile(target / row["archive"]) as z:
                data = z.read(row["path"])
        else:
            data = (target / row["path"]).read_bytes()
        text = _text_of(row["path"], data) or ""
        out = target / s
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text.rstrip() + "\n", encoding="utf-8")
        extracted.append({"source": s, "from": (row["archive"] + "!" if row["archive"] else "") + row["path"],
                          "words": len(_words(text.replace("Page title: ", "", 1)
                                              .replace("Page description: ", "", 1))),
                          "original_words": _visible_words(row["path"], data)})
    changes = ([f"claim source added: {x}" for x in sources if x not in prop["claim_sources"]]
               + [f"claim source removed: {x}" for x in prop["claim_sources"] if x not in sources]
               + [f"exclude added: {x}" for x in excludes if x not in prop["evidence_excludes"]]
               + [f"exclude removed: {x}" for x in prop["evidence_excludes"] if x not in excludes])
    state.update_engagement(eng_dir, claim_sources=sources, evidence_excludes=excludes)
    sel["confirmed"] = {"by": by, "at": state.stamp(), "claim_sources": sources,
                        "evidence_excludes": excludes, "changes": changes,
                        "extracted": extracted}
    write(eng_dir, sel)
    state.set_stage(eng_dir, SORTING, "confirmed", by)
    return sel


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engagement", required=True)
    ap.add_argument("--model", type=Path, default=None, help="model yaml, for sorting")
    ap.add_argument("--confirm", action="store_true")
    ap.add_argument("--by", default=None, help="who confirms")
    ap.add_argument("--claim-source", action="append", default=None,
                    help="with --confirm: the claim sources, in place of the proposal's")
    ap.add_argument("--exclude", action="append", default=None,
                    help="with --confirm: the evidence excludes, in place of the proposal's")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    eng_dir = state.ENGAGEMENTS / args.engagement
    if not eng_dir.is_dir():
        raise SystemExit(f"no engagement '{args.engagement}'")
    if args.confirm:
        if not args.by:
            raise SystemExit("--confirm needs --by <who>")
        sel = confirm(eng_dir, args.by, args.claim_source, args.exclude)
        print(f"confirmed: {len(sel['confirmed']['claim_sources'])} claim sources, "
              f"{len(sel['confirmed']['evidence_excludes'])} excludes")
    else:
        if not args.model:
            raise SystemExit("sorting needs --model <model yaml>")
        sel = sort_materials(eng_dir, args.model)
        kinds = Counter(r["kind"] or "not decided" for r in sel["rows"])
        print(f"sorted {len(sel['rows'])} files: " + ", ".join(f"{k} {v}" for k, v in kinds.items()))
    print(eng_dir / SORTING / "SELECTION.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
