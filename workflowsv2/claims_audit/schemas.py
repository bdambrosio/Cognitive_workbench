"""The audit output schema, and the checks that run after it is parsed.

METHOD §13 states the contract; this file is the machine half of it. The two
must agree, and `lint_workflow.check_schema_vocab` is what makes them.

WHAT THE SCHEMA ENFORCES AND WHAT PYTHON ENFORCES. The schema carries shape:
which fields exist, which are required, and the two closed vocabularies
(`verdict`, `unresolved_because`). Everything that needs to look at the target
— does this document exist, is this line range real, does this quote appear
where it says — is checked here, after parsing, against the corpus.

That division is deliberate and it is the same one `citations.py` was written
for: "an LLM asked whether a marker appears is slower, dearer and less reliable
than a substring test." Two consequences, both of which look like omissions
until you know the reason:

  - **`document` is a plain string, not an enum of the corpus filenames.** An
    enum would make a nonexistent document unrepresentable at decode time,
    which is genuinely stronger — for nine documents. A real target is 1,141
    files and the enum becomes the schema. `check_output` catches the same
    defect with a better message, at every target size.

  - **Evidence is one flat object with a `form` discriminator, not a `oneOf`
    union.** The union is the faithful model of METHOD §7 and it decodes
    correctly on the local route — probed 2026-09-01, four runs, schema-valid
    every time. It was still rejected. `oneOf` with per-branch
    `additionalProperties: false` is the construct most likely to vary between
    constrained-decoding backends, and requiring it would narrow the candidate
    pool to catch a defect `check_output` already catches. Widening that pool
    is what this refactor is for.

TRUNCATION IS THE FAILURE MODE THAT MATTERS. Under constrained decoding a
generation cut at `max_tokens` yields invalid JSON rather than a short report,
so the loss is total instead of partial. `salvage_findings` recovers the
complete elements of a cut findings array, after `claims._salvage_truncated_claims`,
which is sound for the same reason: array elements here are independent.
"""
from __future__ import annotations

import json
import logging
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

logger = logging.getLogger("claims_audit.schemas")

# --------------------------------------------------------------------------
# The closed vocabularies. METHOD §6 and §8 are the source; the linter checks
# that these lists and those tables say the same thing.
# --------------------------------------------------------------------------
VERDICTS: Tuple[str, ...] = (
    "real", "real_with_caveat", "partial", "contradicted", "unverifiable")

#: Verdicts that require `adjudication.gap` (METHOD §13). `real` has nothing
#: beside it; `unverifiable` uses `unresolved_because` instead.
GAP_REQUIRED: Tuple[str, ...] = ("real_with_caveat", "partial", "contradicted")

#: The verdicts that report something wrong with the target. Defined HERE,
#: beside the vocabulary, because which verdicts are adverse is a property of
#: the vocabulary rather than of whoever counts them.
#:
#: `unverifiable` IS DELIBERATELY NOT ADVERSE. It reports a limit of the
#: supplied materials, not a defect in the target — `gap-analysis.md` calls
#: inability to obtain evidence "a scope limitation, not an adverse finding".
#: A reader may still care that a fifth of the claims went unsettled; that is
#: reported as its own figure and never folded into this one.
#:
#: NOTHING HERE COUNTS THEM. The audit does not publish a figure that reads as
#: a score on its own work — that is the audit's account of itself, and the
#: review exists not to trust it. `audit_review` computes the counts, from the
#: artifacts, using this definition.
ADVERSE_VERDICTS: Tuple[str, ...] = ("partial", "contradicted")

UNRESOLVED_BECAUSE: Tuple[str, ...] = (
    "not_in_the_materials", "present_but_not_readable", "outside_the_materials",
    "not_examined")

#: METHOD §8: the disposition for an `unverifiable` claim whose searches named
#: a file that was never opened. The runner chases these before delivery
#: (`claims_audit/runner.py`, the chase loop); what remains is reported as a
#: count of files named and not opened, never folded into the other three.
NOT_EXAMINED = "not_examined"

EVIDENCE_FORMS: Tuple[str, ...] = ("citation", "derived", "search")

#: METHOD §5: whom a claim concerns. `seller` marks an assertion the supplied
#: materials are not expected to reach, so the reader knows why it went
#: unsettled.
ABOUT: Tuple[str, ...] = ("target", "seller", "document")

SEARCH_KINDS: Tuple[str, ...] = ("lexical", "structural")

#: Fields each evidence form requires, per METHOD §7. The schema requires only
#: `form`; this table is enforced after parsing.
FORM_FIELDS: Dict[str, Tuple[str, ...]] = {
    "citation": ("document", "lines", "quote", "shows"),
    "derived":  ("basis", "derivation", "consequence"),
    "search":   ("kind", "performed", "result"),
}

_LINES = {"type": "array", "items": {"type": "integer"},
          "minItems": 2, "maxItems": 2}


def surface_schema() -> Dict[str, Any]:
    """Phase one, one section of the claim source: METHOD §5 and §13.

    Emitted before any evidence is gathered and frozen when the last section
    lands. The freeze is a control, not a convenience — enumerating while
    adjudicating means choosing what counts as a claim with the verdicts
    already in view.

    NO `id`. Ids are assigned by the runner in document order as sections
    complete (`assemble_surface`), so a model enumerating section four cannot
    collide with section one. `restates` names an id the model was given.
    """
    claim = {"type": "object", "properties": {
        "quote": {"type": "string"}, "lines": _LINES,
        "statement": {"type": "string"},
        "about": {"enum": list(ABOUT)},
        "restates": {"type": "integer", "minimum": 1}},
        "required": ["quote", "lines", "statement", "about"]}
    return {"type": "object", "properties": {
        "claim_source": {"type": "string"},
        "claims": {"type": "array", "items": claim},
        "not_completed": {"type": "string"}},
        "required": ["claim_source", "claims"]}


#: Section sizing for enumeration, in characters. A section smaller than
#: MIN is joined to the next; one larger than MAX is cut at a blank line.
SECTION_MIN = 1_500
SECTION_MAX = 12_000

_HEADING = re.compile(r"^#{1,6}\s")
_FENCE = re.compile(r"^\s*(```|~~~)")


def split_sections(text: str, minimum: int = SECTION_MIN,
                   maximum: int = SECTION_MAX) -> List[Tuple[int, int]]:
    """The claim source as sections, each a 1-based inclusive line range.

    Cut at every markdown heading outside a code fence, so a table stays
    with its heading and a `# comment` inside a shell block does not start a
    section. Short sections are joined forward to `minimum`; a section over
    `maximum` is cut at the last blank line before the limit. A document
    with no heading is one section, which is what enumeration was before.
    """
    lines = text.splitlines()
    if not lines:
        return []
    cuts, in_fence = [1], False
    for n, line in enumerate(lines, 1):
        if _FENCE.match(line):
            in_fence = not in_fence
        elif not in_fence and _HEADING.match(line) and n > 1:
            cuts.append(n)
    spans = [(a, b - 1) for a, b in zip(cuts, cuts[1:] + [len(lines) + 1])]

    def size(a: int, b: int) -> int:
        return sum(len(l) + 1 for l in lines[a - 1:b])

    merged: List[Tuple[int, int]] = []
    for a, b in spans:
        if merged and size(*merged[-1]) < minimum:
            merged[-1] = (merged[-1][0], b)
        else:
            merged.append((a, b))
    if len(merged) > 1 and size(*merged[-1]) < minimum:
        _, b = merged.pop()
        merged[-1] = (merged[-1][0], b)
    out: List[Tuple[int, int]] = []
    for a, b in merged:
        while size(a, b) > maximum:
            cut, used = None, 0
            for n in range(a, b + 1):
                used += len(lines[n - 1]) + 1
                if used > maximum:
                    break
                if not lines[n - 1].strip():
                    cut = n
            if cut is None or cut <= a:
                break
            out.append((a, cut))
            a = cut + 1
        out.append((a, b))
    return out


def assemble_surface(claim_source: str, sections: Sequence[Dict[str, Any]]
                     ) -> Dict[str, Any]:
    """The frozen surface from per-section emissions, ids in document order.

    A claim carrying `restates` is not a claim of its own: its quote and
    lines join the named claim's `locations`. A `restates` that names no
    assigned id is kept as a claim and reported by `check_surface`, so the
    assertion is not lost to a bad reference.

    A DUPLICATE WITHIN ONE SECTION FOLDS TOO. `restates` can only name a
    claim from an earlier section, because ids are assigned when a section
    closes; a model that splits one line into several claims and quotes the
    whole line each time has no way to name the repeat. One README line
    came back as seven claims with one quote (2026-09-02). The test is
    string equality on the normalised quote, the rule the merge already
    uses across claim sources, and the second copy becomes a location on
    the first — its `statement` is kept in `statements` so a split the
    model meant is not lost.
    """
    claims: List[Dict[str, Any]] = []
    by_id: Dict[int, Dict[str, Any]] = {}
    not_completed = []
    for sec in sections:
        if sec.get("not_completed"):
            not_completed.append(sec["not_completed"])
        seen_in_section: Dict[str, Dict[str, Any]] = {}
        for c in sec.get("claims") or []:
            loc = {"quote": c.get("quote"), "lines": c.get("lines")}
            target = by_id.get(c.get("restates")) if c.get("restates") else None
            if target is not None:
                target.setdefault("locations", []).append(loc)
                continue
            key = _norm(c.get("quote") or "")
            twin = seen_in_section.get(key) if key else None
            if twin is not None:
                twin.setdefault("locations", []).append(loc)
                st = c.get("statement")
                if st and st != twin.get("statement") \
                        and st not in twin.get("statements", []):
                    twin.setdefault("statements", []).append(st)
                continue
            row = {"id": len(claims) + 1, "quote": c.get("quote"),
                   "lines": c.get("lines"), "statement": c.get("statement"),
                   "about": c.get("about")}
            if c.get("restates"):
                row["restates"] = c["restates"]      # unresolved; reported
            claims.append(row)
            by_id[row["id"]] = row
            if key:
                seen_in_section[key] = row
    out: Dict[str, Any] = {"claim_source": claim_source, "claims": claims}
    if not_completed and not claims:
        out["not_completed"] = "; ".join(not_completed)
    return out


def locations(claim: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Every place a claim is made: its own quote and lines, then any
    restatement `assemble_surface` folded in."""
    return ([{"quote": claim.get("quote"), "lines": claim.get("lines")}]
            + list(claim.get("locations") or []))


def check_surface(obj: Dict[str, Any], corpus: Path,
                  claim_source: str) -> Dict[str, Any]:
    """The frozen surface, against the document it was read from."""
    problems: List[str] = []
    body: List[str] = []
    src = corpus / claim_source
    if src.is_file():
        body = src.read_text(encoding="utf-8", errors="replace").splitlines()
    else:
        problems.append(f"claim source {claim_source!r} is not in the materials")

    claims = obj.get("claims")
    if not isinstance(claims, list):
        return {"ok": False, "problems": ["`claims` is not a list"],
                "figures": {}}
    incomplete = obj.get("not_completed")
    if incomplete and claims:
        problems.append("`not_completed` is set and claims were emitted; "
                        "METHOD §4 requires an empty list")
    if not incomplete and not claims:
        problems.append("no claims and no `not_completed` — METHOD §4 requires "
                        "one or the other")

    seen_id, seen_q = {}, {}
    ids = {c.get("id") for c in claims}
    restated = 0
    for i, c in enumerate(claims, 1):
        w = f"claim {c.get('id', f'#{i}')}"
        cid = c.get("id")
        if cid in seen_id:
            problems.append(f"{w}: id {cid} is used twice (also at position "
                            f"{seen_id[cid]}) — findings refer to claims by id")
        else:
            seen_id[cid] = i
        if c.get("about") not in ABOUT:
            problems.append(f"{w}: `about` {c.get('about')!r} is not one of "
                            f"METHOD §5's")
        # A subclaim (DECOMPOSE.md) names the claim it was read from; the
        # parent has to be in the surface.
        if c.get("implied_by") is not None and c.get("implied_by") not in ids:
            problems.append(f"{w}: implied_by {c.get('implied_by')!r} names no claim "
                            f"in the surface")
        if c.get("restates"):
            problems.append(f"{w}: restates claim {c['restates']}, which "
                            f"{'is not' if c['restates'] not in ids else 'was not yet'}"
                            f" assigned when this section was enumerated; kept "
                            f"as a claim of its own")
        restated += len(c.get("locations") or [])
        for j, loc in enumerate(locations(c)):
            lw = w if j == 0 else f"{w} location {j + 1}"
            q = _norm(loc.get("quote"))
            if not q:
                problems.append(f"{lw}: no quote — METHOD §5 makes the "
                                f"verbatim quote the claim's identity")
            elif j == 0 and q in seen_q:
                problems.append(f"{w}: same quote as claim {seen_q[q]}")
            elif j == 0:
                seen_q[q] = cid
            lines = loc.get("lines")
            if body and isinstance(lines, list) and len(lines) == 2 \
                    and all(isinstance(n, int) for n in lines):
                lo, hi = lines
                if lo < 1 or hi < lo or hi > len(body):
                    problems.append(f"{lw}: lines {lo}-{hi} against a "
                                    f"{len(body)}-line claim source")
                elif q and q not in _norm("\n".join(body[lo - 1:hi])):
                    where = ("elsewhere in the document"
                             if q in _norm("\n".join(body)) else "nowhere in it")
                    problems.append(f"{lw}: quote is not at lines {lo}-{hi}; "
                                    f"it is {where}")
            else:
                problems.append(f"{lw}: lines {lines!r} is not a start and end")

    return {"ok": not problems, "problems": problems,
            "figures": {"claims": len(claims), "restatements": restated,
                        "about": _tally(claims, "about"),
                        "not_completed": bool(incomplete)}}


def _tally(rows: Sequence[Dict[str, Any]], field: str) -> Dict[str, int]:
    out: Dict[str, int] = {}
    for r in rows:
        out[r.get(field)] = out.get(r.get(field), 0) + 1
    return out


def audit_schema() -> Dict[str, Any]:
    """The response schema for the emission call, per METHOD §13.

    Takes no arguments on purpose. Nothing target-specific belongs in a
    schema that a decoder has to compile — see the module docstring.
    """
    basis = {"type": "object", "properties": {
        "document": {"type": "string"}, "lines": _LINES,
        "quote": {"type": "string"}},
        "required": ["document", "lines", "quote"]}
    evidence = {"type": "object", "properties": {
        "form": {"enum": list(EVIDENCE_FORMS)},
        # citation
        "document": {"type": "string"}, "lines": _LINES,
        "quote": {"type": "string"}, "shows": {"type": "string"},
        # derived
        "basis": {"type": "array", "items": basis},
        "derivation": {"type": "string"}, "consequence": {"type": "string"},
        # search
        "kind": {"enum": list(SEARCH_KINDS)},
        "performed": {"type": "string"}, "result": {"type": "string"},
        "candidates": {"type": "array", "items": {"type": "string"}}},
        "required": ["form"]}
    adjudication = {"type": "object", "properties": {
        "verdict": {"enum": list(VERDICTS)},
        "gap": {"type": "string"},
        "unresolved_because": {"enum": list(UNRESOLVED_BECAUSE)}},
        "required": ["verdict"]}
    finding = {"type": "object", "properties": {
        "claim_id": {"type": "integer", "minimum": 1},
        "adjudication": adjudication,
        "evidence": {"type": "array", "items": evidence, "minItems": 1},
        "correction": {"type": "string"}},
        "required": ["claim_id", "adjudication", "evidence"]}
    return {"type": "object", "properties": {
        "claim_source": {"type": "string"},
        "findings": {"type": "array", "items": finding},
        "unclaimed": {"type": "array", "items": {"type": "object", "properties": {
            "note": {"type": "string"}, "evidence": evidence},
            "required": ["note", "evidence"]}},
        "questions": {"type": "array", "items": {"type": "string"}},
        "not_completed": {"type": "string"}},
        "required": ["claim_source", "findings"]}


# --------------------------------------------------------------------------
# Parsing
# --------------------------------------------------------------------------

def salvage_findings(text: str) -> Optional[Dict[str, Any]]:
    """Recover the complete findings of a truncated emission, or None.

    Schema-aware, and local to this module for the same reason
    `claims._salvage_truncated_claims` is local to its own: dropping the
    trailing partial element is only sound because the elements are
    independent, which is true of findings and is not true in general.

    Returns the object with whatever findings parsed whole. The caller must
    record that salvage fired — a salvaged run is not a clean one.
    """
    start = text.find("{")
    if start < 0:
        return None
    src = text[start:]
    m = re.search(r'"findings"\s*:\s*\[', src)
    if not m:
        return None
    head, out, depth, in_str, esc, elem = src[:m.end()], [], 0, False, False, ""
    for ch in src[m.end():]:
        if esc:
            elem += ch; esc = False; continue
        if ch == "\\" and in_str:
            elem += ch; esc = True; continue
        if ch == '"':
            in_str = not in_str
        if not in_str:
            if ch == "{":
                depth += 1
            elif ch == "}":
                depth -= 1
                if depth == 0:
                    elem += ch
                    try:
                        out.append(json.loads(elem))
                    except Exception:      # a cut element; stop, keep the rest
                        break
                    elem = ""
                    continue
            elif ch == "]" and depth == 0:
                break
            elif depth == 0 and ch in ", \n\r\t":
                continue
        elem += ch
    if not out:
        return None
    src_head = json.loads(head[:head.rfind('"findings"')].rstrip().rstrip(",") + "}")
    src_head["findings"] = out
    return src_head


# --------------------------------------------------------------------------
# The checks, after parsing, against the target
# --------------------------------------------------------------------------

#: Markdown decoration a quote routinely loses on the way out of a document.
#: NOT a content filter — every one of these is presentation, and a quote that
#: differs only by them is the same text.
_DECORATION = re.compile(r"[*_`]+|^\s*[-*+]\s+|^\s*\d+\.\s+", re.M)


def _norm(s: str) -> str:
    """Compare quotes the way a reader would: ignoring whitespace and markdown.

    WHITESPACE, because a quote copied out of a document routinely differs
    from it by a line wrap and by nothing else.

    MARKDOWN, because this check's first run reported seven of twelve findings
    on doc9 as quoting text that "does not appear in that document at all",
    and every one of them was a faithful quote of `*   **Dyno:** ...` written
    down as `Dyno: ...`. The model dropped the decoration, which is what a
    person quoting that line would also do. `repair_json_string` strips the
    same characters out of structured output for the same reason.

    The risk is the other way — normalising until a wrong quote matches — so
    this removes presentation only: emphasis, code ticks, and list markers.
    Nothing that carries meaning is touched.

    BOTH SIDES MUST BE JOINED THE SAME WAY BEFORE THIS RUNS. The list-marker
    patterns are line-anchored, so a quote that kept its newlines gets them
    stripped while a source span joined with spaces does not — which reported
    a correct citation as quoting text found "nowhere in it". Callers join
    source lines with a newline.
    """
    return re.sub(r"\s+", " ", _DECORATION.sub(" ", s or "")).strip()


#: Directory names never read as materials when the target is not a git
#: worktree. `.git` holds thousands of binary objects; a dependency tree is
#: not the target. Every other directory is materials, dot-directories
#: included: `.github/workflows/` is where a repository says how it publishes
#: its images, and twelve citations into it were reported "not in the
#: materials" while the subagent had read the file (ChatterMate, 2026-09-02).
#: Files whose own name starts with a dot (`.env.example`) are materials too.
_SKIP_DIRS = (".git", "node_modules", "__pycache__")

#: A file with a NUL byte this early is not text: an image, a zip, a
#: coverage database. It is never a material, whichever view listed it.
_BINARY_PROBE_BYTES = 8192


def corpus_view(corpus: Path) -> Dict[str, Any]:
    """Which files are the materials, and how that was decided.

    THE MATERIALS ARE WHAT THE SELLER COMMITTED. When the target is inside a
    git worktree, the materials are its tracked files (`git ls-files`): an
    ignored virtual environment or build tree is not the seller's work, and
    one target (Body, 2026-09-03) had 21 tracked files under 2,260 ignored
    ones. Outside a worktree, every file under the target except those in
    `_SKIP_DIRS`. In both views a file with a NUL byte in its first 8 KB is
    dropped as binary (ChatterMate: 28 of 1,114 tracked files).

    Returns `{"view": "tracked" | "walk", "materials": [paths],
    "binary_skipped": [paths], "submodules": [paths]}`, paths relative to
    the target in POSIX form and sorted. A SUBMODULE IS A PLACE THE MATERIALS
    DO NOT REACH: a tracked entry of mode 160000 names a directory the seller
    keeps elsewhere, checked out here or not. It is listed so a candidate that
    names it can be recognised as outside the materials (ChatterMate's two
    enterprise submodules, 2026-09-03) rather than as a path that resolves to
    nothing. Its files, when it is checked out, are still not materials.
    """
    import subprocess
    out: Dict[str, Any] = {"view": "walk", "materials": [], "binary_skipped": [],
                           "submodules": []}
    if not corpus.is_dir():
        return out
    rels: List[Path] = []
    try:
        r = subprocess.run(["git", "-C", str(corpus), "ls-files", "-s", "-z"],
                           capture_output=True, timeout=30)
    except Exception as e:                                     # noqa: BLE001
        logger.warning("corpus view: git ls-files failed under %s (%s); "
                       "walking instead", corpus, e)
        r = None
    if r is not None and r.returncode == 0:
        out["view"] = "tracked"
        for entry in r.stdout.decode("utf-8", "replace").split("\0"):
            if not entry or "\t" not in entry:
                continue
            meta, name = entry.split("\t", 1)
            if meta.split(" ", 1)[0] == "160000":
                out["submodules"].append(Path(name).as_posix())
            else:
                rels.append(Path(name))
        out["submodules"].sort()
    else:
        for f in sorted(corpus.rglob("*")):
            rel = f.relative_to(corpus)
            if any(part in _SKIP_DIRS for part in rel.parts[:-1]):
                continue
            rels.append(rel)
    for rel in sorted(rels):
        f = corpus / rel
        if not f.is_file():
            continue
        try:
            with f.open("rb") as fh:
                head = fh.read(_BINARY_PROBE_BYTES)
        except OSError as e:
            logger.warning("corpus view: unreadable %s (%s)", f, e)
            continue
        (out["binary_skipped"] if b"\0" in head else out["materials"]).append(
            rel.as_posix())
    return out


def corpus_view_summary(corpus: Path) -> Dict[str, Any]:
    """The view's name and counts, with the binary files named, for a run's
    metadata and the review's statistics."""
    v = corpus_view(corpus)
    return {"view": v["view"], "materials": len(v["materials"]),
            "binary_skipped": v["binary_skipped"],
            "submodules": v["submodules"]}


def corpus_index(corpus: Path) -> Dict[str, List[str]]:
    """Every material file (`corpus_view`), keyed by its path relative to the
    target root in POSIX form, as a list of lines.

    KEYED BY RELATIVE PATH, NOT BASENAME. The first version keyed on `f.name`,
    which is exact for a nine-document fixture and wrong for a repository:
    ChatterMate has two `README.md` and several `agent.py`, and a basename
    index silently checks a quote against whichever one sorted last.
    `resolve_document` accepts a basename when it is unique, so the fixture's
    citations still resolve as written.
    """
    docs: Dict[str, List[str]] = {}
    for rel in corpus_view(corpus)["materials"]:
        try:
            docs[rel] = (corpus / rel).read_text(
                encoding="utf-8", errors="replace").splitlines()
        except OSError as e:
            logger.warning("corpus index: unreadable %s (%s)", corpus / rel, e)
    return docs


def resolve_document(docs: Dict[str, List[str]], name: Any
                     ) -> Tuple[Optional[str], Optional[str]]:
    """The index key a cited `document` names, or why it names none.

    Returns `(key, problem)`: exactly one is set. A relative path resolves
    exactly; a bare filename resolves when exactly one file has it, and is a
    problem naming the candidates when several do.
    """
    if not isinstance(name, str) or not name.strip():
        return None, f"document {name!r} is not a name"
    # A leading "./" is dropped as a prefix. `lstrip("./")` strips the
    # CHARACTERS, which turned `.gitmodules` into `gitmodules` and reported
    # every dotfile citation as not in the materials (found 2026-09-02).
    n = name.strip()
    while n.startswith("./"):
        n = n[2:]
    n = n.lstrip("/")
    if n in docs:
        return n, None
    hits = [k for k in docs if k.rsplit("/", 1)[-1] == n]
    if len(hits) == 1:
        return hits[0], None
    if len(hits) > 1:
        return None, (f"document {name!r} names {len(hits)} files "
                      f"({', '.join(hits[:4])}); cite the path")
    return None, f"document {name!r} is not in the materials"


def candidate_files(findings: Sequence[Dict[str, Any]],
                    docs: Dict[str, List[str]], read: Optional[set],
                    submodules: Sequence[str] = ()
                    ) -> Dict[Any, Dict[str, List[str]]]:
    """Per finding whose evidence records a search: the files its searches
    named in `candidates`, resolved against the corpus, and which of them
    were not opened. METHOD §8.

    `read` is the set of corpus keys the run opened (`runner.files_read`);
    None means the caller cannot say, and `unopened` is then left empty rather
    than guessed. `unresolved` holds the names that resolve to no single file,
    with the reason, so the caller can report them.

    Keyed by `claim_id`. Only findings with a `search` item are present. A
    claim of absence rests on searches as an `unverifiable` one does (METHOD
    §8, 2026-09-05), so its candidates are the same obligation.

    `outside` holds candidates that name a submodule — a place the materials
    do not reach — which is consistent with `outside_the_materials` and with
    nothing else. A candidate that names a DIRECTORY of the corpus is recorded under
    `directories` and is neither an obligation nor a problem: there is no one
    file to open, and a model naming the module where the material would sit
    is answering §8's question honestly. A name that resolves to nothing is
    a problem, as a cited document would be.
    """
    dirs = {k.rsplit("/", 1)[0] for k in docs if "/" in k}
    dirs = {d[:i] for d in dirs for i in range(len(d) + 1)
            if i == len(d) or d[i] == "/"} - {""}
    out: Dict[Any, Dict[str, List[str]]] = {}
    for f in findings:
        if not any(isinstance(e, dict) and e.get("form") == "search"
                   for e in f.get("evidence") or []):
            continue
        named: List[str] = []
        unresolved: List[str] = []
        directories: List[str] = []
        outside: List[str] = []
        subs = {m.rstrip("/") for m in submodules}
        for e in f.get("evidence") or []:
            if not isinstance(e, dict) or e.get("form") != "search":
                continue
            for c in e.get("candidates") or []:
                key, why = resolve_document(docs, c)
                if key is None:
                    d = (c or "").strip().rstrip("/")
                    while d.startswith("./"):
                        d = d[2:]
                    if d in subs or any(d.startswith(m + "/") for m in subs):
                        if d not in outside:
                            outside.append(d)
                        continue
                    if d in dirs:
                        if d not in directories:
                            directories.append(d)
                        continue
                    unresolved.append(why or f"{c!r} does not resolve")
                elif key not in named:
                    named.append(key)
        unopened = [k for k in named if k not in read] if read is not None else []
        out[f.get("claim_id")] = {"named": named, "unopened": unopened,
                                  "unresolved": unresolved,
                                  "directories": directories,
                                  "outside": outside}
    return out


def doc_excluded(key: Optional[str], excludes: Optional[Sequence[str]]) -> bool:
    """Whether a corpus key is one the engagement excluded from evidence as
    documentation: the entry itself or anything under a directory entry."""
    if not key or not excludes:
        return False
    k = key.strip().lstrip("./")
    for e in excludes:
        e = str(e).strip().lstrip("./").rstrip("/")
        if e and (k == e or k.startswith(e + "/")):
            return True
    return False


def check_output(obj: Dict[str, Any], corpus: Path, claim_source: str,
                 frozen: Sequence[Dict[str, Any]],
                 read: Optional[set] = None,
                 excludes: Optional[Sequence[str]] = None) -> Dict[str, Any]:
    """Everything METHOD requires that the schema cannot express.

    Returns `{"ok": bool, "problems": [...], "figures": {...}}`, the shape the
    runner already records for its other checks. A problem is a sentence a
    person can act on, not a code.

    `read` is the set of corpus keys the run opened. With it, METHOD §8's
    candidate rule is checked: an `unverifiable` finding whose searches named
    a file that was not opened must carry `not_examined`, and one that carries
    `not_examined` must name such a file. Without it (None) the disposition is
    taken as recorded.
    """
    problems: List[str] = []
    view = corpus_view(corpus)
    docs = corpus_index(corpus)
    joined: List[str] = []
    # Citations into documents the engagement excluded from evidence
    # (METHOD §7). Recorded, not failed: a claim about the document itself
    # may cite it, and the review judges relevance with the mark in view.
    excluded_cites: List[Dict[str, Any]] = []

    def _cite(where: str, doc: Any, lines: Any, quote: Any) -> None:
        """One citation, against the corpus. METHOD §7.

        A QUOTE THAT JOINS LINES OF THE CITED RANGE IS COUNTED, NOT FAILED.
        METHOD §7 asks for one contiguous span, and the whole-quote test is
        the check of that. But 11 of the 12 quote failures in one GLM run
        (2026-09-02) were bullets copied from inside the cited range with the
        sub-bullets between them dropped, joined with newlines: the lines all
        exist, at the lines cited. Reporting that as "does not appear in the
        document at all" was false and blocked the run. So a quote whose
        newline-separated pieces each sit inside the cited span is recorded in
        `figures["joined_quotes"]` and is not a problem. The tolerance depends
        on the model joining with a newline; a quote joined with spaces still
        fails whole, as it should, since the method forbids the join.
        """
        key, why = resolve_document(docs, doc)
        if key is None:
            problems.append(f"{where}: {why}")
            return
        if doc_excluded(key, excludes):
            excluded_cites.append({"where": where, "document": key, "lines": lines})
        body = docs[key]
        if (not isinstance(lines, list) or len(lines) != 2
                or not all(isinstance(n, int) for n in lines)):
            problems.append(f"{where}: lines {lines!r} is not a start and end")
            return
        lo, hi = lines
        if lo < 1 or hi < lo:
            problems.append(f"{where}: lines {lo}-{hi} is not a range")
            return
        if hi > len(body):
            problems.append(f"{where}: cites {doc}:{lo}-{hi}, which has "
                            f"{len(body)} lines — the integers are not line "
                            f"numbers, or the range is wrong")
            return
        if isinstance(quote, str) and _norm(quote):
            span = _norm("\n".join(body[lo - 1:hi]))
            if _norm(quote) not in span:
                pieces = [_norm(x) for x in quote.splitlines() if _norm(x)]
                if len(pieces) > 1 and all(pc in span for pc in pieces):
                    joined.append(where)
                    return
                whole = _norm("\n".join(body))
                missing = [pc for pc in pieces if pc not in span] or [_norm(quote)]
                problems.append(
                    f"{where}: quote is not at {doc}:{lo}-{hi}"
                    + ("; it is elsewhere in that document"
                       if _norm(quote) in whole else
                       f"; this part is not at those lines: "
                       f"{missing[0][:80]!r}"))

    findings = obj.get("findings")
    if not isinstance(findings, list):
        return {"ok": False, "problems": ["`findings` is not a list"],
                "figures": {}}

    # The object checked here is assembled from batches (runner
    # `_merge_emissions`), so findings from the batches that completed sit
    # beside the `not_completed` of any that did not, or of the claims the
    # runner never put to adjudication. Either way METHOD §4 says the run is
    # not an audit, and that is reported once, as the run's incompleteness.
    incomplete = obj.get("not_completed")
    if incomplete:
        problems.append(f"run not completed (METHOD §4): {incomplete}")
    if not incomplete and not findings:
        problems.append("no findings and no `not_completed` — METHOD §4 "
                        "requires one or the other")

    # THE FROZEN SURFACE IS THE DENOMINATOR, and this is the check that
    # freezing buys. v1 could not make it: enumeration and adjudication
    # happened in one pass, so a claim quietly dropped left no trace of ever
    # having been enumerated.
    ids = {c.get("id") for c in frozen}
    seen: Dict[Any, int] = {}
    verdicts: Dict[str, int] = {}
    forms: Dict[str, int] = {}
    candidates = candidate_files(findings, docs, read, view["submodules"])
    not_examined = 0
    unopened = sorted({k for c in candidates.values() for k in c["unopened"]})
    for i, f in enumerate(findings, 1):
        w = f"finding {i}"
        adj = f.get("adjudication") or {}
        ev = f.get("evidence") or []

        cid = f.get("claim_id")
        if cid not in ids:
            problems.append(f"{w}: claim_id {cid!r} is not in the frozen "
                            f"surface")
        elif cid in seen:
            problems.append(f"{w}: claim {cid} was already adjudicated by "
                            f"finding {seen[cid]} — METHOD §4 gives each claim "
                            f"exactly one finding")
        else:
            seen[cid] = i

        v = adj.get("verdict")
        verdicts[v] = verdicts.get(v, 0) + 1
        if v not in VERDICTS:
            problems.append(f"{w}: verdict {v!r} is not one of METHOD §6's")
        if v in GAP_REQUIRED and not _norm(adj.get("gap")):
            problems.append(f"{w}: verdict {v!r} requires `gap` (METHOD §13)")
        if v == "real" and _norm(adj.get("gap")):
            problems.append(f"{w}: verdict `real` carries a `gap`")
        if v == "unverifiable":
            because = adj.get("unresolved_because")
            if because not in UNRESOLVED_BECAUSE:
                problems.append(f"{w}: `unverifiable` requires "
                                f"`unresolved_because` (METHOD §8)")
            kinds = {e.get("kind") for e in ev
                     if isinstance(e, dict) and e.get("form") == "search"}
            missing = [k for k in SEARCH_KINDS if k not in kinds]
            if missing:
                problems.append(f"{w}: `unverifiable` needs a lexical and a "
                                f"structural search (METHOD §8); missing "
                                f"{', '.join(missing)}")
            cand = candidates.get(cid) or {}
            for why in cand.get("unresolved") or []:
                problems.append(f"{w}: candidate {why}")
            # A candidate in a submodule names a place the materials do not
            # reach: consistent with `outside_the_materials`, and with no
            # other disposition.
            if cand.get("outside") and because != "outside_the_materials":
                problems.append(
                    f"{w}: candidate {', '.join(cand['outside'])} is a "
                    f"submodule the materials do not include — the "
                    f"disposition is `outside_the_materials` (METHOD §8)")
            if because == NOT_EXAMINED:
                not_examined += 1
            if read is not None:
                if cand.get("unopened") and because in UNRESOLVED_BECAUSE \
                        and because != NOT_EXAMINED:
                    problems.append(
                        f"{w}: searches named "
                        f"{', '.join(cand['unopened'])} and the run did not "
                        f"open it — the disposition is `not_examined` "
                        f"(METHOD §8)")
                if because == NOT_EXAMINED and not cand.get("unopened"):
                    problems.append(
                        f"{w}: `not_examined` but every candidate the "
                        f"searches named was opened"
                        + (" (none were named)" if not cand.get("named")
                           else "") + " (METHOD §8)")
        elif not any(isinstance(e, dict) and e.get("form") == "citation"
                     for e in ev) and \
                any(isinstance(e, dict) and e.get("form") == "search" for e in ev):
            # A claim of absence holds on its searches: both kinds, and only
            # once every candidate is opened (METHOD §8).
            kinds = {e.get("kind") for e in ev
                     if isinstance(e, dict) and e.get("form") == "search"}
            missing = [k for k in SEARCH_KINDS if k not in kinds]
            if missing:
                problems.append(f"{w}: verdict {v!r} rests on searches and "
                                f"needs a lexical and a structural one "
                                f"(METHOD §8); missing {', '.join(missing)}")
            if read is not None and (candidates.get(cid) or {}).get("unopened"):
                problems.append(
                    f"{w}: verdict {v!r} rests on searches that named "
                    f"{', '.join(candidates[cid]['unopened'])} and the run "
                    f"did not open it — until every candidate is opened the "
                    f"verdict is `unverifiable`, `not_examined` (METHOD §8)")

        if not ev:
            problems.append(f"{w}: no evidence — METHOD §4 gives every finding "
                            f"the evidence its adjudication rests on")
        for j, e in enumerate(ev, 1):
            ew = f"{w} evidence {j}"
            if not isinstance(e, dict):
                problems.append(f"{ew}: not an object"); continue
            form = e.get("form")
            forms[form] = forms.get(form, 0) + 1
            if form not in FORM_FIELDS:
                problems.append(f"{ew}: form {form!r} is not one of "
                                f"{', '.join(EVIDENCE_FORMS)}")
                continue
            for field in FORM_FIELDS[form]:
                if e.get(field) in (None, "", []):
                    problems.append(f"{ew}: form {form!r} requires {field!r} "
                                    f"(METHOD §7)")
            if form == "citation":
                _cite(ew, e.get("document"), e.get("lines"), e.get("quote"))
            elif form == "derived":
                basis = e.get("basis") or []
                if len(basis) < 2:
                    problems.append(f"{ew}: a derived fact follows from two or "
                                    f"more facts (METHOD §7); {len(basis)} given")
                for k, b in enumerate(basis, 1):
                    if isinstance(b, dict):
                        _cite(f"{ew} basis {k}", b.get("document"),
                              b.get("lines"), b.get("quote"))

    for cid in sorted(ids - set(seen), key=lambda x: (x is None, x)):
        problems.append(f"claim {cid} in the frozen surface has no finding — "
                        f"METHOD §4 requires one for every claim")

    for i, u in enumerate(obj.get("unclaimed") or [], 1):
        e = (u or {}).get("evidence") or {}
        if e.get("form") == "citation":
            _cite(f"unclaimed {i}", e.get("document"), e.get("lines"),
                  e.get("quote"))

    return {"ok": not problems, "problems": problems,
            "figures": {"findings": len(findings),
                        "frozen_claims": len(ids),
                        "adjudicated": len(seen),
                        "verdicts": verdicts,
                        "evidence_forms": forms,
                        "joined_quotes": joined, "excluded_citations": excluded_cites,
                        # METHOD §8: findings recorded `not_examined`, and the
                        # files searches named that the run never opened.
                        "not_examined": not_examined,
                        "unopened_candidates": unopened,
                        "unclaimed": len(obj.get("unclaimed") or []),
                        "questions": len(obj.get("questions") or []),
                        "not_completed": bool(incomplete)}}
