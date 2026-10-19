"""The licence of each component the scan lists, where the dependency file
does not record one, and how much it asks of the buyer. No model.

    licences.complete(syft_doc, comps)   adds `licence` to each component row

WHY. Syft reads a licence only from the dependency file, and most kinds do
not carry one: Cargo.lock, pip requirements and GitHub Action references
record none, so chhoto's appendix said "not recorded" for all 266 components
(2026-10-19). A published package declares its licence to its registry, and
deps.dev (Google's Open Source Insights) serves that declaration for an exact
version of a Rust, npm, PyPI, Go, Maven or NuGet package. GitHub Actions are
looked up as their repository's licence.

WHAT LEAVES THE MACHINE. The name and version of each component from a public
registry, sent to deps.dev, and the owner and name of each GitHub Action's
repository, sent to GitHub. A component declared by path or by git, from a
private registry, or with a range instead of one version is never sent: its
name could be the seller's own (Bruce, 2026-10-19: lookups for all
engagements).

THE CACHE. One file outside the repository, which is public: it would show
which packages the practice's clients use. A published version cannot be
changed on its registry, so its entry never expires; "not found" expires
after a week, a GitHub repository's licence after 30 days, because it is the
repository's current licence and not the tag's. No engagement is named in it.

FLAGS. A component is flagged when its licence could change what the buyer
may do, or cannot be told: none found, copyleft or source-available, an
identifier the table below does not know, or two declarations that disagree.
For a flagged package from crates.io, npm or PyPI, the archive published for
that version is downloaded, read in memory and never unpacked or run: the
licence its own manifest declares is recorded, and its licence files are
listed with their first lines for a person to read. Nothing classifies their
text (Bruce, 2026-10-19).
"""
from __future__ import annotations

import datetime
import email.parser
import io
import json
import logging
import os
import re
import sys
import tarfile
import tomllib
import urllib.error
import urllib.parse
import urllib.request
import zipfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

REPO = Path(__file__).resolve().parents[2]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from utils.file_utils import atomic_write_json                  # noqa: E402

logger = logging.getLogger("composition.licences")

CACHE = Path(os.environ.get("TUUYI_LICENCE_CACHE",
                            Path.home() / ".cache/tuuyi/licences.json"))
NOT_FOUND_DAYS = 7
GITHUB_DAYS = 30
ARCHIVE_MAX_BYTES = 20 * 1024 * 1024
TIMEOUT_S = 20
WORKERS = 8
USER_AGENT = "tuuyi-composition-scan (info@tuuyi.com)"

#: deps.dev's name for each purl type it serves.
SYSTEMS = {"cargo": "cargo", "npm": "npm", "pypi": "pypi", "golang": "go",
           "maven": "maven", "nuget": "nuget"}
#: Cargo.lock `source` values that name the public crates.io index.
CRATES_IO = ("registry+https://github.com/rust-lang/crates.io-index",
             "sparse+https://index.crates.io/")
NPM_HOSTS = ("registry.npmjs.org", "registry.yarnpkg.com")

#: What each licence asks of the buyer, in rising order. An OR expression
#: takes its lowest option, an AND its highest.
CLASSES = ("permissive", "weak copyleft", "strong copyleft", "network copyleft",
           "source-available", "unknown")
_TABLE = {
    "permissive": (
        "0BSD", "AFL-3.0", "Apache-1.1", "Apache-2.0", "Artistic-2.0",
        "BlueOak-1.0.0", "BSD-1-Clause", "BSD-2-Clause", "BSD-2-Clause-Patent",
        "BSD-3-Clause", "BSD-3-Clause-Clear", "BSD-4-Clause", "BSL-1.0",
        "CC-BY-3.0", "CC-BY-4.0", "CC0-1.0", "curl", "HPND", "ISC", "MIT",
        "MIT-0", "MIT-CMU", "NCSA", "OpenSSL", "PostgreSQL", "PSF-2.0",
        "Python-2.0", "Python-2.0.1", "Unicode-3.0", "Unicode-DFS-2016",
        "Unlicense", "UPL-1.0", "W3C", "WTFPL", "X11", "Zlib", "zlib-acknowledgement"),
    "weak copyleft": (
        "CC-BY-SA-3.0", "CC-BY-SA-4.0", "CDDL-1.0", "CDDL-1.1", "EPL-1.0",
        "EPL-2.0", "LGPL-2.0", "LGPL-2.0-only", "LGPL-2.0-or-later", "LGPL-2.1",
        "LGPL-2.1-only", "LGPL-2.1-or-later", "LGPL-3.0", "LGPL-3.0-only",
        "LGPL-3.0-or-later", "MPL-1.1", "MPL-2.0", "MPL-2.0-no-copyleft-exception",
        "OFL-1.1"),
    "strong copyleft": (
        "EUPL-1.1", "EUPL-1.2", "GPL-1.0-or-later", "GPL-2.0", "GPL-2.0-only",
        "GPL-2.0-or-later", "GPL-3.0", "GPL-3.0-only", "GPL-3.0-or-later", "OSL-3.0"),
    "network copyleft": (
        "AGPL-1.0", "AGPL-3.0", "AGPL-3.0-only", "AGPL-3.0-or-later", "SSPL-1.0"),
    "source-available": (
        "BUSL-1.1", "CC-BY-NC-4.0", "CC-BY-NC-SA-4.0", "CC-BY-ND-4.0", "Elastic-2.0",
        "PolyForm-Noncommercial-1.0.0", "PolyForm-Small-Business-1.0.0"),
}
_CLASS_OF = {i.casefold(): c for c, ids in _TABLE.items() for i in ids}
_LICENCE_FILE = re.compile(r"^(licen[cs]e|copying|notice)", re.IGNORECASE)


# ---- what each licence asks ---------------------------------------------------

def _tokens(expr: str) -> List[str]:
    return re.findall(r"\(|\)|[^\s()]+", expr)


def classify(expr: Optional[str]) -> Optional[str]:
    """The class of an SPDX expression; None for no expression. An identifier
    is looked up whole and without regard to case; one the table does not
    have, or an expression that does not parse, is `unknown`."""
    if not expr or not expr.strip():
        return None
    toks, pos = _tokens(expr), 0

    def peek():
        return toks[pos] if pos < len(toks) else None

    def take():
        nonlocal pos
        pos += 1
        return toks[pos - 1]

    def atom() -> int:
        t = take() if peek() is not None else None
        if t is None or t == ")":
            raise ValueError("expected a licence")
        if t == "(":
            r = alternatives()
            if take() != ")":
                raise ValueError("unbalanced")
        else:
            ident = t[:-1] if t.endswith("+") else t
            r = CLASSES.index(_CLASS_OF.get(ident.casefold(), "unknown"))
        if peek() is not None and peek().upper() == "WITH":
            take(); take()                       # an exception keeps the base's class
        return r

    def all_of() -> int:
        r = atom()
        while peek() is not None and peek().upper() == "AND":
            take(); r = max(r, atom())
        return r

    def alternatives() -> int:
        r = all_of()
        while peek() is not None and peek().upper() == "OR":
            take(); r = min(r, all_of())
        return r

    try:
        r = alternatives()
        if pos != len(toks):
            raise ValueError("trailing text")
    except (ValueError, IndexError):
        return "unknown"
    return CLASSES[r]


def _ids(expr: Optional[str]) -> frozenset:
    return frozenset(t.rstrip("+").casefold() for t in _tokens(expr or "")
                     if t not in ("(", ")") and t.upper() not in ("AND", "OR", "WITH"))


# ---- which components are sent out ---------------------------------------------

def _purl_name(purl: str) -> Tuple[Optional[str], Optional[str], Optional[str]]:
    """(type, name, version) from a purl, with maven's group joined by ':'."""
    m = re.match(r"^pkg:([^/]+)/(.+?)(?:@([^?#]+))?(?:[?#].*)?$", purl or "")
    if not m:
        return None, None, None
    typ, path, ver = m.group(1), urllib.parse.unquote(m.group(2)), m.group(3)
    if typ == "maven" and "/" in path:
        path = path.replace("/", ":", 1)
    return typ, path, urllib.parse.unquote(ver) if ver else None


def eligibility(comp: Dict[str, Any], artifact: Dict[str, Any]) -> Tuple[str, Optional[str]]:
    """("deps.dev"|"github", key) for a component that may be looked up, else
    ("local"|"private"|"range"|"unsupported", None). The key is
    "<system>/<name>@<version>", or "github/<owner>/<repo>"."""
    typ, name, ver = _purl_name(comp.get("purl") or "")
    meta = artifact.get("metadata") or {}
    # Syft gives the target's own crate (and a path dependency) an empty
    # source and no purl.
    if comp.get("type") == "rust-crate" and not meta.get("source"):
        return "local", None
    if typ == "github":
        parts = (name or "").split("/")
        return ("github", f"github/{parts[0]}/{parts[1]}") if len(parts) >= 2 else ("unsupported", None)
    if typ not in SYSTEMS:
        return "unsupported", None
    if comp.get("version_range") or not comp.get("version"):
        return "range", None
    if typ == "cargo" and not meta.get("source", "").startswith(CRATES_IO):
        return "private", None
    if typ == "npm":
        # No download address: the project's own package, or a workspace or
        # linked one inside it.
        if not meta.get("resolved"):
            return "local", None
        if urllib.parse.urlparse(meta["resolved"]).hostname not in NPM_HOSTS:
            return "private", None
    version = comp["version"]
    if typ == "pypi":
        # A local build label (torch 2.8.0+cpu) is not a release on PyPI; the
        # release it was built from is.
        version = version.split("+", 1)[0]
    return "deps.dev", f"{SYSTEMS[typ]}/{name}@{version}"


# ---- the network ------------------------------------------------------------------

def _get(url: str, limit: int = ARCHIVE_MAX_BYTES) -> Optional[bytes]:
    """The body, or None for a 404. Other failures raise. Replaced in tests."""
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    try:
        with urllib.request.urlopen(req, timeout=TIMEOUT_S) as r:
            body = r.read(limit + 1)
    except urllib.error.HTTPError as e:
        if e.code == 404:
            return None
        raise
    if len(body) > limit:
        raise ValueError(f"larger than {limit} bytes")
    return body


def _q(s: str) -> str:
    return urllib.parse.quote(s, safe="")


def _lookup(source: str, key: str) -> Dict[str, Any]:
    now = datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")
    if source == "github":
        _, owner, repo = key.split("/", 2)
        url = f"https://api.github.com/repos/{_q(owner)}/{_q(repo)}/license"
        body = _get(url, 1024 * 1024)
        spdx = ((json.loads(body).get("license") or {}).get("spdx_id")) if body else None
        return {"expression": spdx, "source": "github", "url": url, "at": now,
                "found": body is not None}
    system, rest = key.split("/", 1)
    name, version = rest.rsplit("@", 1)
    url = (f"https://api.deps.dev/v3/systems/{system}/packages/{_q(name)}"
           f"/versions/{_q(version)}")
    body = _get(url, 4 * 1024 * 1024)
    lic = (json.loads(body).get("licenses") or []) if body else []
    expr = " AND ".join(f"({x})" if " " in x else x for x in lic) if len(lic) > 1 else (lic[0] if lic else None)
    return {"expression": expr, "source": "deps.dev", "url": url, "at": now,
            "found": body is not None}


def _fresh(entry: Dict[str, Any], key: str) -> bool:
    days = (GITHUB_DAYS if key.startswith("github/") else
            None if entry.get("found") else NOT_FOUND_DAYS)
    if days is None:
        return True
    at = datetime.datetime.fromisoformat(entry["at"])
    return datetime.datetime.now(datetime.timezone.utc) - at < datetime.timedelta(days=days)


def _load_cache() -> Dict[str, Any]:
    if not CACHE.is_file():
        return {}
    try:
        return json.loads(CACHE.read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        logger.warning("licence cache unreadable (%s); starting empty", e)
        return {}


# ---- the archive of a flagged package -------------------------------------------

def _first_lines(data: bytes, n: int = 3) -> List[str]:
    lines = [l.strip() for l in data.decode("utf-8", "replace").splitlines() if l.strip()]
    return lines[:n]


def _archive_record(system: str, name: str, version: str) -> Dict[str, Any]:
    """The licence the package's own manifest declares and its licence files'
    first lines, from the archive published for this version."""
    rec: Dict[str, Any] = {"url": None, "declared": None, "files": []}
    if system == "cargo":
        rec["url"] = f"https://static.crates.io/crates/{_q(name)}/{_q(name)}-{_q(version)}.crate"
        blob = _get(rec["url"])
        if blob is None:
            return {**rec, "error": "not found"}
        top = f"{name}-{version}/"
        with tarfile.open(fileobj=io.BytesIO(blob), mode="r:gz") as tf:
            members = {m.name: m for m in tf.getmembers() if m.isfile()}
            toml = members.get(top + "Cargo.toml")
            if toml is not None:
                pkg = tomllib.loads(tf.extractfile(toml).read().decode("utf-8", "replace")).get("package") or {}
                rec["declared"] = pkg.get("license") or (
                    f"licence file {pkg['license-file']}" if pkg.get("license-file") else None)
            rec["files"] = [{"file": m[len(top):], "first_lines": _first_lines(tf.extractfile(members[m]).read())}
                            for m in sorted(members) if m.startswith(top) and "/" not in m[len(top):]
                            and _LICENCE_FILE.match(m[len(top):])]
        return rec
    if system == "npm":
        meta = _get(f"https://registry.npmjs.org/{name.replace('/', '%2F')}/{_q(version)}", 4 * 1024 * 1024)
        if meta is None:
            return {**rec, "error": "not found"}
        rec["url"] = (json.loads(meta).get("dist") or {}).get("tarball")
        blob = _get(rec["url"]) if rec["url"] else None
        if blob is None:
            return {**rec, "error": "no archive"}
        with tarfile.open(fileobj=io.BytesIO(blob), mode="r:gz") as tf:
            members = {m.name: m for m in tf.getmembers() if m.isfile()}
            pj = members.get("package/package.json")
            if pj is not None:
                lic = json.loads(tf.extractfile(pj).read()).get("license")
                rec["declared"] = lic.get("type") if isinstance(lic, dict) else lic
            rec["files"] = [{"file": m[8:], "first_lines": _first_lines(tf.extractfile(members[m]).read())}
                            for m in sorted(members) if m.startswith("package/") and "/" not in m[8:]
                            and _LICENCE_FILE.match(m[8:])]
        return rec
    if system == "pypi":
        meta = _get(f"https://pypi.org/pypi/{_q(name)}/{_q(version)}/json", 4 * 1024 * 1024)
        if meta is None:
            return {**rec, "error": "not found"}
        urls = json.loads(meta).get("urls") or []
        pick = (next((u for u in urls if u.get("packagetype") == "sdist"), None)
                or next((u for u in urls if u.get("packagetype") == "bdist_wheel"), None))
        if pick is None:
            return {**rec, "error": "no archive"}
        rec["url"] = pick["url"]
        blob = _get(rec["url"])
        if blob is None:
            return {**rec, "error": "no archive"}
        files: Dict[str, bytes] = {}
        if pick["packagetype"] == "sdist" and pick["url"].endswith((".tar.gz", ".tgz")):
            with tarfile.open(fileobj=io.BytesIO(blob), mode="r:gz") as tf:
                files = {m.name: tf.extractfile(m).read() for m in tf.getmembers()
                         if m.isfile() and m.name.count("/") == 1}
            head = next((v for k, v in files.items() if k.endswith("/PKG-INFO")), None)
        elif pick["packagetype"] == "sdist" and pick["url"].endswith(".zip") or pick["packagetype"] == "bdist_wheel":
            with zipfile.ZipFile(io.BytesIO(blob)) as zf:
                files = {n: zf.read(n) for n in zf.namelist()
                         if n.count("/") <= 1 or ".dist-info/licenses/" in n}
            head = next((v for k, v in files.items() if k.endswith((".dist-info/METADATA", "/PKG-INFO"))), None)
        else:
            return {**rec, "error": "archive format not read"}
        if head is not None:
            msg = email.parser.BytesHeaderParser().parsebytes(head)
            rec["declared"] = msg.get("License-Expression") or msg.get("License")
        rec["files"] = [{"file": k, "first_lines": _first_lines(v)} for k, v in sorted(files.items())
                        if _LICENCE_FILE.match(k.rsplit("/", 1)[-1])]
        return rec
    return {**rec, "error": "archive not checked for this kind"}


#: Some manifests put a whole licence text in the licence field (agno 1.7.6
#: carries all of MPL-2.0 in PKG-INFO's License); the record keeps its start.
DECLARED_CHARS = 200


def _short(declared: Optional[str]) -> Optional[str]:
    if not declared:
        return declared
    flat = " ".join(str(declared).split())
    return flat if len(flat) <= DECLARED_CHARS else flat[:DECLARED_CHARS] + " [cut; the full text is in the archive]"


# ---- the whole pass -------------------------------------------------------------

def _flags(cls: Optional[str], expr: Optional[str], lockfile: Optional[str]) -> List[str]:
    out = []
    if cls is None:
        out.append("no licence found")
    elif cls == "unknown":
        out.append("identifier not recognised")
    elif cls != "permissive":
        out.append(cls)
    if lockfile and expr and _ids(lockfile) != _ids(expr):
        out.append("the dependency file and the registry disagree")
    return out


def complete(syft_doc: Dict[str, Any], comps: List[Dict[str, Any]]) -> Dict[str, int]:
    """Add `licence` to each component row in place; return the counts for
    scan_meta. Never raises for a lookup: a failure is recorded on the row."""
    arts = {a.get("id"): a for a in syft_doc.get("artifacts") or []}
    cache = _load_cache()
    plan, wanted = [], set()
    for c in comps:
        how, key = eligibility(c, arts.get(c["id"]) or {})
        plan.append((c, how, key))
        if key and not (key in cache and _fresh(cache[key], key)):
            wanted.add((how, key))
    counts = {"looked_up": len(wanted), "cached": 0, "not_looked_up": 0, "lookup_failed": 0}

    def fetch(item):
        how, key = item
        try:
            return key, _lookup(how, key)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("licence lookup %s failed: %s", key, type(e).__name__)
            return key, {"error": type(e).__name__}

    with ThreadPoolExecutor(WORKERS) as pool:
        for key, got in pool.map(fetch, sorted(wanted)):
            if "error" in got:
                counts["lookup_failed"] += 1
            else:
                cache[key] = got
    archives = []
    for c, how, key in plan:
        lockfile = " OR ".join(c["licences"]) if c["licences"] else None
        entry = cache.get(key) if key else None
        if not key:
            counts["not_looked_up"] += 1
        elif entry and (how, key) not in wanted:
            counts["cached"] += 1
        registry = (entry or {}).get("expression")
        # GitHub answers NOASSERTION for a licence file it cannot identify;
        # it stays as the expression, is `unknown`, and is flagged for reading.
        expr = registry or lockfile
        frm = ("local" if how == "local" else entry["source"] if registry
               else "lockfile" if lockfile else "none")
        cls = classify(expr)
        if how == "local":
            flags = []
        elif key and entry is None and not lockfile:
            flags = ["lookup failed"]
        elif not key and not lockfile:
            flags = [{"private": "not looked up: not from a public registry",
                      "range": "not looked up: no single version",
                      "unsupported": "not looked up: a kind the lookup does not cover"}[how]]
        else:
            flags = _flags(cls, expr, lockfile if registry else None)
        c["licence"] = {"expression": expr, "from": frm, "url": (entry or {}).get("url"),
                        "at": (entry or {}).get("at"), "class": cls, "flags": flags,
                        "eligibility": how, "archive": None}
        if flags and how == "deps.dev" and key.split("/", 1)[0] in ("cargo", "npm", "pypi"):
            archives.append((c, key))

    def check(item):
        c, key = item
        akey = "archive:" + key
        if akey in cache:
            return c, key, cache[akey], True
        system, rest = key.split("/", 1)
        name, version = rest.rsplit("@", 1)
        try:
            return c, key, _archive_record(system, name, version), False
        except Exception as e:                                 # noqa: BLE001
            logger.warning("archive check %s failed: %s", key, type(e).__name__)
            return c, key, {"error": type(e).__name__}, False

    with ThreadPoolExecutor(WORKERS) as pool:
        for c, key, rec, was_cached in pool.map(check, archives):
            if not was_cached and "error" not in rec:
                cache["archive:" + key] = rec
            # The cache keeps the declaration whole; the row shows its start.
            full = rec.get("declared")
            c["licence"]["archive"] = {**rec, "declared": _short(full)}
            if full and c["licence"]["expression"] and \
                    _ids(full) != _ids(c["licence"]["expression"]):
                c["licence"]["flags"].append("the package's own manifest disagrees")

    CACHE.parent.mkdir(parents=True, exist_ok=True)
    atomic_write_json(CACHE, cache, indent=1)
    counts["flagged"] = sum(1 for c in comps if c["licence"]["flags"])
    counts["archives_checked"] = len(archives)
    by = {}
    for c in comps:
        k = c["licence"]["class"] or ("part of the target" if c["licence"]["from"] == "local" else "none found")
        by[k] = by.get(k, 0) + 1
    counts["by_licence_class"] = by
    return counts


#: Flags that say the licence was not looked up, as opposed to one a person
#: has to read.
NOT_LOOKED_UP = ("not looked up: not from a public registry",
                 "not looked up: no single version",
                 "not looked up: a kind the lookup does not cover")


def to_read(comp: Dict[str, Any]) -> bool:
    return any(f not in NOT_LOOKED_UP for f in comp["licence"]["flags"])
