#!/usr/bin/env python3
"""Scan an engagement's target for the third-party components its dependency
files declare, and match them against a pinned vulnerability database.

    python3 workflowsv2/composition/scan.py --engagement <name>

Runs only when the engagement enables it (`composition: true` in
engagement.yaml). The chain job calls it first when it is enabled; it can
also be run on its own.

WHAT RUNS. Two programs, both run locally, and no model:

  syft    reads the dependency files under the target (lockfiles, manifests,
          CI workflow files) and lists every component they declare, with the
          licence where the file records one. Unpinned requirements (a version
          range, not one version) are listed with the range, and the version
          syft guesses for them (the range's lower bound) is not used.
  grype   matches the listed components against a vulnerability database by
          name and version. The database is a pinned offline copy (GRYPE_DB),
          never updated during a scan, and its build date is recorded.

WHAT A MATCH IS. A component whose recorded version falls in a range some
advisory names. It is level 1 of the Linux Foundation guide's ladder at most:
nobody checked whether the component runs, or whether the vulnerable part is
called. Components whose version is not fixed are not matched at all.

WHAT IS WRITTEN, under <engagement>/composition/<timestamp>/, outside the
target because the seller sees the target:

  syft.json        syft's own output, kept whole
  grype.json       grype's own output, kept whole
  components.json  one row per component, with a stable id
  matches.json     one row per match, with a stable id; the practice's record,
                   not shown to the client
  scan_meta.json   tool versions, database build, target commit, the files
                   the scan read, and the counts

The report shows these as an appendix (appendix.py).
"""
from __future__ import annotations

import argparse
import datetime
import json
import logging
import os
import subprocess
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from workflowsv2 import engagement_state as state              # noqa: E402

logger = logging.getLogger("composition.scan")

#: Where the pinned vulnerability database lives: a grype cache directory made
#: by `grype db import <archive>`. Outside the repository because the archive
#: is ~160 MB. Override with TUUYI_GRYPE_DB_DIR.
GRYPE_DB = Path(os.environ.get("TUUYI_GRYPE_DB_DIR",
                               Path.home() / ".local/share/tuuyi/grype-db/cache"))
#: Paths under the target the scan leaves out: files the practice extracted
#: there (materials sorting writes claim sources to claim_sources/).
EXCLUDE = ("./claim_sources/**",)
SCANS = "composition"


def enabled(eng_dir: Path) -> bool:
    return state._engagement_yaml(eng_dir).get("composition") is True


def _env() -> Dict[str, str]:
    env = dict(os.environ)
    env.update({"SYFT_CHECK_FOR_APP_UPDATE": "false",
                "GRYPE_CHECK_FOR_APP_UPDATE": "false",
                # List unpinned requirements rather than drop them silently;
                # their guessed versions are not used (see _fixed).
                "SYFT_PYTHON_GUESS_UNPINNED_REQUIREMENTS": "true",
                "GRYPE_DB_CACHE_DIR": str(GRYPE_DB),
                "GRYPE_DB_AUTO_UPDATE": "false",
                "GRYPE_DB_VALIDATE_AGE": "false"})
    return env


def _run(argv: List[str]) -> None:
    r = subprocess.run(argv, env=_env(), capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(f"{argv[0]} failed ({r.returncode}): {r.stderr.strip()[-600:]}")


def _fixed(artifact: Dict[str, Any]) -> Optional[str]:
    """The version constraint when the file gives a range instead of one
    version, else None. A range's guessed version is not a fact about the
    target, so such a component is listed with its range and not matched."""
    c = (artifact.get("metadata") or {}).get("versionConstraint")
    if not c:
        return None
    c = str(c).strip()
    return None if c.startswith("==") and "," not in c else c


def components(syft_doc: Dict[str, Any]) -> List[Dict[str, Any]]:
    rows = []
    for a in syft_doc.get("artifacts") or []:
        rng = _fixed(a)
        rows.append({
            "id": a["id"],
            "name": a.get("name"),
            "version": None if rng else a.get("version"),
            "version_range": rng,
            "type": a.get("type"),
            "purl": a.get("purl"),
            "licences": [l.get("spdxExpression") or l.get("value")
                         for l in (a.get("licenses") or [])
                         if l.get("spdxExpression") or l.get("value")],
            "files": sorted({(l.get("path") or "").lstrip("/")
                             for l in (a.get("locations") or [])} - {""}),
        })
    return sorted(rows, key=lambda r: (r["type"] or "", (r["name"] or "").lower(),
                                       r["version"] or ""))


def matches(grype_doc: Dict[str, Any], comps: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    ranged = {c["id"] for c in comps if c["version_range"]}
    rows = []
    for m in grype_doc.get("matches") or []:
        art, vul = m.get("artifact") or {}, m.get("vulnerability") or {}
        if art.get("id") in ranged:
            continue
        rows.append({
            "id": f"{vul.get('id')}@{art.get('id')}",
            "vulnerability": vul.get("id"),
            "severity": vul.get("severity"),
            "source": vul.get("dataSource"),
            "component": art.get("id"),
            "name": art.get("name"),
            "version": art.get("version"),
            "match": [d.get("type") for d in m.get("matchDetails") or []],
            "fixed_in": (vul.get("fix") or {}).get("versions") or [],
        })
    return sorted(rows, key=lambda r: (r["name"] or "", r["vulnerability"] or ""))


def _target_rev(target: Path) -> Optional[str]:
    """The target's commit, only when the target is itself the top of a git
    checkout. A plain directory inside some other repository (a fixture's
    corpus inside this one) would otherwise report that repository's commit."""
    def git(*a: str) -> str:
        return subprocess.run(["git", "-C", str(target), *a],
                              capture_output=True, text=True).stdout.strip()
    top = git("rev-parse", "--show-toplevel")
    if not top or Path(top).resolve() != target.resolve():
        return None
    return git("rev-parse", "HEAD") or None


def run(eng_dir: Path) -> Path:
    target = state.target_dir(eng_dir)
    if not target.is_dir():
        raise SystemExit(f"{eng_dir.name}: no target")
    ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    out = eng_dir / SCANS / ts
    out.mkdir(parents=True)
    syft_out, grype_out = out / "syft.json", out / "grype.json"
    argv = ["syft", "scan", f"dir:{target}", "-o", f"syft-json={syft_out}", "-q"]
    for x in EXCLUDE:
        argv += ["--exclude", x]
    _run(argv)
    _run(["grype", f"sbom:{syft_out}", "-o", "json", "--file", str(grype_out), "-q"])

    syft_doc = json.loads(syft_out.read_text(encoding="utf-8"))
    grype_doc = json.loads(grype_out.read_text(encoding="utf-8"))
    db = ((grype_doc.get("descriptor") or {}).get("db") or {}).get("status") or {}
    if not db.get("valid"):
        raise SystemExit(f"the vulnerability database at {GRYPE_DB} is not valid: {db}")
    comps = components(syft_doc)
    found = matches(grype_doc, comps)
    rev = _target_rev(target)
    meta = {
        "engagement": eng_dir.name,
        "scanned_at": ts,
        "target_rev": rev,
        "excluded": list(EXCLUDE),
        "syft": (syft_doc.get("descriptor") or {}).get("version"),
        "grype": (grype_doc.get("descriptor") or {}).get("version"),
        "db": {"built": db.get("built"), "schema": db.get("schemaVersion"),
               "from": db.get("from")},
        "files": dict(sorted(Counter(f for c in comps for f in c["files"]).items())),
        "counts": {"components": len(comps),
                   "by_type": dict(Counter(c["type"] for c in comps)),
                   "version_range": sum(1 for c in comps if c["version_range"]),
                   "no_licence": sum(1 for c in comps if not c["licences"]),
                   "matches": len(found),
                   "matched_components": len({m["component"] for m in found})},
    }
    for name, doc in (("components.json", comps), ("matches.json", found),
                      ("scan_meta.json", meta)):
        (out / name).write_text(json.dumps(doc, indent=1, ensure_ascii=False) + "\n",
                                encoding="utf-8")
    logger.info("%s: %d components, %d matches in %d components -> %s",
                eng_dir.name, len(comps), len(found),
                meta["counts"]["matched_components"], out)
    return out


def latest(eng_dir: Path) -> Optional[Dict[str, Any]]:
    """The newest scan of an engagement that enables composition analysis, as
    {"dir", "meta", "components", "matches"}, or None."""
    if not enabled(eng_dir):
        return None
    d = eng_dir / SCANS
    scans = sorted(p for p in d.iterdir() if (p / "scan_meta.json").is_file()) if d.is_dir() else []
    if not scans:
        return None
    s = scans[-1]
    load = lambda n: json.loads((s / n).read_text(encoding="utf-8"))   # noqa: E731
    return {"dir": str(s), "meta": load("scan_meta.json"),
            "components": load("components.json"), "matches": load("matches.json")}


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engagement", required=True)
    args = ap.parse_args()
    eng = REPO / "workflowsv2" / "claims_audit" / "engagements" / args.engagement
    if not eng.is_dir():
        raise SystemExit(f"no engagement {args.engagement}")
    if not enabled(eng):
        raise SystemExit(f"{args.engagement}: composition analysis is not enabled "
                         f"(composition: true in engagement.yaml)")
    print(run(eng))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
