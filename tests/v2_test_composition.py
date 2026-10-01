"""Composition analysis, version 1: the scan record and the report appendix.

The programs (syft, grype) are not run here; their output shapes are."""
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from workflowsv2.composition import appendix, scan          # noqa: E402
from workflowsv2 import engagement_state as state           # noqa: E402


def _artifact(i, name, version, typ="python", constraint=None, licences=()):
    a = {"id": i, "name": name, "version": version, "type": typ, "purl": f"pkg:x/{name}",
         "licenses": [{"value": l, "spdxExpression": l} for l in licences],
         "locations": [{"path": "/requirements.txt"}]}
    if constraint:
        a["metadata"] = {"versionConstraint": constraint}
    return a


def test_a_range_is_listed_with_its_range_and_not_matched():
    syft = {"artifacts": [_artifact("a", "pyyaml", "6.0", constraint=">=6.0"),
                          _artifact("b", "requests", "2.31.0", constraint="==2.31.0",
                                    licences=["Apache-2.0"])]}
    comps = scan.components(syft)
    by = {c["name"]: c for c in comps}
    assert by["pyyaml"]["version"] is None and by["pyyaml"]["version_range"] == ">=6.0"
    assert by["requests"]["version"] == "2.31.0" and by["requests"]["version_range"] is None
    grype = {"matches": [
        {"artifact": {"id": "a", "name": "pyyaml", "version": "6.0"},
         "vulnerability": {"id": "CVE-1", "severity": "High"}, "matchDetails": []},
        {"artifact": {"id": "b", "name": "requests", "version": "2.31.0"},
         "vulnerability": {"id": "CVE-2", "severity": "Low"}, "matchDetails": []}]}
    found = scan.matches(grype, comps)
    assert [m["vulnerability"] for m in found] == ["CVE-2"]


def _scan(comps, matches=0, matched=0, ranged=0, rev="abc123"):
    return {"meta": {"counts": {"components": len(comps), "version_range": ranged,
                                "matches": matches, "matched_components": matched},
                     "files": {"requirements.txt": len(comps)} if comps else {},
                     "syft": "1", "grype": "2", "target_rev": rev,
                     "db": {"built": "2026-09-22T06:30:41Z", "schema": "v6"},
                     "scanned_at": "2026-09-22T21-00-00Z"},
            "components": comps, "matches": []}


def test_the_appendix_gives_matches_as_a_count_and_says_what_a_match_is():
    comps = scan.components({"artifacts": [_artifact("b", "requests", "2.31.0",
                                                     licences=["Apache-2.0"])]})
    md = "\n".join(appendix.render(_scan(comps, matches=3, matched=1)))
    assert "3 published vulnerabilities match 1 of the 1 components" in md
    assert "not a finding about the target" in md
    assert "CVE" not in md                       # the list stays in the record
    assert "Apache-2.0 1" in md


def test_no_dependency_file_gives_no_table_and_no_claims_about_components():
    md = "\n".join(appendix.render(_scan([], rev=None)))
    assert "No dependency file the scanning program recognises." in md
    assert "| kind |" not in md and "Known vulnerabilities" not in md
    assert "not a git checkout" in md


def test_the_flag_is_settable_and_read_as_a_boolean(tmp_path):
    eng = tmp_path / "e"
    eng.mkdir()
    (eng / "engagement.yaml").write_text("target: target\n")
    assert scan.enabled(eng) is False and scan.latest(eng) is None
    state.update_engagement(eng, composition=True)
    assert "composition: true" in (eng / "engagement.yaml").read_text()
    assert scan.enabled(eng) is True
    assert scan.latest(eng) is None              # enabled, not yet scanned


SECRET = "Zx8vQ2mN4pL7rT1wK9sB3dF6hJ0cY5aE"


def _git(repo, *a):
    import subprocess
    subprocess.run(["git", "-C", str(repo), "-c", "user.name=t", "-c", "user.email=t@t",
                    *a], check=True, capture_output=True)


def test_secrets_the_seller_exempts_or_removed_are_still_reported_without_values(tmp_path):
    """The target is the seller's: its own gitleaks settings and allow
    comments must not hide a match, a credential deleted in a later commit is
    still reported, and no value reaches the record."""
    import shutil
    import pytest
    if not (shutil.which("gitleaks") and shutil.which("syft") and shutil.which("grype")
            and scan.GITLEAKS_RULES.is_file()):
        pytest.skip("gitleaks, syft, grype or the pinned rules are not installed")
    target = tmp_path / "target"
    target.mkdir()
    _git(target, "init", "-q")
    (target / "old.py").write_text(f'api_key = "{SECRET}"\n')
    _git(target, "add", "."); _git(target, "commit", "-qm", "one")
    (target / "old.py").unlink()
    (target / "settings.py").write_text(f'auth_token = "{SECRET[::-1]}"  # gitleaks:allow\n')
    (target / ".gitleaks.toml").write_text(
        '[extend]\nuseDefault = true\n[[allowlists]]\npaths = [".*"]\n')
    import subprocess
    first = subprocess.run(["git", "-C", str(target), "rev-parse", "HEAD"],
                           capture_output=True, text=True).stdout.strip()
    (target / ".gitleaksignore").write_text(f"{first}:old.py:generic-api-key:1\n")
    _git(target, "add", "-A"); _git(target, "commit", "-qm", "two")
    eng = tmp_path / "e"
    eng.mkdir()
    (eng / "engagement.yaml").write_text(f"target: {target}\ncomposition: true\n")

    out = scan.run(eng)
    rows = scan.latest(eng)["secrets"]
    assert ("files", "settings.py") in {(r["where"], r["file"]) for r in rows}
    assert ("history", "old.py") in {(r["where"], r["file"]) for r in rows}
    for f in out.iterdir():
        text = f.read_text(encoding="utf-8")
        assert SECRET not in text and SECRET[::-1] not in text, f.name
