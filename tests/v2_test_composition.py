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
