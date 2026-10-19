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


# ---- licences ---------------------------------------------------------------------

from workflowsv2.composition import licences                 # noqa: E402


def test_a_licence_is_classed_by_what_it_asks_and_flagged_when_it_asks_more():
    c = licences.classify
    # An OR lets the buyer take the permissive option; an AND imposes all.
    assert c("Apache-2.0 OR LGPL-2.1-or-later OR MIT") == "permissive"
    assert c("MIT AND AGPL-3.0-only") == "network copyleft"
    assert c("(MIT OR Apache-2.0) AND Unicode-3.0") == "permissive"
    assert c("GPL-2.0-only WITH Classpath-exception-2.0") == "strong copyleft"
    assert c("mit") == "permissive"
    assert c("LicenseRef-Proprietary") == "unknown"
    assert c("MIT OR") == "unknown"
    assert c(None) is None
    assert licences._flags("permissive", "MIT OR Apache-2.0", None) == []
    assert licences._flags("strong copyleft", "GPL-3.0-only", None) == ["strong copyleft"]
    assert licences._flags("unknown", "LicenseRef-x", None) == ["identifier not recognised"]
    assert licences._flags(None, None, None) == ["no licence found"]
    assert licences._flags("permissive", "MIT", "Apache-2.0") == \
        ["the dependency file and the registry disagree"]


def _crate(i, name, version, source="registry+https://github.com/rust-lang/crates.io-index"):
    a = _artifact(i, name, version, typ="rust-crate")
    a["purl"] = f"pkg:cargo/{name}@{version}"
    a["metadata"] = {"source": source} if source else {}
    return a


def test_only_public_exact_versions_are_sent_out_and_the_cache_answers_again(tmp_path, monkeypatch):
    monkeypatch.setattr(licences, "CACHE", tmp_path / "licences.json")
    asked = []

    def fake_get(url, limit=0):
        asked.append(url)
        return json.dumps({"licenses": ["MIT"]}).encode()
    monkeypatch.setattr(licences, "_get", fake_get)
    syft = {"artifacts": [
        _crate("a", "serde", "1.0.0"),
        _crate("b", "our-internal-crate", "0.1.0", source="registry+https://crates.example.corp/index"),
        _crate("c", "the-target", "2.0.0", source=None),
        _artifact("d", "pinned-elsewhere", "1.0", constraint=">=1.0")]}
    syft["artifacts"][3]["purl"] = "pkg:pypi/pinned-elsewhere@1.0"
    comps = scan.components(syft)
    licences.complete(syft, comps)
    assert len(asked) == 1 and "serde" in asked[0]
    assert not any(n in u for u in asked for n in ("our-internal-crate", "the-target", "pinned-elsewhere"))
    by = {x["name"]: x["licence"] for x in comps}
    assert by["serde"]["expression"] == "MIT" and by["serde"]["from"] == "deps.dev"
    assert by["the-target"]["from"] == "local" and by["the-target"]["flags"] == []
    assert by["our-internal-crate"]["flags"] == ["not looked up: not from a public registry"]
    asked.clear()
    comps = scan.components(syft)
    counts = licences.complete(syft, comps)
    assert asked == [] and counts["cached"] == 1


def test_the_appendix_leads_with_what_the_licences_ask_and_names_the_flagged():
    comps = scan.components({"artifacts": [_artifact("a", "lib-a", "1.0"), _artifact("b", "lib-b", "2.0"),
                                           _artifact("c", "app", "0.1")]})
    given = {"lib-a": ("MIT OR Apache-2.0", "deps.dev", "permissive", []),
             "lib-b": ("GPL-3.0-only", "deps.dev", "strong copyleft", ["strong copyleft"]),
             "app": (None, "local", None, [])}
    for x in comps:
        expr, frm, cls, flags = given[x["name"]]
        x["licence"] = {"expression": expr, "from": frm, "class": cls, "flags": flags,
                        "at": "2026-10-19T00:00:00+00:00", "url": None, "archive": None}
    md = "\n".join(appendix.render(_scan(comps)))
    assert "Of the 2 third-party components: 1 permissive; 1 strong copyleft." in md
    assert "- lib-b 2.0 (GPL-3.0-only; strong copyleft): **not yet read**." in md
    assert "app is the target's own code" in md
    assert "not recorded" not in md
    lib_b = next(x for x in comps if x["name"] == "lib-b")
    lib_b["licence"].update(expression="MIT", flags=[])
    lib_b["licence"]["class"] = "permissive"
    md = "\n".join(appendix.render(_scan(comps)))
    assert "All 2 third-party components declare permissive licences." in md


def test_a_licence_only_the_archive_declares_is_used_and_clears_the_flag(tmp_path, monkeypatch):
    # xmlhttprequest-ssl 2.1.2: deps.dev has no licence for it, and its
    # package.json declares MIT only in npm's old `licenses` array.
    import io
    import tarfile
    monkeypatch.setattr(licences, "CACHE", tmp_path / "licences.json")
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as tf:
        for name, text in (("package/package.json", json.dumps({"licenses": [{"type": "MIT"}]})),
                           ("package/LICENSE", "Copyright (c) 2010 passive.ly LLC\n\nPermission is hereby granted")):
            data = text.encode()
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tf.addfile(info, io.BytesIO(data))
    pages = {"api.deps.dev": json.dumps({"licenses": []}).encode(),
             "registry.npmjs.org/xmlhttprequest-ssl/": json.dumps({"dist": {"tarball": "https://registry.npmjs.org/x.tgz"}}).encode(),
             "registry.npmjs.org/x.tgz": buf.getvalue()}
    monkeypatch.setattr(licences, "_get", lambda url, limit=0: next(v for k, v in pages.items() if k in url))
    art = _artifact("x", "xmlhttprequest-ssl", "2.1.2", typ="npm")
    art["purl"] = "pkg:npm/xmlhttprequest-ssl@2.1.2"
    art["metadata"] = {"resolved": "https://registry.npmjs.org/xmlhttprequest-ssl/-/xmlhttprequest-ssl-2.1.2.tgz"}
    syft = {"artifacts": [art]}
    comps = scan.components(syft)
    licences.complete(syft, comps)
    lic = comps[0]["licence"]
    assert (lic["expression"], lic["from"], lic["class"], lic["flags"]) == ("MIT", "archive", "permissive", [])
    assert lic["archive"]["files"][0]["first_lines"][0] == "Copyright (c) 2010 passive.ly LLC"


def test_the_summary_and_the_appendix_carry_the_practices_reading():
    from workflowsv2.composition import judgements
    comps = scan.components({"artifacts": [_artifact("a", "lib-a", "1.0"), _artifact("b", "agno", "1.7.6"),
                                           _artifact("c", "fastembed", "0.7.3")]})
    given = {"lib-a": ("MIT", "permissive", []),
             "agno": ("non-standard", "unknown", ["identifier not recognised"]),
             "fastembed": ("non-standard", "unknown", ["identifier not recognised"])}
    for x in comps:
        expr, cls, flags = given[x["name"]]
        x["licence"] = {"expression": expr, "from": "deps.dev", "class": cls, "flags": flags,
                        "at": "2026-10-19T00:00:00+00:00", "url": None, "archive": None}
    s = _scan(comps)
    s["judgements"] = {"agno@1.7.6": {"verdict": "conditions apply", "note": "MPL-2.0 binds changed agno files.",
                                      "by": "p", "at": "t"}}
    line = " ".join(judgements.summary_line(s))
    assert line.startswith("**The components' licences.** 1 of the 3 third-party components declare permissive")
    assert "The practice read the 1 licence the scan flagged: conditions apply to agno." in line
    assert "1 flagged licence is not yet read: fastembed@0.7.3." in line
    md = "\n".join(appendix.render(s))
    assert "- agno 1.7.6 (non-standard; identifier not recognised): **conditions apply**. MPL-2.0" in md
    assert "- fastembed 0.7.3 (non-standard; identifier not recognised): **not yet read**." in md
    assert [r["code"] for r in judgements.worklist_rows(s)] == ["licence_not_read"]
