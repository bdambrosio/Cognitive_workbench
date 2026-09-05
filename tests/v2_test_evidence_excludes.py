"""Documentation is not evidence (METHOD §7): the engagement's exclusion list,
enforced at the evidence subagent's primitives, recorded by the output
check, and marked for the reviewer."""
import json
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workflowsv2 import engagement_state as st                  # noqa: E402
from workflowsv2.claims_audit import schemas as sch            # noqa: E402
from chat.subagents import code_subagent as cs                  # noqa: E402


def _repo(tmp_path):
    root = tmp_path / "target"; root.mkdir()
    (root / "README.md").write_text("# Tool\nIt records only the hit.\n")
    (root / "docs").mkdir(); (root / "docs" / "SECURITY.md").write_text("Hardened by default.\n")
    (root / "src").mkdir(); (root / "src" / "main.rs").write_text("fn main() {\n  log_hit();\n}\n")
    (root / "LICENSE").write_text("MIT License\n")
    subprocess.run(["git", "-C", str(root), "init", "-q"], check=True)
    subprocess.run(["git", "-C", str(root), "add", "-A"], check=True)
    subprocess.run(["git", "-C", str(root), "-c", "user.email=t@t", "-c", "user.name=t",
                    "commit", "-q", "-m", "x"], check=True)
    return root


def test_excluded_matching():
    ex = ["README.md", "docs/"]
    assert cs._excluded("README.md", ex) and cs._excluded("docs/SECURITY.md", ex)
    assert cs._excluded("./docs/a/b.md", ex)
    assert not cs._excluded("src/main.rs", ex) and not cs._excluded("docs2/x.md", ex)
    assert not cs._excluded("README.md", None) and not cs._excluded("README.md", [])
    assert sch.doc_excluded("docs/SECURITY.md", ex) and not sch.doc_excluded("LICENSE", ex)


def test_primitives_refuse_documentation_but_list_it(tmp_path):
    pytest.importorskip("shutil").which("git") or pytest.skip("no git")
    root = _repo(tmp_path)
    sub = cs.CodeSubagent(root, None, tmp_path / "traces", mode="external",
                          excludes=["README.md", "docs/"])
    prims = sub.primitives()
    listing = prims["list"]({"path": ""})
    assert "README.md" in listing and "(excluded: documentation, not evidence)" in listing
    assert listing.count("excluded") == 2                          # README.md and docs/
    assert "LICENSE" in listing and "src/" in listing
    r = prims["read"]({"file": "README.md"})
    assert r.startswith("ERROR: read refused") and "documentation" in r
    assert prims["read"]({"file": "docs/SECURITY.md"}).startswith("ERROR: read refused")
    assert prims["read"]({"file": "LICENSE"}).startswith("OK:")
    assert prims["read"]({"file": "src/main.rs"}).startswith("OK:")
    c = prims["cite"]({"file": "README.md", "start_line": 1, "end_line": 2})
    assert c.startswith("ERROR: read refused") and sub.answer_suffix() == ""
    assert prims["cite"]({"file": "src/main.rs", "start_line": 2, "end_line": 2}).startswith("OK: cited")
    if pytest.importorskip("shutil").which("rg"):
        g = prims["grep"]({"pattern": "hit"})
        assert "src/main.rs" in g and "README.md" not in g
    assert "Excluded from evidence" in sub.system_prompt()
    # no excludes: everything readable, prompt unchanged
    plain = cs.CodeSubagent(root, None, tmp_path / "traces2", mode="external")
    assert plain.primitives()["read"]({"file": "README.md"}).startswith("OK:")
    assert "Excluded from evidence" not in plain.system_prompt()


def test_engagement_default_and_explicit_excludes(tmp_path):
    eng = st.new_engagement(tmp_path / "e")
    (eng / "engagement.yaml").write_text("target: target\nclaim_sources: [README.md, llms.txt]\nclient_emails: []\n")
    assert st.evidence_excludes(eng) == ["README.md", "llms.txt"]        # default: every claim source
    st.update_engagement(eng, evidence_excludes=["README.md", "docs/"])
    assert st.evidence_excludes(eng) == ["README.md", "docs/"]
    st.update_engagement(eng, evidence_excludes=[])
    assert st.evidence_excludes(eng) == []                               # explicit empty excludes nothing


def test_output_check_records_citations_into_excluded_documents(tmp_path):
    root = _repo(tmp_path)
    frozen = [{"id": 1, "quote": "It records only the hit.", "lines": [2, 2], "statement": "s", "about": "target"}]
    findings = {"claim_source": "README.md", "findings": [{
        "claim_id": 1, "adjudication": {"verdict": "real"},
        "evidence": [{"form": "citation", "document": "README.md", "lines": [2, 2],
                      "quote": "It records only the hit.", "shows": "restates"},
                     {"form": "citation", "document": "src/main.rs", "lines": [2, 2],
                      "quote": "log_hit();", "shows": "logs the hit"}]}]}
    res = sch.check_output(findings, root, "README.md", frozen, excludes=["README.md", "docs/"])
    ex = res["figures"]["excluded_citations"]
    assert len(ex) == 1 and ex[0]["document"] == "README.md"
    # recorded, not failed: the review judges relevance with the mark in view
    assert not any("excluded" in p for p in res["problems"])
    res0 = sch.check_output(findings, root, "README.md", frozen)
    assert res0["figures"]["excluded_citations"] == []


def test_files_read_tolerates_a_path_that_is_prose(tmp_path):
    from workflowsv2.claims_audit.runner import files_read, files_matched
    root = _repo(tmp_path)
    traces = tmp_path / "traces"; traces.mkdir()
    junk = "frontend/src/" + "previous turn was injected noise " * 12
    (traces / "inspect_external_1.txt").write_text(
        json.dumps({"tool": "read", "file": junk}) + "\n"
        + json.dumps({"tool": "read", "file": "src/main.rs"}) + "\n"
        + f"OK: {junk}:3:x\nOK: src/main.rs:2:  log_hit();\n")
    assert list(files_read(traces, root)) == ["src/main.rs"]
    assert files_matched(traces, root) == ["src/main.rs"]
