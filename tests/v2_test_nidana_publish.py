"""The browsable site: every id in the knowledge base is a link target on its
chapter page, every commentary segment shows its Sanskrit beside its
English, and every page carries the draft notice."""
import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2.nidana_kb import kb as kbmod                   # noqa: E402
from workflowsv2.nidana_kb import publish_html as pub           # noqa: E402


def chapter(n):
    return {
        "chapter": n, "title": f"अध्यायः {n}", "source": {"doc": f"{n:02d}.docx"},
        "verses": [{"id": f"MN.{n}.1", "n": "1", "sa": "अ ॥ १ ॥", "iast": "a || 1 ||", "cites": "Ca. Ni. 1"},
                   {"id": f"MN.{n}.2", "n": "2", "sa": "आ ॥ २ ॥", "iast": "ā || 2 ||", "cites": ""}],
        "passages": [{"id": f"MN.{n}.1-2", "section": "s", "verses": [f"MN.{n}.1", f"MN.{n}.2"],
                      "translation": "Two verses <b>.", "words": [], "padaccheda": "", "notes": [],
                      "edition": f"{n:02d}.docx §0",
                      "commentary": {"madhukosha": [
                          {"k": 1, "id": f"MN.{n}.1-2:mk1", "sa": "मधुकोश एक", "en": "Madhukośa one"},
                          {"k": 2, "id": f"MN.{n}.1-2:mk2", "sa": "मधुकोश द्वे", "en": "Madhukośa two"}],
                          "atankadarpana": [
                          {"k": 1, "id": f"MN.{n}.1-2:at1", "sa": "आतङ्क", "en": "Ātaṅka one"}]}},
                     {"id": f"MN.{n}.u2", "section": "colophon", "verses": [], "translation": "",
                      "edition": f"{n:02d}.docx §1",
                      "commentary": {"madhukosha": [{"k": None, "id": f"MN.{n}.u2:mk1", "sa": "इति", "en": "Thus"}]}}]}


def test_every_id_is_an_anchor_and_every_segment_shows_both_languages(tmp_path):
    kb, tr, out = tmp_path / "kb", tmp_path / "tr", tmp_path / "site"
    for n in (1, 2):
        kbmod.write_json(kbmod.text_path(kb, n), chapter(n))
        (tr / f"ch{n:02d}").mkdir(parents=True)
        (tr / f"ch{n:02d}" / "sec_00.json").write_text(json.dumps(
            {"heading_en": "Heading", "notes": ["a note"],
             "review": {"changes": [{"where": "translation", "type": "addition",
                                     "before": "x", "after": "[x]", "why": "added"}]}}))
    counts = pub.publish(kb, tr, out, "team@example.com")
    assert counts == {"verses": 4, "sections": 4, "segments": 8, "pages": 2}
    for n in (1, 2):
        page = (out / f"ch{n:02d}.html").read_text()
        ids = set(re.findall(r'id="([^"]+)"', page))
        doc = chapter(n)
        want = ({v["id"] for v in doc["verses"]} | {p["id"] for p in doc["passages"]}
                | {s["id"] for p in doc["passages"] for segs in p["commentary"].values() for s in segs})
        assert want <= ids
        row = re.search(rf'<div class="mn-seg" id="MN\.{n}\.1-2:mk2">(.*?)</div></div>', page, re.S).group(1)
        assert "मधुकोश द्वे" in row and "Madhukośa two" in row
        assert "Two verses &lt;b&gt;." in page                   # text is escaped, not injected
        assert "Review corrections (1)" in page and "a note" in page
    for p in out.glob("*.html"):
        text = p.read_text()
        assert 'class="mn-draft"' in text and 'content="noindex,nofollow"' in text
