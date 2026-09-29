"""Reading a translated chapter into the knowledge base's text layer: the
format the translation team is given (TRANSLATION_FORMAT.md), and the earlier
sampler, which is the only real document so far."""
import io
import json
import sys
import zipfile
from pathlib import Path
from xml.sax.saxutils import escape

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2.nidana_kb import ingest as ig                 # noqa: E402

SAMPLER = Path("/data/Sree/sources/Madhava Nidana .docx")


def docx(path: Path, paras):
    """A minimal .docx: each item is text, or (style, text)."""
    body = ""
    for p in paras:
        style, text = p if isinstance(p, tuple) else ("", p)
        ppr = f'<w:pPr><w:pStyle w:val="{style}"/></w:pPr>' if style else ""
        body += f"<w:p>{ppr}<w:r><w:t xml:space=\"preserve\">{escape(text)}</w:t></w:r></w:p>"
    xml = ('<?xml version="1.0" encoding="UTF-8"?><w:document xmlns:w="http://schemas.'
           'openxmlformats.org/wordprocessingml/2006/main"><w:body>' + body + "</w:body></w:document>")
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        z.writestr("word/document.xml", xml)
    path.write_bytes(buf.getvalue())
    return path


CHAPTER_2 = [
    ("Heading1", "Chapter 2 — Jvaranidānam (Fever)"),
    ("Heading2", "Purvarupa — general"),
    "Verse: श्रमोऽरतिर्विवर्णत्वं वैरस्यं नयनप्लवः । इच्छाद्वेषौ मुहुश्चापि शीतवातातपादिषु ॥ ४ ॥",
    "Verse: जृम्भाऽङ्गमर्दो गुरुता रोमहर्षोऽरुचिस्तमः । अप्रहर्षश्च शीतं च भवत्युत्पत्स्यति ज्वरे ॥ ५ ॥",
    "Source: Ca. Ni. 1",
    "IAST: jṛmbhā'ṅgamardo gurutā romaharṣo'rucis tamaḥ | apraharṣaś ca śītaṃ ca bhavaty utpatsyati jvare || 5 ||",
    "IAST: śramo'ratir vivarṇatvaṃ vairasyaṃ nayanaplavaḥ | icchādveṣau muhuś cāpi śītavātātapādiṣu || 4 ||",
    "Words:",
    "śrama — fatigue",
    "jṛmbhā — yawning",
    "Translation: When fever is about to arise, these appear: fatigue, yawning …",
    ("Heading2", "Purvarupa — by dosha"),
    "Verse: सामान्यतो विशेषात्तु जृम्भाऽत्यर्थं समीरणात् । पित्तान्नयनयोर्दाहः कफादन्नारुचिर्भवेत् ॥ ६ ॥",
    "IAST: sāmānyato viśeṣāt tu jṛmbhā'tyarthaṃ samīraṇāt | pittān nayanayor dāhaḥ kaphād annārucir bhavet || 7 ||",
    "Translation: From Vata, excessive yawning; from Pitta, burning of the eyes; from Kapha, aversion to food.",
    ("Heading2", "Other signs"),
    "Translation: Thirst — dryness of the mouth",
]


def test_the_team_format_pairs_each_verse_with_its_own_transliteration(tmp_path):
    rep = ig.ingest(docx(tmp_path / "02_jvaranidanam.docx", CHAPTER_2), tmp_path / "kb")
    doc = json.loads((tmp_path / "kb/text/ch02.json").read_text())
    v = {x["id"]: x for x in doc["verses"]}
    assert list(v) == ["MN.2.4", "MN.2.5", "MN.2.6"]
    # the IAST lines were given out of order; each lands on its numeral
    assert v["MN.2.4"]["iast"].startswith("śramo") and v["MN.2.5"]["iast"].startswith("jṛmbhā")
    assert v["MN.2.4"]["cites"] == v["MN.2.5"]["cites"] == "Ca. Ni. 1"
    first = doc["passages"][0]
    assert first["id"] == "MN.2.4-5" and first["verses"] == ["MN.2.4", "MN.2.5"]
    assert first["words"] == [{"term": "śrama", "en": "fatigue"}, {"term": "jṛmbhā", "en": "yawning"}]
    assert first["translation"].startswith("When fever is about to arise")
    assert rep["written"] == [2]


def test_problems_are_flagged_where_they_are_and_logged(tmp_path):
    ig.ingest(docx(tmp_path / "02.docx", CHAPTER_2), tmp_path / "kb")
    doc = json.loads((tmp_path / "kb/text/ch02.json").read_text())
    by_id = {p["id"]: p for p in doc["passages"]}
    assert [f["code"] for f in by_id["MN.2.6"]["flags"]] == ["numeral_mismatch"]
    orphan = [p for p in doc["passages"] if not p["verses"]]
    assert len(orphan) == 1 and orphan[0]["flags"][0]["code"] == "unanchored"
    logged = [json.loads(l)["code"] for l in (tmp_path / "kb/issues.jsonl").read_text().splitlines()]
    assert sorted(logged) == ["numeral_mismatch", "unanchored"]


def test_a_chapter_from_another_document_is_not_overwritten_without_replace(tmp_path):
    kb = tmp_path / "kb"
    ig.ingest(docx(tmp_path / "a.docx", CHAPTER_2), kb)
    other = docx(tmp_path / "b.docx", CHAPTER_2[:4])
    assert ig.ingest(other, kb)["refused"] == [2]
    assert json.loads((kb / "text/ch02.json").read_text())["source"]["doc"] == "a.docx"
    assert ig.ingest(other, kb, replace=True)["written"] == [2]


@pytest.mark.skipif(not SAMPLER.is_file(), reason="the team's sampler is not on this machine")
def test_the_sampler_reads_by_script_and_its_orphan_lists_are_unanchored(tmp_path):
    rep = ig.ingest(SAMPLER, tmp_path / "kb")
    doc = json.loads((tmp_path / "kb/text/ch02.json").read_text())
    assert [v["id"] for v in doc["verses"]] == ["MN.2.4", "MN.2.5", "MN.2.6", "MN.2.7"]
    assert all(v["iast"] for v in doc["verses"])
    assert [p["id"] for p in doc["passages"]] == ["MN.2.4-5", "MN.2.6-7"]
    # Arshas (chapter 5) is an English list with no verse in the sampler
    assert [p["code"] for p in rep["chapters"][5]["problems"]] == ["unanchored"]


EDITION_3 = Path("/data/Sree/sources/Madhava_Nidanam_Chapters/03_atisaranidanam.docx")


@pytest.mark.skipif(not EDITION_3.is_file(), reason="the team's edition is not on this machine")
def test_each_commentary_sentence_can_be_looked_up_with_its_sanskrit(tmp_path):
    """The back-index: a translation answering the work file's numbered
    segments comes back from the knowledge base as Sanskrit beside English,
    by segment id."""
    from workflowsv2.nidana_kb import compose as cp
    from workflowsv2.nidana_kb.kb import KB
    ed = cp.read_edition(EDITION_3)
    wf = cp.work_file(ed)
    tr = tmp_path / "tr"
    tr.mkdir()
    for i, sec in enumerate(wf["sections"]):
        (tr / f"sec_{i:02d}.json").write_text(json.dumps({
            "heading_en": f"s{i}", "translation": f"verse {i}", "words": [], "notes": [],
            "madhukosa": [{"k": s["k"], "en": f"MK {i}.{s['k']}"} for s in sec["madhukosa"]],
            "atanka": [{"k": s["k"], "en": f"AT {i}.{s['k']}"} for s in sec["atanka"]]}))
    assert cp.check_translation(ed, tr) == []
    cp.compose(ed, tr, 3, "Diarrhoea", tmp_path / "03.docx")
    ig.ingest(tmp_path / "03.docx", tmp_path / "kb")
    kb = KB(tmp_path / "kb")
    sec2 = wf["sections"][2]["madhukosa"]
    sid = next(k for k, v in kb.segments.items() if v["en"] == "MK 2.2")
    assert kb.segment(sid)["sa"] == sec2[1]["sa"] and sid.endswith(":mk2")
    total = sum(len(s["madhukosa"]) + len(s["atanka"]) for s in wf["sections"])
    assert len(kb.segments) == total


def test_a_missing_segment_is_reported(tmp_path):
    from workflowsv2.nidana_kb import compose as cp
    ed = {"title_sa": "", "title_iast": "", "sections": [
        {"head_sa": "x", "head_iast": "x", "mula": {"sa": "", "iast": ""},
         "madhukosa": {"sa": "अ" * 130 + "| " + "ब" * 130 + "|", "iast": ""},
         "atanka": {"sa": "", "iast": ""}}]}
    (tmp_path / "sec_00.json").write_text(json.dumps({"madhukosa": [{"k": 1, "en": "a"}]}))
    assert cp.check_translation(ed, tmp_path) == ["section 0 madhukosa: segments [2] untranslated"]


@pytest.mark.skipif(not EDITION_3.is_file(), reason="the team's edition is not on this machine")
def test_a_section_links_back_to_its_place_in_the_edition_and_shows_its_sanskrit(tmp_path):
    """Per-section link: the lookup for a passage names the edition file and
    section, and gives the verses' Devanagari; the lookup for a passage's
    commentary gives that commentary's Sanskrit beside its English."""
    from workflowsv2.nidana_kb import compose as cp
    from workflowsv2.nidana_kb.kb import KB
    from workflowsv2.ayur_consult.record import lookup
    ed = cp.read_edition(EDITION_3)
    wf = cp.work_file(ed)
    tr = tmp_path / "tr"
    tr.mkdir()
    for i, sec in enumerate(wf["sections"]):
        (tr / f"sec_{i:02d}.json").write_text(json.dumps({
            "heading_en": f"s{i}", "translation": f"verse {i}", "words": [], "notes": [],
            "madhukosa": [{"k": s["k"], "en": f"MK {i}.{s['k']}"} for s in sec["madhukosa"]],
            "atanka": [{"k": s["k"], "en": f"AT {i}.{s['k']}"} for s in sec["atanka"]]}))
    cp.compose(ed, tr, 3, "Diarrhoea", tmp_path / "03.docx")
    ig.ingest(tmp_path / "03.docx", tmp_path / "kb")
    kb = KB(tmp_path / "kb")
    pid = next(p for p, v in kb.passages.items() if v["verses"] and v["translation"] == "verse 2")
    assert kb.passages[pid]["edition"] == "03_atisaranidanam.docx §2"
    shown = lookup(kb, [pid])["text"]
    first_verse = kb.verse(kb.passages[pid]["verses"][0])["sa"]
    assert "03_atisaranidanam.docx §2" in shown and first_verse in shown
    whole = lookup(kb, [pid + ":mk"])["text"]
    assert wf["sections"][2]["madhukosa"][0]["sa"] in whole and "MK 2.1" in whole
