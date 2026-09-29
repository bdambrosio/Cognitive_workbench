"""The consultation's code paths with the model stubbed: extraction keeps
only citations the text holds, the case record keeps each finding's match
across re-emissions, a finding is matched only to a feature it was offered,
and the program's assessment is frozen before anyone reviews it."""
import json
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2.ayur_consult import runner as rn               # noqa: E402
from workflowsv2.ayur_consult import schemas as sch             # noqa: E402
from workflowsv2.nidana_kb import extract as ex                  # noqa: E402
from workflowsv2.nidana_kb import kb as kbmod                    # noqa: E402


def text_kb(root: Path) -> Path:
    kbmod.write_json(kbmod.text_path(root, 2), {
        "chapter": 2, "title": "Chapter 2", "source": {"doc": "t.docx"},
        "verses": [{"id": "MN.2.6", "n": "6", "section": "by dosha", "sa": "…",
                    "iast": "sāmānyato viśeṣāt tu jṛmbhā'tyarthaṃ samīraṇāt | pittān nayanayor dāhaḥ",
                    "cites": "", "flags": []}],
        "passages": [{"id": "MN.2.6", "section": "by dosha", "verses": ["MN.2.6"], "words": [],
                      "translation": "From Vata, excessive yawning; from Pitta, burning of the eyes."}]})
    return root


def answer(features, new=()):
    return {"obj": {"diseases": [{"id": "jvara", "names": {"sa": "ज्वर", "iast": "jvara", "en": "fever"},
                                  "variants": [{"id": "jvara.vata", "doshas": ["vata"], "label": "Vata"},
                                               {"id": "jvara.pitta", "doshas": ["pitta"], "label": "Pitta"}],
                                  "features": features}],
                    "new_features": list(new), "notes": []}, "parse": "parsed", "raw": "{}"}


NEW = [{"id": "F.excess_yawning", "en": "excessive yawning", "clinical": "yawning", "sa": ["jṛmbhā"],
        "lay_question": "Yawning a lot?", "observable": ["reported"]},
       {"id": "F.burning_eyes", "en": "burning eyes", "clinical": "ocular burning", "sa": ["nayana dāha"],
        "lay_question": "Do your eyes burn?", "observable": ["reported"]}]


def test_extraction_keeps_only_citations_the_verse_holds(tmp_path):
    kb_dir = text_kb(tmp_path / "kb")
    feats = [
        {"feature": "F.excess_yawning", "role": "purvarupa", "variant": "jvara.vata",
         "weight": "cardinal", "verses": ["MN.2.6"], "quote": "jṛmbhā'tyarthaṃ"},
        {"feature": "F.burning_eyes", "role": "purvarupa", "variant": "jvara.pitta",
         "weight": "cardinal", "verses": ["MN.2.6"], "quote": "burning of the eyes"},  # English, not the verse
        {"feature": "F.fatigue", "role": "purvarupa", "variant": "",
         "weight": "supporting", "verses": ["MN.2.99"], "quote": ""},                 # no such verse
    ]
    s = ex.extract_chapter(kb_dir, 2, lambda *a: answer(feats, NEW), "METHOD")
    d = json.loads((kb_dir / "clinical/ch02.json").read_text())["diseases"][0]
    status = {f["feature"]: (f["status"], f["verses"]) for f in d["features"]}
    assert status["F.excess_yawning"] == ("draft", ["MN.2.6"])
    assert status["F.burning_eyes"] == ("uncited", [])
    assert status["F.fatigue"] == ("uncited", [])
    assert s["uncited"] == 2
    assert set(json.loads((kb_dir / "lexicon.json").read_text())) == {"F.excess_yawning", "F.burning_eyes"}
    logged = (kb_dir / "issues.jsonl").read_text()
    assert "MN.2.99 is not in the knowledge base" in logged and "F.fatigue" in logged


def test_the_case_record_keeps_a_findings_match_while_its_words_are_unchanged():
    prev = sch.empty_form()
    prev["findings"] = [{"text": "Yawning a lot", "status": "present", "source": "reported",
                         "when": "", "feature_id": "F.excess_yawning", "matched": True}]
    new = sch.empty_form()
    new["findings"] = [{"text": "yawning  a lot", "status": "present", "source": "reported", "when": ""},
                       {"text": "eyes burn", "status": "present", "source": "reported", "when": ""}]
    res = rn.fill_form(lambda *a: {"obj": new}, "M", [("practitioner", "…")], prev, 100)
    got = {f["text"]: (f["feature_id"], f["matched"]) for f in res["form"]["findings"]}
    assert got == {"yawning  a lot": ("F.excess_yawning", True), "eyes burn": (None, False)}
    kept = rn.fill_form(lambda *a: {"obj": None, "parse_error": "x"}, "M", [], prev, 100)
    assert kept["form"] is prev and not kept["updated"]


class FakeKB:
    lexicon = {"F.burning_eyes": {"en": "burning eyes"}, "F.redness": {"en": "red eyes"},
               "F.fever": {"en": "fever"}}

    def candidates(self, text, k=6):
        return [dict(id=i, **self.lexicon[i]) for i in ("F.burning_eyes", "F.redness")]


def test_a_finding_is_matched_only_to_a_feature_it_was_offered():
    form = sch.empty_form()
    form["findings"] = [{"text": "eyes burn", "status": "present", "feature_id": None, "matched": False},
                        {"text": "hot body", "status": "present", "feature_id": None, "matched": False}]
    picks = {"mappings": [{"finding": "eyes burn", "feature_id": "F.burning_eyes", "why": ""},
                          {"finding": "hot body", "feature_id": "F.fever", "why": "not offered"}]}
    left = rn.map_findings(lambda *a: {"obj": picks}, FakeKB(), form)
    assert [f["feature_id"] for f in form["findings"]] == ["F.burning_eyes", None]
    # an answer outside the candidates is not a judgement: asked again next turn
    assert left == ["hot body"] and [f["matched"] for f in form["findings"]] == [True, False]


def test_a_finding_the_answer_leaves_out_is_asked_again():
    form = sch.empty_form()
    form["findings"] = [{"text": "eyes burn", "status": "present", "feature_id": None, "matched": False}]
    rn.map_findings(lambda *a: {"obj": {"mappings": [{"finding": "very tired", "feature_id": "", "why": ""}]}},
                    FakeKB(), form)
    assert form["findings"][0]["matched"] is False


def test_review_is_refused_until_the_programs_assessment_is_frozen(tmp_path, monkeypatch):
    monkeypatch.setenv("AYUR_CASES", str(tmp_path / "cases"))
    kb = kbmod.KB(text_kb(tmp_path / "kb"))
    case = rn.cases_root() / "c1"
    rn.new_case(case, "R-17", [{"name": "Asha", "role": "intern"}], kb.version)
    filled = tmp_path / "review.yaml"
    filled.write_text(yaml.safe_dump({"system_candidates": [], "diagnosis": {"kb_id": "", "text": "jvara"}}))
    with pytest.raises(SystemExit, match="not finished"):
        rn.review(case, kb, filled, "Prof. Rao", "professor")
    rn.finish(case, kb, "Asha")
    with pytest.raises(SystemExit, match="frozen once"):
        rn.finish(case, kb, "Asha")
    out = rn.review(case, kb, filled, "Prof. Rao", "professor")
    rec = json.loads(out.read_text())
    assert rec["by"] == "Prof. Rao" and rec["role"] == "professor"
    assert out.name == "review_professor_prof-rao.json"
