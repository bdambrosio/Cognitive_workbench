"""The outreach runner: prompts, citation checks, what is kept and flagged.

    python3 -m pytest tests/v2_test_outreach.py -q
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2.outreach import runner, schemas                     # noqa: E402

BODY = ["Source: https://example.com/post", "Retrieved: 2026-09-20", "---",
        "Several of our sellers are entering diligence at the same time.",
        "Technical diligence costs more than these deals can carry."]
CAND = {"name": "Jane Smith", "firm": "Acme Advisers", "relationship": "introduced by a friend"}


class Backend:
    def resolved_model(self):
        return "fake-model"


def _fake(objs, seen):
    """emit replaced: returns the next of `objs` and records each prompt."""
    objs = list(objs)

    def fake_emit(loop, system, user, schema, max_tokens, salvage=None):
        seen.append({"system": system, "user": user, "schema": schema})
        return {"obj": objs.pop(0), "parse": "parsed", "parse_error": None, "raw": "{}"}
    return fake_emit


def _cite(quote, lines=(4, 4), file="01_page.md"):
    return {"file": file, "lines": list(lines), "quote": quote}


def test_citation_kept_dropped_and_misplaced():
    ev = {"01_page.md": BODY}
    kept, dropped = schemas.check_citations([
        _cite("Several of our sellers are entering diligence"),
        _cite("Technical diligence costs more", lines=(4, 4)),
        _cite("We love Tuuyi"),
        _cite("entering diligence", file="99_none.md"),
        _cite("   4|Several of our sellers are entering diligence at the same time."),
    ], ev)
    assert [k["at_lines"] for k in kept] == [True, False, True]
    assert kept[2]["quote"].startswith("Several")          # the line-number display is stripped
    assert [d["why"][:12] for d in dropped] == ["the quote is", "no evidence "]


def test_typographic_quotes_in_the_page_do_not_fail_a_true_quote():
    ev = {"01_page.md": ["The firm\u2019s \u201cfocus\u201d \u2014 technical diligence\u2026"]}
    kept, dropped = schemas.check_citations(
        [_cite("The firm's \"focus\" - technical diligence...", lines=(1, 1))], ev)
    assert len(kept) == 1 and kept[0]["at_lines"] and not dropped


def _qualification(category="strong", now=None):
    return {"answers": [{"question": n, "answer": "No evidence.", "citations": []}
                        for n in range(1, 10)],
            "prospect_type": "M&A adviser", "relationship": "Referral",
            "problem_recognition": "Strong", "category": category,
            "why_person": "Advises sellers of software companies.",
            "why_now": "Wrote that several sellers are entering diligence.",
            "why_now_citations": now if now is not None else
            [_cite("Several of our sellers are entering diligence")],
            "use_case": "A review before diligence.", "concerns": "", "reject_reason": ""}


def _prepare(tmp_path):
    ev = tmp_path / "evidence"
    ev.mkdir(exist_ok=True)
    (ev / "01_page.md").write_text("\n".join(BODY) + "\n")
    (tmp_path / "research.json").write_text(json.dumps({"files": [
        {"file": "01_page.md", "source": "https://example.com/post", "found_by": "given", "words": 20}]}))


def test_qualify_prompt_and_record(tmp_path, monkeypatch):
    _prepare(tmp_path)
    seen = []
    monkeypatch.setattr(runner, "emit", _fake([_qualification()], seen))
    rec = runner.qualify(Backend(), CAND, tmp_path, contact="sent 2026-09-01")
    user = seen[0]["user"]
    assert "PROSPECT.md §14" in user and "introduced by a friend" in user
    assert "sent 2026-09-01" in user and "   4|Several of our sellers" in user
    assert "## 21." not in seen[0]["system"]                 # practice sections do not reach the model
    assert rec["category"] == "strong" and rec["attempts"] == 1 and not rec["flags"]
    assert json.loads((tmp_path / "qualification.json").read_text())["model"] == "fake-model"


def test_strong_without_a_checked_citation_is_asked_again_then_flagged(tmp_path, monkeypatch):
    _prepare(tmp_path)
    seen = []
    bad = _qualification(now=[_cite("He told me he needs Tuuyi")])
    monkeypatch.setattr(runner, "emit", _fake([bad, bad], seen))
    rec = runner.qualify(Backend(), CAND, tmp_path)
    assert len(seen) == 2 and "could not be used" in seen[1]["user"]
    assert rec["category"] == "strong"                       # code never changes the category
    assert any("no checked citation" in f for f in rec["flags"])
    assert len(rec["citations_dropped"]) == 1
    assert "CHECK:" in runner.brief(CAND, tmp_path)


def test_draft_only_for_strong_and_flags(tmp_path, monkeypatch):
    _prepare(tmp_path)
    seen = []
    monkeypatch.setattr(runner, "emit", _fake([_qualification("plausible")], seen))
    runner.qualify(Backend(), CAND, tmp_path)
    assert runner.draft(Backend(), CAND, tmp_path) is None and len(seen) == 1

    monkeypatch.setattr(runner, "emit", _fake([_qualification(), {
        "angle": "Review before diligence.", "message": "word " * 120,
        "rests_on": [], "assumes": ""}], seen))
    runner.qualify(Backend(), CAND, tmp_path)
    d = runner.draft(Backend(), CAND, tmp_path)
    assert "PROSPECT.md §15" in seen[-1]["user"]
    assert len(d["flags"]) == 2                              # no checked citation, and too long
    assert "Suggested message" in runner.brief(CAND, tmp_path)


def test_research_saves_pages_and_leaves_out_another_person(tmp_path, monkeypatch):
    seen = []
    monkeypatch.setattr(runner, "emit", _fake([
        {"queries": ["Jane Smith Acme Advisers SaaS"], "reason": "recent activity"},
        {"about_candidate": "no", "reason": "a dentist of the same name"},
        {"about_candidate": "yes", "reason": "names Acme Advisers"}], seen))
    monkeypatch.setattr(runner, "page_text", lambda url: None)
    monkeypatch.setattr(runner.tavily_client, "search", lambda q, n: [
        {"url": "https://a.example/dentist", "title": "Dentist", "raw_content": "tooth " * 30},
        {"url": "https://b.example/deal", "title": "Deal", "raw_content": "Acme Advisers sold a SaaS firm. " * 8},
        {"url": "https://c.example/thin", "title": "Thin", "raw_content": "too short"}])
    cand = {**CAND, "urls": ["https://www.linkedin.com/in/jane", "https://acme.example/team"],
            "pasted": [{"source": "LinkedIn post", "date": "2026-09-15", "text": "We have four deals in diligence."}]}
    rec = runner.research(Backend(), cand, tmp_path)
    assert [f["found_by"][:6] for f in rec["files"]] == ["pasted", "search", "search"]
    assert [f["file"] for f in runner.kept_files(rec)] == [rec["files"][0]["file"], rec["files"][2]["file"]]
    assert "PROSPECT.md §12" in seen[0]["user"] and "PROSPECT.md §13" in seen[1]["user"]
    first = (tmp_path / "evidence" / rec["files"][0]["file"]).read_text()
    assert first.startswith("Source: LinkedIn post") and "four deals" in first
    assert "unreadable_page" in (tmp_path / "issues.jsonl").read_text()


# ---- attio.py, against a fake HTTP session: no live calls ----------------------

class _Resp:
    def __init__(self, body, status=200):
        self._body, self.status_code, self.text = body, status, str(body)

    def json(self):
        return self._body


def _attio(monkeypatch, answers, calls):
    from workflowsv2.outreach import attio

    def request(method, url, json=None, params=None, timeout=None, headers=None):
        calls.append({"method": method, "path": url.replace(attio.API, ""), "json": json, "params": params})
        return answers.pop(0)
    monkeypatch.setenv("ATTIO_API_KEY", "k")
    monkeypatch.setattr(attio.requests, "request", request)
    return attio


def test_attio_contact_record_reads_stage_tasks_and_notes(monkeypatch):
    calls = []
    attio = _attio(monkeypatch, [
        _Resp({"data": [{"list_api_slug": "tuuyi_outreach", "entry_id": "e1"},
                        {"list_api_slug": "another", "entry_id": "e2"}]}),
        _Resp({"data": {"entry_values": {"stage": [{"status": {"title": "Initial sent"}}],
                                         "last_contact": [{"value": "2026-09-16"}]}}}),
        _Resp({"data": [{"is_completed": False, "deadline_at": "2026-09-23T00:00:00Z",
                         "content_plaintext": "Follow up  if no\nresponse"}]}),
        _Resp({"data": [{"created_at": "2026-09-14T01:00:00Z", "title": "Proposed Tuuyi outreach"}]})], calls)
    text = attio.contact_record({"id": {"record_id": "r1"}})
    assert "stage 'Initial sent', last contact 2026-09-16" in text
    assert "Task (open, due 2026-09-23): Follow up if no response" in text
    assert "Note of 2026-09-14: Proposed Tuuyi outreach" in text
    assert [c["method"] for c in calls] == ["GET"] * 4          # reading writes nothing


def test_attio_two_people_of_one_name_is_an_error_and_writes_have_the_documented_shape(monkeypatch):
    import pytest
    calls = []
    attio = _attio(monkeypatch, [_Resp({"data": [{}, {}]}), _Resp({"data": {"id": {}}}),
                                 _Resp({"data": {}}), _Resp({"detail": "no"}, 403)], calls)
    with pytest.raises(attio.AttioError):
        attio.find_person("Jane Smith")
    attio.upsert_entry("r1", {"stage": "Ready to contact"})
    assert calls[1]["method"] == "PUT" and calls[1]["path"] == "/lists/tuuyi_outreach/entries"
    assert calls[1]["json"]["data"] == {"parent_object": "people", "parent_record_id": "r1",
                                        "entry_values": {"stage": "Ready to contact"}}
    attio.create_note("r1", "Outreach brief", "# x")
    assert calls[2]["json"]["data"]["format"] == "markdown"
    with pytest.raises(attio.AttioError):
        attio.create_person("Jane Smith")


def _fake_attio(monkeypatch, wrote, person=None, entry_stage=None):
    a = runner.attio
    monkeypatch.setattr(a, "find_person", lambda name: person)
    monkeypatch.setattr(a, "create_person",
                        lambda name, title, li: wrote.append(("person", name, li)) or {"id": {"record_id": "r1"}})
    monkeypatch.setattr(a, "entry_of", lambda rid: None if entry_stage is None else
                        {"entry_values": {"stage": [{"status": {"title": entry_stage}}]}})
    monkeypatch.setattr(a, "not_pursuing", lambda rid, reason, on: wrote.append(("not pursuing", rid, reason)))
    monkeypatch.setattr(a, "upsert_entry",
                        lambda rid, v: wrote.append(("entry", rid, v)) or {"id": {"entry_id": "e1"}})
    monkeypatch.setattr(a, "create_note",
                        lambda rid, title, md: wrote.append(("note", title, md)) or {"id": {"note_id": "n1"}})


def _qualified(tmp_path, monkeypatch, category, with_draft=True):
    _prepare(tmp_path)
    for f in ("attio.json", "draft.json"):
        (tmp_path / f).unlink(missing_ok=True)
    drafted = {"angle": "a", "message": "Jane, a short message." if with_draft else "", "assumes": "",
               "rests_on": [_cite("Several of our sellers are entering diligence")]}
    monkeypatch.setattr(runner, "emit", _fake([_qualification(category), drafted, drafted], []))   # an empty draft is asked for twice
    runner.qualify(Backend(), CAND, tmp_path)
    runner.draft(Backend(), CAND, tmp_path)


def test_push_writes_a_strong_candidate_once(tmp_path, monkeypatch):
    wrote = []
    _fake_attio(monkeypatch, wrote)
    _qualified(tmp_path, monkeypatch, "strong")
    rec = runner.push(CAND, tmp_path)
    assert [w[0] for w in wrote] == ["person", "entry", "note"]
    v = wrote[1][2]
    assert v["stage"] == "Ready to contact" and v["category"] == "M&A adviser"
    assert v["relationship"] == "Referral" and v["probelm_recognition"] == "Strong"
    assert "Jane, a short message." in wrote[2][2] and rec["person_created"]
    assert runner.push(CAND, tmp_path) is None and len(wrote) == 3    # not pushed twice


def test_push_leaves_alone_what_is_not_its_to_write(tmp_path, monkeypatch):
    known = {"id": {"record_id": "r9"}}
    wrote = []
    _fake_attio(monkeypatch, wrote)                                   # nobody in Attio by that name
    _qualified(tmp_path, monkeypatch, "plausible")
    assert runner.push(CAND, tmp_path) is None and not wrote          # not strong: no new person
    _fake_attio(monkeypatch, wrote, person=known, entry_stage="Initial sent")
    _qualified(tmp_path, monkeypatch, "strong")
    assert runner.push(CAND, tmp_path) is None and not wrote          # a person set that stage
    _qualified(tmp_path, monkeypatch, "strong", with_draft=False)
    _fake_attio(monkeypatch, wrote, person=known, entry_stage="Research")
    assert runner.push(CAND, tmp_path) is None and not wrote          # strong with no draft


def test_push_moves_a_research_entry_by_category(tmp_path, monkeypatch):
    known = {"id": {"record_id": "r9"}}
    wrote = []
    _fake_attio(monkeypatch, wrote, person=known, entry_stage="Research")
    _qualified(tmp_path, monkeypatch, "plausible")
    runner.push(CAND, tmp_path)
    assert [w[0] for w in wrote] == ["entry", "note"] and wrote[0][2]["stage"] == "Qualified"
    assert "next_action" not in wrote[0][2]
    wrote.clear()
    _qualified(tmp_path, monkeypatch, "reject")
    rec = runner.push(CAND, tmp_path)
    assert [w[0] for w in wrote] == ["not pursuing"] and wrote[0][2].startswith("Qualified as reject.")
    assert rec["not_pursuing"] and runner.push(CAND, tmp_path) is None   # once


def test_candidate_from_an_attio_entry(monkeypatch):
    calls = []
    attio = _attio(monkeypatch, [
        _Resp({"data": {"values": {"name": [{"full_name": "Jane Smith"}], "job_title": [{"value": "Partner"}],
                                   "company": [{"target_record_id": "c1"}],
                                   "linkedin": [{"value": "https://www.linkedin.com/in/jane"}]}}}),
        _Resp({"data": {"values": {"name": [{"value": "Acme Advisers"}], "domains": [{"domain": "acme.example"}]}}}),
        _Resp({"data": [{"title": "Evidence: LinkedIn post", "created_at": "2026-09-15T10:00:00Z",
                         "content_plaintext": "Four deals in diligence."},
                        {"title": "Outreach brief 2026-09-01", "content_plaintext": "old"}]})], calls)
    c = attio.candidate_from({"parent_record_id": "r1", "entry_values": {
        "notes": [{"value": "Met at a conference in May."}],
        "relationship": [{"option": {"title": "Warm"}}]}})
    assert c["name"] == "Jane Smith" and c["firm"] == "Acme Advisers" and c["title"] == "Partner"
    assert c["urls"] == ["https://acme.example", "https://www.linkedin.com/in/jane"]
    assert "Warm" in c["relationship"] and "conference in May" in c["relationship"]
    assert c["pasted"] == [{"source": "LinkedIn post", "date": "2026-09-15", "text": "Four deals in diligence."}]


def test_scout_skips_known_names_looks_once_and_limits_research(tmp_path, monkeypatch):
    profile = "M&A adviser to founder-led SaaS companies. " * 6
    monkeypatch.setattr(runner.exa, "search", lambda q, cat: [
        {"title": "Ann Known", "url": "https://www.linkedin.com/in/ann", "text": profile},
        {"title": "Bob New", "url": "https://www.linkedin.com/in/bob", "text": profile, "publishedDate": "2026-08-01T00:00:00Z"},
        {"title": "Cy Student", "url": "https://www.linkedin.com/in/cy", "text": "Finance student. " * 12},
        {"title": "Di New", "url": "https://www.linkedin.com/in/di", "text": profile}])
    monkeypatch.setattr(runner.attio, "known", lambda name: name == "Ann Known")
    seen = []
    monkeypatch.setattr(runner, "emit", _fake([
        {"queries": ["sell-side advisers to bootstrapped vertical SaaS founders"], "reason": "first search"},
        {"fits": "yes", "prospect_type": "M&A adviser", "reason": "says so"},
        {"fits": "no", "prospect_type": "none", "reason": "a student"},
        {"fits": "yes", "prospect_type": "M&A adviser", "reason": "says so"}], seen))
    got = runner.scout(Backend(), "M&A adviser", tmp_path, want=1)
    assert [c["name"] for c in got] == ["Bob New"]                    # the limit; Di New waits
    assert "PROSPECT.md §16" in seen[0]["user"] and "PROSPECT.md §17" in seen[1]["user"]
    assert not (tmp_path / "ann_known").exists() and (tmp_path / "cy_student" / "first_look.json").is_file()
    assert got[0]["pasted"][0]["date"] == "2026-08-01" and "linkedin.com/in/bob" in got[0]["pasted"][0]["source"]
    (tmp_path / "bob_new" / "research.json").write_text("{}")
    monkeypatch.setattr(runner.exa, "search", lambda q, cat: (_ for _ in ()).throw(AssertionError("searched")))
    assert [c["name"] for c in runner.scout(Backend(), "M&A adviser", tmp_path, want=1)] == ["Di New"]
    assert "vertical SaaS" in (tmp_path / "scout_log.jsonl").read_text()


def test_firm_scout_picks_one_checked_person_per_firm(tmp_path, monkeypatch):
    firm_text = "Acme Software Group buys small vertical software companies and holds them. " * 4
    head = "Head of M&A at Acme Software Group. Runs acquisitions of vertical software companies. " * 3
    def search(q, category="people", *a):
        if category == "company":
            return [{"title": "Acme Software Group", "url": "https://acme.example/", "text": firm_text},
                    {"title": "Known Holdings", "url": "https://known.example/", "text": firm_text}]
        return [{"title": "Pat Head", "url": "https://www.linkedin.com/in/pat", "text": head},
                {"title": "Sam Former", "url": "https://www.linkedin.com/in/sam", "text": "Formerly at Acme. " * 10}]
    monkeypatch.setattr(runner.exa, "search", search)
    monkeypatch.setattr(runner.attio, "firm_record", lambda firm, domain: {} if firm == "Known Holdings" else None)
    monkeypatch.setattr(runner.attio, "known", lambda name: False)
    seen = []
    whom = {"first": "01_pat_head.md", "first_role": "Head of M&A", "alternate": "02_sam_former.md",
            "first_citation": {"file": "01_pat_head.md", "lines": [4, 4], "quote": "Head of M&A at Acme Software Group."},
            "reason": "Runs the acquisitions."}
    monkeypatch.setattr(runner, "emit", _fake([
        {"queries": ["holding companies that buy small vertical software businesses"], "reason": "first"},
        {"fits": "yes", "prospect_type": "Repeat acquirer", "reason": "buys and holds"}, whom], seen))
    got = runner.scout_firms(Backend(), "Repeat acquirer", tmp_path, want=3)
    assert [c["name"] for c in got] == ["Pat Head"] and got[0]["firm"] == "Acme Software Group"
    assert "Alternate at the firm" in got[0]["notes"] and "Sam Former" in got[0]["notes"]
    assert "PROSPECT.md §18" in seen[0]["user"] and "§19" in seen[1]["user"] and "§20" in seen[2]["user"]
    assert not (tmp_path / "_firms" / "known_holdings").exists()          # Attio has that firm
    firm = json.loads((tmp_path / "_firms" / "acme_software_group" / "firm.json").read_text())
    assert firm["chosen"] == "Pat Head" and firm["alternate"] == "Sam Former"
    assert runner.firm_domain(got[0]) == "acme.example"

    # a choice whose citation is not in the chosen profile makes no candidate
    monkeypatch.setattr(runner.exa, "search", lambda q, category="people", *a: (
        [{"title": "Beta Holdings", "url": "https://beta.example/", "text": firm_text}] if category == "company"
        else [{"title": "Lee Other", "url": "https://www.linkedin.com/in/lee", "text": head}]))
    bad = {**whom, "first": "01_lee_other.md", "alternate": "",
           "first_citation": {"file": "01_lee_other.md", "lines": [4, 4], "quote": "Head of M&A at Beta Holdings."}}
    monkeypatch.setattr(runner, "emit", _fake([
        {"queries": ["another description"], "reason": "second"},
        {"fits": "yes", "prospect_type": "Repeat acquirer", "reason": "buys"}, bad], []))
    (tmp_path / "pat_head" / "research.json").write_text("{}")
    assert runner.scout_firms(Backend(), "Repeat acquirer", tmp_path, want=3) == []
    assert "did not match" in json.loads((tmp_path / "_firms" / "beta_holdings" / "firm.json").read_text())["chosen_reason"]


def test_a_better_contact_named_in_qualification_waits_for_research(tmp_path, monkeypatch):
    cand = {**CAND, "urls": ["https://acme.example/team", "https://www.linkedin.com/in/jane"]}
    q = {"better_contact_name": "Ravi Diligence", "better_contact_role": "VP R&D, M&A", "prospect_type": "Repeat acquirer"}
    d = runner.better_contact(cand, q, tmp_path, use_attio=False)
    made = runner.waiting(tmp_path, "Repeat acquirer")
    assert d is not None and [m["name"] for m in made] == ["Ravi Diligence"]
    assert made[0]["firm"] == "Acme Advisers" and made[0]["urls"] == ["https://acme.example/team"]
    assert runner.better_contact(cand, q, tmp_path, use_attio=False) is None     # once
    assert runner.better_contact(cand, {"better_contact_name": ""}, tmp_path, use_attio=False) is None


def test_attio_contact_for_someone_unknown_reports_colleagues_at_the_firm(monkeypatch):
    calls = []
    attio = _attio(monkeypatch, [
        _Resp({"data": []}),                                                       # no such person
        _Resp({"data": [{"values": {"team": [{"target_record_id": "r2"}]}}]}),      # the firm, by domain
        _Resp({"data": {"values": {"name": [{"full_name": "Pat Head"}]}}}),
        _Resp({"data": [{"list_api_slug": "tuuyi_outreach", "entry_id": "e2"}]}),
        _Resp({"data": {"entry_values": {"stage": [{"status": {"title": "Initial sent"}}]}}})], calls)
    text = attio.contact_for("Ravi Diligence", "Acme Software Group", "acme.example")
    assert text == "A colleague at the same firm, Pat Head, is in the outreach list at stage 'Initial sent'."
    assert calls[1]["json"]["filter"] == {"domains": "acme.example"}


def test_daily_works_the_named_then_scouts_the_least_scouted_kind_when_the_pool_is_low(tmp_path, monkeypatch):
    did = []
    named = [{"name": "Ann Named"}]
    monkeypatch.setattr(runner, "pickup", lambda data: named)
    monkeypatch.setattr(runner, "work", lambda backend, cands, stages, data, use_attio, redo=False:
                        did.append(("work", [c["name"] for c in cands], stages[-1], use_attio)))
    ready = [{"entry_values": {"stage": [{"status": {"title": "Ready to contact"}}]}}] * 2
    monkeypatch.setattr(runner.attio, "entries", lambda: ready)
    monkeypatch.setattr(runner, "scout", lambda b, kind, data, want: did.append(("scout", kind, want)) or [{"name": "Sue Scouted"}])
    monkeypatch.setattr(runner, "scout_firms", lambda b, kind, data, want: did.append(("firms", kind, want)) or [])
    monkeypatch.setattr(runner, "next_kind", lambda: "Repeat acquirer")
    text = runner.daily(Backend(), tmp_path, pool=5, want=2)
    assert did == [("work", ["Ann Named"], "push", True), ("work", [], "push", True),
                   ("firms", "Repeat acquirer", 2), ("work", [], "push", True)]
    assert "Scouted for: Repeat acquirer, by firm." in text and "Ready to contact now: 2." in text
    did.clear()
    runner.daily(Backend(), tmp_path, pool=2, want=2)                 # the pool is full: no scouting
    assert [d[0] for d in did] == ["work", "work"]


def test_attio_not_pursuing_writes_the_note_then_removes_the_entry(monkeypatch):
    calls = []
    attio = _attio(monkeypatch, [
        _Resp({"data": [{"list_api_slug": "tuuyi_outreach", "entry_id": "e1"}]}),
        _Resp({"data": {"id": {"entry_id": "e1"}, "entry_values": {
            "stage": [{"status": {"title": "Ready to contact"}}], "fit_rationale": [{"value": "Advises sellers."}]}}}),
        _Resp({"data": {"id": {"note_id": "n1"}}}), _Resp({})], calls)
    attio.not_pursuing("r1", "Skipped by the practice. Too large.", "2026-09-21")
    note = calls[2]["json"]["data"]
    assert note["title"] == "Not pursuing 2026-09-21" and "Too large." in note["content"]
    assert "Ready to contact" in note["content"] and "Advises sellers." in note["content"]
    assert (calls[3]["method"], calls[3]["path"]) == ("DELETE", "/lists/tuuyi_outreach/entries/e1")


def test_the_daily_kind_takes_turns_by_date():
    import datetime
    kinds = [runner.next_kind(datetime.date(2026, 9, 21) + datetime.timedelta(days=i)) for i in range(6)]
    assert sorted(kinds) == sorted(runner.DAILY_KINDS)


def test_an_empty_draft_is_asked_for_once_more(tmp_path, monkeypatch):
    _prepare(tmp_path)
    empty = {"angle": "", "message": "", "rests_on": [], "assumes": ""}
    good = {"angle": "a", "message": "Jane, a short message.", "assumes": "",
            "rests_on": [_cite("Several of our sellers are entering diligence")]}
    seen = []
    monkeypatch.setattr(runner, "emit", _fake([_qualification(), empty, good], seen))
    runner.qualify(Backend(), CAND, tmp_path)
    d = runner.draft(Backend(), CAND, tmp_path)
    assert len(seen) == 3 and d["message"].startswith("Jane") and not d["flags"]
    monkeypatch.setattr(runner, "emit", _fake([empty, empty], seen))
    assert "the draft returned nothing usable" in runner.draft(Backend(), CAND, tmp_path)["flags"]
