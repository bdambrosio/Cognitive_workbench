"""The outreach runner: prompts, citation checks, what is kept and flagged.

    python3 -m pytest tests/v2_test_outreach.py -q
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import pytest                                                         # noqa: E402

from workflowsv2.outreach import contacts, runner, schemas           # noqa: E402

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
    assert "## 23." in seen[0]["system"] and "## 24." not in seen[0]["system"]   # practice sections do not reach the model
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


# ---- contacts.py, on files in a temporary folder --------------------------------

@pytest.fixture
def store(tmp_path, monkeypatch):
    monkeypatch.setattr(contacts, "DIR", tmp_path / "contacts")
    return contacts


def _person(name, stage=None, next_date="", firm="", domain="", **entry):
    """A contact, in the outreach list at `stage` when one is given. Returns the id."""
    c = contacts.create_person(name, firm=firm, domain=domain)
    if stage:
        contacts.upsert_entry(c["id"], {"stage": stage, "next_action_date": next_date, **entry})
    return c["id"]


def test_contact_record_reads_stage_tasks_and_notes(store):
    cid = _person("Jane Smith", "Initial sent", last_contact="2026-09-16")
    c = store.get(cid)
    c["tasks"] = [{"text": "Follow up  if no\nresponse", "due": "2026-09-23", "done": False}]
    store._write(c)
    store.create_note(cid, "Proposed Tuuyi outreach", "text")
    text = store.contact_record(store.get(cid))
    assert "stage 'Initial sent', last contact 2026-09-16" in text
    assert "Task (open, due 2026-09-23): Follow up if no response" in text
    assert f"Note of {runner.today()}: Proposed Tuuyi outreach" in text


def test_one_contact_per_name_and_only_known_fields_are_written(store):
    _person("Jane Smith")
    assert store.find_person("jane  SMITH")["name"] == "Jane Smith" and store.known("Jane Smith")
    with pytest.raises(store.ContactError):
        store.create_person("Jane Smith")
    with pytest.raises(store.ContactError):
        store.upsert_entry("jane_smith", {"stagee": "Ready to contact"})
    with pytest.raises(store.ContactError):
        store.update_person("jane_smith", {"name": "Janet Smith"})       # the runner files records under the name
    store.update_person("jane_smith", {"email": " jane@acme.example "})
    assert store.get("jane_smith")["email"] == "jane@acme.example"


def _add_notes(n):
    for i in range(n):
        contacts.create_note("jane_smith", "Note", str(i))


def test_two_writers_at_once_lose_no_note(store):
    # the page and the runner are separate processes writing the same file
    import multiprocessing
    _person("Jane Smith")
    ctx = multiprocessing.get_context("fork")
    procs = [ctx.Process(target=_add_notes, args=(40,)) for _ in range(2)]
    for p in procs:
        p.start()
    for p in procs:
        p.join()
    assert len(store.get("jane_smith")["notes"]) == 80


def _qualified(tmp_path, monkeypatch, category, with_draft=True):
    _prepare(tmp_path)
    for f in ("pushed.json", "draft.json"):
        (tmp_path / f).unlink(missing_ok=True)
    drafted = {"angle": "a", "message": "Jane, a short message." if with_draft else "", "assumes": "",
               "rests_on": [_cite("Several of our sellers are entering diligence")]}
    monkeypatch.setattr(runner, "emit", _fake([_qualification(category), drafted, drafted], []))   # an empty draft is asked for twice
    runner.qualify(Backend(), CAND, tmp_path)
    runner.draft(Backend(), CAND, tmp_path)


def test_push_writes_a_strong_candidate_once(tmp_path, monkeypatch, store):
    _qualified(tmp_path, monkeypatch, "strong")
    rec = runner.push(CAND, tmp_path)
    c = store.get("jane_smith")
    e = c["entry"]
    assert e["stage"] == "Ready to contact" and e["category"] == "M&A adviser"
    assert e["relationship"] == "Referral" and e["problem_recognition"] == "Strong"
    assert c["firm"] == "Acme Advisers" and rec["person_created"]
    assert [n["title"][:14] for n in c["notes"]] == ["Outreach brief"] and "Jane, a short message." in c["notes"][0]["text"]
    assert runner.push(CAND, tmp_path) is None and len(store.get("jane_smith")["notes"]) == 1   # not pushed twice


def test_push_leaves_alone_what_is_not_its_to_write(tmp_path, monkeypatch, store):
    _qualified(tmp_path, monkeypatch, "plausible")
    assert runner.push(CAND, tmp_path) is None and not store.known("Jane Smith")   # not strong: no new person
    _person("Jane Smith", "Initial sent")
    _qualified(tmp_path, monkeypatch, "strong")
    assert runner.push(CAND, tmp_path) is None                        # a person set that stage
    assert store.stage_of(store.get("jane_smith")) == "Initial sent" and not store.get("jane_smith")["notes"]
    store.upsert_entry("jane_smith", {"stage": "Research"})
    _qualified(tmp_path, monkeypatch, "strong", with_draft=False)
    assert runner.push(CAND, tmp_path) is None                        # strong with no draft
    assert store.stage_of(store.get("jane_smith")) == "Research"


def test_push_moves_a_research_entry_by_category(tmp_path, monkeypatch, store):
    _person("Jane Smith", "Research")
    _qualified(tmp_path, monkeypatch, "plausible")
    runner.push(CAND, tmp_path)
    c = store.get("jane_smith")
    assert c["entry"]["stage"] == "Qualified" and not c["entry"]["next_action"]
    assert [n["title"][:14] for n in c["notes"]] == ["Outreach brief"]
    store.upsert_entry("jane_smith", {"stage": "Research"})
    _qualified(tmp_path, monkeypatch, "reject")
    rec = runner.push(CAND, tmp_path)
    c = store.get("jane_smith")
    assert c["entry"] is None and c["notes"][-1]["title"].startswith("Not pursuing")
    assert c["notes"][-1]["text"].startswith("Qualified as reject.")
    assert rec["not_pursuing"] and runner.push(CAND, tmp_path) is None   # once


def test_candidate_from_a_contact(store):
    c = store.create_person("Jane Smith", "Partner", "https://www.linkedin.com/in/jane",
                            firm="Acme Advisers", domain="acme.example")
    store.upsert_entry(c["id"], {"stage": "Research", "notes": "Met at a conference in May.", "relationship": "Warm"})
    store.create_note(c["id"], "Evidence: LinkedIn post", "Four deals in diligence.")
    store.create_note(c["id"], "Outreach brief 2026-09-01", "old")
    got = store.candidate_from(store.get(c["id"]))
    assert got["name"] == "Jane Smith" and got["firm"] == "Acme Advisers" and got["title"] == "Partner"
    assert got["urls"] == ["https://acme.example", "https://www.linkedin.com/in/jane"]
    assert "Warm" in got["relationship"] and "conference in May" in got["relationship"]
    assert got["pasted"] == [{"source": "LinkedIn post", "date": runner.today(), "text": "Four deals in diligence."}]


def test_scout_skips_known_names_looks_once_and_limits_research(tmp_path, monkeypatch, store):
    profile = "M&A adviser to founder-led SaaS companies. " * 6
    monkeypatch.setattr(runner.exa, "search", lambda q, cat: [
        {"title": "Ann Known", "url": "https://www.linkedin.com/in/ann", "text": profile},
        {"title": "Bob New", "url": "https://www.linkedin.com/in/bob", "text": profile, "publishedDate": "2026-08-01T00:00:00Z"},
        {"title": "Cy Student", "url": "https://www.linkedin.com/in/cy", "text": "Finance student. " * 12},
        {"title": "Di New", "url": "https://www.linkedin.com/in/di", "text": profile}])
    _person("Ann Known")
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


def test_firm_scout_picks_one_checked_person_per_firm(tmp_path, monkeypatch, store):
    firm_text = "Acme Software Group buys small vertical software companies and holds them. " * 4
    head = "Head of M&A at Acme Software Group. Runs acquisitions of vertical software companies. " * 3
    def search(q, category="people", *a):
        if category == "company":
            return [{"title": "Acme Software Group", "url": "https://acme.example/", "text": firm_text},
                    {"title": "Known Holdings", "url": "https://known.example/", "text": firm_text}]
        return [{"title": "Pat Head", "url": "https://www.linkedin.com/in/pat", "text": head},
                {"title": "Sam Former", "url": "https://www.linkedin.com/in/sam", "text": "Formerly at Acme. " * 10}]
    monkeypatch.setattr(runner.exa, "search", search)
    _person("Kim Known", firm="Known Holdings")
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
    assert not (tmp_path / "_firms" / "known_holdings").exists()          # a contact works there
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
    d = runner.better_contact(cand, q, tmp_path, use_contacts=False)
    made = runner.waiting(tmp_path, "Repeat acquirer")
    assert d is not None and [m["name"] for m in made] == ["Ravi Diligence"]
    assert made[0]["firm"] == "Acme Advisers" and made[0]["urls"] == ["https://acme.example/team"]
    assert runner.better_contact(cand, q, tmp_path, use_contacts=False) is None     # once
    assert runner.better_contact(cand, {"better_contact_name": ""}, tmp_path, use_contacts=False) is None


def test_contact_for_someone_unknown_reports_colleagues_at_the_firm(store):
    _person("Pat Head", "Initial sent", domain="acme.example")
    _person("Lee Elsewhere", "Initial sent", firm="Other Group")
    text = store.contact_for("Ravi Diligence", "Acme Software Group", "acme.example")
    assert text == "A colleague at the same firm, Pat Head, is in the outreach list at stage 'Initial sent'."


def test_daily_works_the_named_then_scouts_the_least_scouted_kind_when_the_pool_is_low(tmp_path, monkeypatch, store):
    did = []
    named = [{"name": "Ann Named"}]
    monkeypatch.setattr(runner, "pickup", lambda data: named)
    monkeypatch.setattr(runner, "work", lambda backend, writer, cands, stages, data, use_contacts, redo=False:
                        did.append(("work", [c["name"] for c in cands], stages[-1], use_contacts)))
    _person("Ann Ready", "Ready to contact")
    _person("Bob Ready", "Ready to contact")
    monkeypatch.setattr(runner, "scout", lambda b, kind, data, want: did.append(("scout", kind, want)) or [{"name": "Sue Scouted"}])
    monkeypatch.setattr(runner, "scout_firms", lambda b, kind, data, want: did.append(("firms", kind, want)) or [])
    monkeypatch.setattr(runner, "next_kind", lambda: "Repeat acquirer")
    text = runner.daily(Backend(), Backend(), tmp_path, pool=5, want=2)
    assert did == [("work", ["Ann Named"], "push", True), ("work", [], "push", True),
                   ("firms", "Repeat acquirer", 2), ("work", [], "push", True)]
    assert "Scouted for: Repeat acquirer, by firm." in text and "Ready to contact now: 2." in text
    did.clear()
    runner.daily(Backend(), Backend(), tmp_path, pool=2, want=2)                 # the pool is full: no scouting
    assert [d[0] for d in did] == ["work", "work"]


def test_not_pursuing_writes_the_note_then_removes_the_entry(store):
    _person("Jane Smith", "Ready to contact", fit_rationale="Advises sellers.")
    store.not_pursuing("jane_smith", "Skipped by the practice. Too large.", "2026-09-21")
    c = store.get("jane_smith")
    assert c["entry"] is None and store.known("Jane Smith")          # kept, so scouting does not bring her back
    note = c["notes"][-1]
    assert note["title"] == "Not pursuing 2026-09-21" and "Too large." in note["text"]
    assert "Ready to contact" in note["text"] and "Advises sellers." in note["text"]


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


# ---- after a message is sent ---------------------------------------------------

def test_due_and_note_texts(store):
    def c(stage, next_date=""):
        return {"entry": {"stage": stage, "next_action_date": next_date}}
    assert store.due(c("Initial sent", "2026-09-21"), "Initial sent", "2026-09-21")
    assert not store.due(c("Initial sent", "2026-09-27"), "Initial sent", "2026-09-21")
    assert store.due(c("Initial sent"), "Initial sent", "2026-09-21")          # no date: nobody said when
    assert not store.due(c("Follow-up sent"), "Initial sent", "2026-09-21")
    among = [{"note_id": "n2", "title": "Message sent 2026-09-21", "date": "2026-09-21", "text": " second "},
             {"note_id": "n0", "title": "Outreach brief 2026-09-20", "date": "2026-09-20", "text": "brief"},
             {"note_id": "n1", "title": "Message sent 2026-09-02", "date": "2026-09-02", "text": "first"}]
    assert store.note_texts("r1", store.SENT_NOTE, among) == [
        {"note_id": "n1", "date": "2026-09-02", "text": "first"},
        {"note_id": "n2", "date": "2026-09-21", "text": "second"}]


def test_followup_prompt_record_and_flags(tmp_path, monkeypatch):
    seen = []
    empty = {"idea": "", "message": "", "assumes": ""}
    good = {"idea": "The data-room comparison.", "message": "Jane, one more thought. " * 3, "assumes": ""}
    monkeypatch.setattr(runner, "emit", _fake([empty, good], seen))
    rec = runner.followup(Backend(), CAND, tmp_path, "In the outreach list at stage 'Initial sent'.",
                          "The first message, sent 2026-09-15:\n\nJane, I read your post.")
    assert len(seen) == 2 and "PROSPECT.md §21" in seen[0]["user"]              # an empty one is asked for again
    assert "Jane, I read your post." in seen[0]["user"] and "stage 'Initial sent'" in seen[0]["user"]
    assert rec["first_message_recorded"] and not rec["flags"]
    assert json.loads((tmp_path / "followup.json").read_text())["message"].startswith("Jane, one more")
    monkeypatch.setattr(runner, "emit", _fake([{**good, "message": "word " * 70}], seen))
    rec = runner.followup(Backend(), CAND, tmp_path, "", "")
    assert "its text was not recorded" in seen[-1]["user"] and not rec["first_message_recorded"]
    assert rec["flags"] == ["the follow-up is 70 words"]


def test_followups_are_drafted_once_for_those_whose_date_has_come(tmp_path, monkeypatch, store):
    monkeypatch.setattr(runner, "today", lambda: "2026-09-21")
    _person("Ann Due", "Initial sent", "2026-09-21")
    _person("Bo Later", "Initial sent", "2026-09-27")
    _person("Cy Ready", "Ready to contact")
    store.create_note("ann_due", "Message sent 2026-09-15", "Ann, I read your post.")
    seen = []
    monkeypatch.setattr(runner, "emit", _fake([{"idea": "i", "message": "Ann, one more thought.", "assumes": ""}], seen))
    assert runner.followups(Backend(), tmp_path) == ["Ann Due"]
    assert "stage 'Initial sent'" in seen[0]["user"] and "Ann, I read your post." in seen[0]["user"]
    assert (tmp_path / "ann_due" / "candidate.yaml").is_file()                 # someone the workflow had no record of
    assert runner.followups(Backend(), tmp_path) == [] and len(seen) == 1      # once


def test_a_reply_is_read_once_and_its_quotes_are_checked(tmp_path, monkeypatch, store):
    reply = "Thanks Bruce.\nHonestly our diligence provider already covers this.\nTalk to Pat Head at Acme, she buys small SaaS."
    _person("Jane Smith", "Replied", firm="Acme Advisers")
    _person("Bo Waiting", "Initial sent")
    store.create_note("jane_smith", "Message sent 2026-09-21", "Jane, I read your post.")
    got = store.create_note("jane_smith", "Reply received 2026-09-24", reply)
    seen = []
    monkeypatch.setattr(runner, "emit", _fake([{"gives": [
        {"what": "objection", "quote": "our diligence provider already covers this", "note": "Says diligence covers it."},
        {"what": "introduction", "quote": "You should meet Pat Head", "note": "Suggests Pat Head at Acme."},
        {"what": "applause", "quote": "Thanks", "note": "not a kind"}],
        "introduced_name": "Pat Head", "next_step": "Thank her; do not argue. Approach Pat Head as a referral."},
        {"takes_up": "The objection and the introduction.", "message": "Jane, thank you. May I use your name with Pat?",
         "assumes": ""}], seen))
    assert runner.replies(Backend(), Backend(), tmp_path) == ["Jane Smith"]
    user = seen[0]["user"]
    assert "PROSPECT.md §22" in user and "   2|Honestly our diligence" in user and "Jane, I read your post." in user
    rec = json.loads((tmp_path / "jane_smith" / "replies.json").read_text())[0]
    assert [(g["what"], g["quote_found"]) for g in rec["gives"]] == [("objection", True), ("introduction", False)]
    assert rec["flags"] == ["the words quoted for `introduction` are not in the reply"]
    assert rec["note_id"] == got["note_id"]
    c = store.get("jane_smith")
    read = c["notes"][-1]
    assert read["title"].startswith("Reply read") and "objection, introduction" in read["title"]
    assert "Introduces: Pat Head" in read["text"] and "CHECK:" in read["text"]
    assert c["entry"]["next_action"] == rec["next_step"] and c["entry"]["next_action_date"] == runner.today()
    assert "PROSPECT.md §23" in seen[1]["user"] and '"what": "objection"' in seen[1]["user"]
    assert "   2|Honestly our diligence" in seen[1]["user"]
    assert rec["answer"]["message"].startswith("Jane, thank you.") and not rec["answer"]["flags"]
    assert runner.replies(Backend(), Backend(), tmp_path) == [] and len(seen) == 2         # read once, answered once

    # a reply read before answers existed gets its answer on the next run
    log = tmp_path / "jane_smith" / "replies.json"
    have = json.loads(log.read_text())
    del have[0]["answer"]
    log.write_text(json.dumps(have))
    monkeypatch.setattr(runner, "emit", _fake([{"takes_up": "t", "message": "word " * 95, "assumes": ""}], seen))
    assert runner.replies(Backend(), Backend(), tmp_path) == ["Jane Smith"]
    assert len(store.get("jane_smith")["notes"]) == 3                                     # nothing more written to the contact
    assert json.loads(log.read_text())[0]["answer"]["flags"] == ["the answer is 95 words"]


def test_the_page_records_a_follow_up_and_a_reply(tmp_path, monkeypatch, store):
    from workflowsv2.outreach import app
    started = []
    _person("Jane Smith", "Initial sent")
    monkeypatch.setattr(app, "today", lambda: "2026-09-27")
    app.followup_sent(app.Sent(record_id="jane_smith", name="Jane Smith", message="Jane, one more thought."))
    c = store.get("jane_smith")
    assert (c["notes"][0]["title"], c["notes"][0]["text"]) == ("Follow-up sent 2026-09-27", "Jane, one more thought.")
    assert c["entry"]["stage"] == "Follow-up sent" and c["entry"]["next_action"] == "Close if no response"

    monkeypatch.setattr(app, "_running", lambda: False)
    monkeypatch.setattr(app, "_start", lambda steps, what: started.append(steps[0][2]))
    store.create_note("jane_smith", "Reply received 2026-09-26", "Already  here.")
    with pytest.raises(app.HTTPException) as refused:                            # the same words a second time
        app.replied(app.Replied(record_id="jane_smith", reply="Already here.\n"))
    assert refused.value.status_code == 409 and len(store.get("jane_smith")["notes"]) == 2
    assert app.replied(app.Replied(record_id="jane_smith", reply=" Not for us. "))["reading"]
    c = store.get("jane_smith")
    assert (c["notes"][-1]["title"], c["notes"][-1]["text"]) == ("Reply received 2026-09-27", "Not for us.")
    assert c["entry"]["stage"] == "Replied" and started == ["replies"]
    monkeypatch.setattr(app, "_running", lambda: True)                           # a run is going: read later
    store.upsert_entry("jane_smith", {"stage": "Conversation"})
    assert not app.replied(app.Replied(record_id="jane_smith", reply="And another thing."))["reading"]
    assert store.stage_of(store.get("jane_smith")) == "Conversation" and started == ["replies"]   # a conversation stays one

    monkeypatch.setattr(app.runner, "DATA", tmp_path)
    (tmp_path / "jane_smith").mkdir()
    (tmp_path / "jane_smith" / "followup.json").write_text(json.dumps({"message": "model's text"}))
    app.edit(app.Edit(name="Jane Smith", message="my text", which="followup"))
    saved = json.loads((tmp_path / "jane_smith" / "followup.json").read_text())
    assert saved["message"] == "model's text" and saved["edited_message"] == "my text"


def test_the_page_records_an_answer_and_keeps_an_edit(tmp_path, monkeypatch, store):
    from workflowsv2.outreach import app
    _person("Jane Smith", "Replied")
    monkeypatch.setattr(app, "today", lambda: "2026-09-21")
    monkeypatch.setattr(app.runner, "DATA", tmp_path)
    (tmp_path / "jane_smith").mkdir()
    log = tmp_path / "jane_smith" / "replies.json"
    log.write_text(json.dumps([{"note_id": "n7", "answer": {"message": "model's answer"}}]))
    app.edit(app.Edit(name="Jane Smith", message="my answer", which="answer", note_id="n7"))
    assert json.loads(log.read_text())[0]["edited_answer"] == "my answer"
    app.answer_sent(app.Answered(record_id="jane_smith", name="Jane Smith", note_id="n7", message="my answer"))
    c = store.get("jane_smith")
    assert (c["notes"][0]["title"], c["notes"][0]["text"]) == ("Answer sent 2026-09-21", "my answer")
    assert c["entry"]["stage"] == "Conversation"
    rec = json.loads(log.read_text())[0]
    assert rec["answer_sent"] == "2026-09-21" and rec["answer"]["message"] == "model's answer"
