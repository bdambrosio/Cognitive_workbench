"""The differential and next questions are arithmetic over the knowledge
base; these fix the orderings a practitioner would check by hand."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2.ayur_consult import differential as dx        # noqa: E402


def feat(fid, role="rupa", weight="supporting", verses=("MN.1.1",), status="draft"):
    return {"feature": fid, "role": role, "weight": weight, "verses": list(verses), "status": status}


# Three variants of a fever and one diarrhoea, shaped like Madhava Nidana 2.4–7
# and 3.5: shared prodromal signs, one cardinal sign per dosha.
GENERAL = [feat("F.fatigue", "purvarupa"), feat("F.yawning", "purvarupa"),
           feat("F.chills", "purvarupa")]
VARIANTS = [
    {"id": "jvara.vata", "disease": "jvara", "names": {"en": "fever"}, "doshas": ["vata"],
     "features": GENERAL + [feat("F.excess_yawning", "purvarupa", "cardinal")]},
    {"id": "jvara.pitta", "disease": "jvara", "names": {"en": "fever"}, "doshas": ["pitta"],
     "features": GENERAL + [feat("F.burning_eyes", "purvarupa", "cardinal")]},
    {"id": "jvara.kapha", "disease": "jvara", "names": {"en": "fever"}, "doshas": ["kapha"],
     "features": GENERAL + [feat("F.food_aversion", "purvarupa", "cardinal")]},
    {"id": "atisara", "disease": "atisara", "names": {"en": "diarrhoea"}, "doshas": [],
     "features": [feat("F.fatigue", "purvarupa"), feat("F.bloating", "purvarupa"),
                  feat("F.retained_stool", "purvarupa")]},
]


def present(*fids):
    return [{"text": f, "feature_id": f, "status": "present"} for f in fids]


def test_the_cardinal_sign_decides_between_forms_of_the_same_disease():
    r = dx.rank(VARIANTS, present("F.fatigue", "F.chills", "F.burning_eyes"))
    assert r[0]["id"] == "jvara.pitta"
    assert {s["feature"] for s in r[0]["supporting"]} == {"F.fatigue", "F.chills", "F.burning_eyes"}


def test_an_absent_cardinal_sign_counts_against_its_variant():
    base = present("F.fatigue", "F.chills")
    tied = {x["id"]: x["score"] for x in dx.rank(VARIANTS, base)}
    assert tied["jvara.vata"] == tied["jvara.pitta"]
    r = {x["id"]: x["score"] for x in dx.rank(
        VARIANTS, base + [{"text": "no burning", "feature_id": "F.burning_eyes", "status": "absent"}])}
    assert r["jvara.pitta"] < r["jvara.vata"]


def test_the_confirmed_vikriti_breaks_a_tie_toward_its_dosha():
    r = dx.rank(VARIANTS, present("F.fatigue", "F.chills"), vikriti=["kapha"])
    assert r[0]["id"] == "jvara.kapha"


def test_uncited_features_do_not_count_and_unmatched_variants_are_not_listed():
    v = [{"id": "x", "disease": "x", "names": {}, "doshas": [],
          "features": [feat("F.a", verses=(), status="uncited")]}]
    assert dx.rank(v, present("F.a")) == []
    assert "atisara" not in {x["id"] for x in dx.rank(VARIANTS, present("F.burning_eyes"))}


def test_the_next_question_is_the_one_that_separates_the_leaders():
    found = present("F.fatigue", "F.chills", "F.yawning")
    r = dx.rank(VARIANTS, found)
    q = dx.next_questions(r, VARIANTS, found)
    asked = [x["feature"] for x in q]
    # the three cardinal signs each split the fevers; a shared sign splits nothing
    assert set(asked) == {"F.excess_yawning", "F.burning_eyes", "F.food_aversion"}
    assert not {"F.fatigue", "F.chills", "F.yawning"} & set(asked)


def test_an_arishta_sign_found_present_is_a_red_flag():
    v = [{"id": "y", "disease": "y", "names": {}, "doshas": [],
          "features": [feat("F.a"), feat("F.grave", "arishta")]}]
    r = dx.rank(v, present("F.grave"))
    assert r[0]["red_flags"] == [{"feature": "F.grave", "verses": ["MN.1.1"]}]


def test_a_sign_listed_generally_and_again_as_the_variants_mark_counts_once():
    v = [{"id": "jvara.vata", "disease": "jvara", "names": {}, "doshas": ["vata"],
          "features": [feat("F.yawning", "purvarupa"), feat("F.yawning", "purvarupa", "cardinal")]}]
    r = dx.rank(v, present("F.yawning"))
    assert r[0]["score"] == 1.6                   # cardinal 2.0 × purvarupa 0.8, once
    assert len(r[0]["supporting"]) == 1


def test_with_one_disease_leading_its_shared_signs_are_still_worth_asking():
    # all three fever forms hold F.chills; asking it tells fever from "none of these"
    fevers = VARIANTS[:3]
    found = present("F.fatigue")
    r = dx.rank(fevers, found)
    q = {x["feature"]: x for x in dx.next_questions(r, fevers, found, n=10)}
    assert q["F.chills"]["value"] > 0 and q["F.chills"]["not_held_by"] == []
