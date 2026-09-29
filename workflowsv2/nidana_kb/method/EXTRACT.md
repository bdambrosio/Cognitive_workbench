# Extracting the clinical content of Madhava Nidana — method

## 1. Purpose

You are given one section of one chapter of Madhava Nidana, as verses with their transliteration and translation. Your job is to record, for each disease the section describes, the features it gives for that disease, and to cite the verse each feature comes from.

> Record what these verses say about each disease, one feature at a time, and cite the verse for each.

The record is used by a consultation program that suggests diagnostic questions to a supervised Ayurveda practitioner. A feature you record that the verses do not state becomes a question asked of a patient for no reason, and a diagnosis suggested on no basis. Record only what the verses state.

## 2. Terms

- A **disease** is a condition the chapter names, such as jvara (fever) or atisāra (diarrhoea).
- A **variant** is a form of a disease distinguished by the doṣa that causes it: vātaja, pittaja, kaphaja, a two-doṣa form (dvandvaja), or the three-doṣa form (sannipātaja). A disease the section does not divide has no variants.
- A **feature** is one thing a practitioner can find or ask about: a symptom, a sign, a cause, something that relieves or aggravates. "Yawning" is a feature; "the prodrome of fever" is not. A doṣa is never a feature: when a verse says a disease or variant is caused by a doṣa, that doṣa goes in the variant's `doshas`.
- The **role** of a feature is what the verse says it is for this disease, one of these nine. The first six are the five-part scheme of diagnosis in the text's first chapter, with relief and aggravation as two roles; the last three are the text's prognostic terms.

| role | meaning |
|---|---|
| `nidana` | a cause of the disease: a food, a behaviour, a season, an injury |
| `purvarupa` | a prodromal sign, appearing before the disease is manifest |
| `rupa` | a sign or symptom of the manifest disease |
| `upashaya` | something that relieves the condition |
| `anupashaya` | something that aggravates the condition |
| `samprapti` | a step in how the disease develops |
| `upadrava` | a complication |
| `arishta` | a sign that the verse says means death is near or the case is incurable |
| `sadhyata` | a statement about curability that is not a sign, such as "curable if recent" |

## 3. What to record

**One feature per entry.** A verse listing six signs gives six entries. A compound the translation splits into several signs (pricking pain in the heart, navel, anus, abdomen and flanks) gives one entry per site.

**The disease and variant it belongs to.** A feature the verse gives for the disease as a whole has `variant` empty. A feature the verse gives for one doṣa form has that variant's id. The id of a variant is the disease id, a full stop, and the doṣa: `jvara.vata`, `jvara.pitta`, `jvara.kapha`, `jvara.dvandva`, `jvara.sannipata`.

**The weight.** `cardinal` when the verse presents the feature as the distinguishing mark of the disease or variant (verses such as "from Vāta, excessive yawning" name one distinguishing sign per doṣa). `supporting` for a feature in a general list.

**The citation.** `verses` holds the id of each verse the feature is stated in, from the ids given to you. `quote` is the words of that verse, copied from its transliteration as given, that state the feature: a single word or a short phrase. The program checks every quote against the verse and discards any that it does not find there.

**The feature id.** You are given a list of features already recorded from other sections. If one of them is the same feature, use its id. Otherwise make a new id: `F.` followed by a short English name in lower case with underscores (`F.yawning`, `F.burning_eyes`), and describe it once in `new_features`. A sign the verse qualifies is a different feature from the plain sign: "excessive yawning" (jṛmbhā atyartham) is `F.excessive_yawning`, not `F.yawning`, even when both appear in the same chapter.

**A new feature's description.**
- `en`: the plain English name.
- `clinical`: the clinical term.
- `sa`: the Sanskrit term or terms, in IAST.
- `lay_question`: one question a practitioner could ask a patient to find out whether the feature is present, in everyday words.
- `observable`: how it can be found, any of `reported` (the patient says so), `examined` (the practitioner sees or feels it), `wearable` (a wearable device measures it, such as temperature, heart rate or sleep), `lab` (a laboratory test shows it).

## 4. What not to record

- Nothing the verses do not state. The translation may add explanation; record a feature only when the verse it translates states it.
- Nothing from your own knowledge of Ayurveda or of medicine, even when you are sure it is true of the disease.
- No modern diagnosis as a feature. A modern correlate the translator gives in a note goes in `notes`, with the verse id.
- No rule as a feature. A verse that says a variant is recognised by a combination of other variants' signs (any two together for the two-doṣa form, all together for the three-doṣa form) states a rule, not a sign a patient can have. Record the variant, and put the rule in `notes` with the verse id.
- A passage marked as having no verse cannot be cited. Record nothing from it, and say in `notes` what it contains.

## 5. The output

One JSON object. Its shape is enforced; this section says what makes a field correct.

| Field | Contents |
|---|---|
| `diseases[].id` | The disease's id: its Sanskrit name in IAST without diacritics, lower case, words joined by underscores (`jvara`, `grahani`, `vata_vyadhi`). Use the id you were given if the disease was recorded from an earlier section |
| `diseases[].names` | `sa` (Devanagari), `iast`, `en` (the English name the translation uses) |
| `diseases[].variants[]` | `id` (per §3), `doshas` (the doṣas that cause it: `vata`, `pitta`, `kapha`; empty for a variant defined otherwise) and `label` (the English name) |
| `diseases[].features[]` | `feature`, `role`, `variant`, `weight`, `verses`, `quote`, per §3 |
| `new_features[]` | `id`, `en`, `clinical`, `sa`, `lay_question`, `observable`, per §3 |
| `notes[]` | Anything a reviewer should know: a verse you could not read, a doubtful translation, a modern correlate the translator gave. One per entry, starting with the verse or passage id |

Emit nothing outside the JSON object.
