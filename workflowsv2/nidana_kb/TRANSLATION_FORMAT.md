# Madhava Nidana translation: document format

This is the format for each translated chapter. A program reads these
documents into the knowledge base that the consultation app uses, so the
layout below is a requirement, not a style preference. A chapter that follows
it is read without anyone correcting it by hand.

## One document per chapter

- Name the file with the chapter number and its Sanskrit name, e.g.
  `02_jvaranidanam.docx`. Word or Google Docs is fine; download a Google Doc
  as .docx.
- The first line is a **Heading 1** that contains the words `Chapter N`, e.g.
  `Chapter 2 — Jvaranidānam (Fever)`.
- Each part of the chapter starts with a **Heading 2**, e.g. `Nidana (causes)`,
  `Purvarupa (prodrome)`, `Rupa — Vataja`, `Sadhya-asadhyata (prognosis)`.

## One block per verse, or per run of verses translated together

Each paragraph in a block starts with a label and a colon. Use the labels
exactly as written here. Put each on its own paragraph.

| Label | What follows it | Required |
|---|---|---|
| `Verse:` | The verse in Devanagari, ending with its number between double dandas: `॥ ४ ॥`. One paragraph per verse. | Yes |
| `Source:` | Where Madhava took it from, e.g. `Ca. Ni. 1` or `Su. U. 40` | When known |
| `IAST:` | The transliteration in IAST, ending with the same number: `\|\| 4 \|\|` | Yes |
| `Padaccheda:` | The word split | Recommended |
| `Words:` | The word meanings, one per line below the label, as `term — meaning` (with a dash between) | Yes |
| `Translation:` | The translation, in clinical English | Yes |
| `Madhukosha:` | The Madhukośa commentary on these verses: the Sanskrit on this paragraph, the English on the paragraphs after it | When available |
| `Atankadarpana:` | The Ātaṅkadarpaṇa commentary, laid out the same way | When available |
| `Note:` | Anything else: a variant reading, a doubt, a modern correlate | Optional |

When two or more verses are translated together, give each its own `Verse:`
and `IAST:` paragraph, then one `Words:`, one `Translation:` and so on for the
group.

### Example

> **Heading 2:** Purvarupa — general
>
> Verse: श्रमोऽरतिर्विवर्णत्वं वैरस्यं नयनप्लवः । इच्छाद्वेषौ मुहुश्चापि शीतवातातपादिषु ॥ ४ ॥
> Verse: जृम्भाऽङ्गमर्दो गुरुता रोमहर्षोऽरुचिस्तमः । अप्रहर्षश्च शीतं च भवत्युत्पत्स्यति ज्वरे ॥ ५ ॥
> Source: Ca. Ni. 1
> IAST: śramo'ratir vivarṇatvaṃ vairasyaṃ nayanaplavaḥ | icchādveṣau muhuś cāpi śītavātātapādiṣu || 4 ||
> IAST: jṛmbhā'ṅgamardo gurutā romaharṣo'rucis tamaḥ | apraharṣaś ca śītaṃ ca bhavaty utpatsyati jvare || 5 ||
> Words:
> śrama — fatigue
> arati — restlessness, malaise
> nayanaplava — watery eyes
> Translation: When fever is about to arise, these appear: fatigue, restlessness, …
> Note: Some editions read …

## Rules the program checks

1. **Every verse has its number**, in Devanagari (`॥ ४ ॥`) and in the IAST
   (`|| 4 ||`), and the two numbers agree.
2. **Every verse has an IAST line.**
3. **No translation without its verse.** A list of symptoms or a paragraph of
   English with no `Verse:` above it cannot be cited, and the app will not use
   it. In the sampler, Arshas, Raktapitta and several other diseases have
   English lists only; those need their verses.
4. **Numbers go at the end of the verse.** Do not put `॥ n ॥` in the middle of
   a verse. A verse split across two sections is numbered with a letter:
   the first half `॥ १४a ॥` / `|| 14a ||`, the second `॥ १४b ॥` / `|| 14b ||`.
5. **Use the standard text.** Where the edition you are translating from
   differs from another reading, give the one you follow in `Verse:` and the
   other in a `Note:`.

## What happens to the document

The program reports every place a rule is not met, verse by verse. Nothing is
corrected silently. An Ayurveda reviewer then checks each verse and its
translation before the app uses it, and the reviewer's name and date are
recorded against each verse.
