# Selection record: `cw-site`, claim source "the tuuyi.com website"

Status: **CONFIRMED 2026-09-17 by Bruce**, in the terminal session (there is
no page for this yet). Prepared by Claude following
`docs/claim-source-selection.md`. The source files in this directory were
written after confirmation by `extract_sources.py` in the engagement
directory.

## 1. The source and the copy read

Instruction (Bruce, 2026-09-16): run the claims review on this repository
"with the tuuyi.com site text as the claim source".

Copy read: `site/` in the target clone at commit 3b57e91d. Compared with the
live site on 2026-09-17 by `diff` of `curl` output against the file: index,
how-it-works, pricing, about, introduction, demo, privacy, the not-found page
and the caption file are identical. Contact differs in one respect only:
Cloudflare replaces the address `info@tuuyi.com` with an obfuscated link and
adds its decoding script; the visible text is the same. The images were not
compared. No network snapshot is kept, because the pinned commit is the
record.

## 2. Candidates

Listings used: the files in `site/`; `site/sitemap.xml`; every `href` and
`src` in the nine HTML files. The directory and the sitemap agree on eight
pages. The directory also holds `404.html`, which the sitemap does not list.
The links add one other host, one video, one caption file and one third-party
PDF. No page text is written by script (`site.js` sets only the theme
button's label). The pages contain no `<img>` elements and no alt text.

| # | Candidate | Found by | Proposed | Reason when left out | Source file |
|---|---|---|---|---|---|
| 1 | `/how-it-works` | dir, sitemap, links | in | | `how-it-works.md` |
| 2 | `/` (index) | dir, sitemap, links | in | | `index.md` |
| 3 | `/introduction`, page text and chapter list | dir, sitemap, links | in | | `introduction.md` |
| 4 | `/introduction`, the transcript printed on the page | same page | in, as its own file | | `introduction-transcript.md` |
| 5 | `/pricing` | dir, sitemap, links | in | | `pricing.md` |
| 6 | `/demo` | dir, sitemap, links | in | | `demo.md` |
| 7 | `/about` | dir, sitemap, links | in | | `about.md` |
| 8 | `/privacy` | dir, sitemap, links | in | | `privacy.md` |
| 9 | `/contact` | dir, sitemap, links | in (was dropped in the first pass on a reading of its headings) | | `contact.md` |
| 10 | `404.html` | dir only | out (2026-09-17, Bruce) | An orphan: no link on the site and no sitemap entry leads to it; a visitor reaches it only by a wrong address. It was enumerated once (one claim, "The site has six" pages) before the orphan rule was set. | |
| 11 | `media/introduction.vtt`, the video's captions | `<track>` on the page | out | Its words are the printed transcript's words: 1,347 against 1,347, identical in order after removing speaker labels and punctuation. | |
| 12 | `media/tuuyi-claims-review.mp4`, what is shown on screen | `<video>` on the page | out | The slides carry text that the narration does not (the poster frame reads "Tuuyi identifies and checks software product claims against supplied code and materials, then delivers a report the client can interrogate."). There is no text file of it to quote from, and the file is not in the repository. Reviewing it needs a transcription of the slides. | |
| 13 | `media/introduction-poster.jpg` | `poster=` on the page | out | One frame of candidate 12. | |
| 14 | `og.png`, the link-preview image | `og:image` meta | out | Its words ("Technical due diligence for software acquisitions. The seller's claims, checked against the code. Every finding cited.") all appear in the index title and meta description, which are in. | |
| 15 | `https://demo.tuuyi.com/` | links on index and demo | out | A different host serving a delivered report about another target and a chat page. The statements the practice makes about the demo are on `/demo` and `/privacy`, which are in. | |
| 16 | `mason.gmu.edu/~rhanson/SciCast2015.pdf` | link on about | out | Not the seller's text. | |
| 17 | `robots.txt`, `sitemap.xml`, `site.css`, `site.js`, favicons | dir | out | No text a buyer reads. | |
| 18 | The engagement site under `/p/` and the letter template | not linked from the public site | out | Not part of "the website" a buyer can open; served by `src/client_ui/`, which is evidence. | |

## 3. Regions

Kept from every page: the `<main>` element, all of it.

Proposed to add, as the first two lines of each source file: the page's
`<title>` and its meta description. Search engines and link previews show
them to buyers and several make statements the page body words differently,
for example privacy: "This site sets no cookies and runs no analytics."; how
it works: "claims frozen before testing, every finding cited to file and
line, an independent check, ratings against your thresholds, a person
signs."; index: "From a free demo to a signed opinion."

Dropped, with what is lost:

- `<header>` and `<nav>`: link labels, and one statement repeated on every
  page: "Beta: free claims reviews, limited". Proposed: add that sentence
  once, to `index.md`, under a line saying it is the banner shown on every
  page.
- `<footer>`: "© 2026 Tuuyi", and link labels. Nothing else.
- `<form>` on contact: field labels and the option list of reasons for
  writing. No statements.
- `<video>` fallback text on introduction: "Your browser cannot play this
  video. Download it instead, or read the transcript below."
- Open Graph and Twitter meta tags: they repeat the title and description.
- `aria-label="One finding, from the demo"` on index: repeats the visible
  panel heading.

## 4. Judgements the table does not show

- The introduction page is split in two because the transcript is speech by
  two named speakers and the rest is page text; a quote from one reads
  differently from a quote from the other.
- The chapter list on the introduction page repeats the transcript's
  headings. It stays in `introduction.md`; enumeration will see the same
  twelve headings twice across the two files.
- Ten enumeration runs instead of eight. Order: how-it-works first, alone,
  as agreed.

## 5. Confirmation

Confirmed by: Bruce D'Ambrosio, 2026-09-17, each proposal ticked separately:
contact and 404 in; title and meta description at the top of each file; the
banner once, in `index.md`; `demo.tuuyi.com` out; on-screen video text out;
caption file and `og.png` out.
Changes made at confirmation: none.
Changed later the same day: `404.md` removed as an orphan (Bruce, after asking whether orphan pages belong; the privacy page was checked and is linked from every page's footer).

## 6. Extraction check, after confirmation

Word counts of each file's body against the visible text of `<main>`: equal
for six pages; index 3 short, contact 2, privacy 1. All six words were
looked at. Each is a punctuation mark counted apart from its word in one
count and not the other; no word is missing. The check did find a defect:
two labels side by side on the home page came out joined ("partly
trueRating"). The extractor now separates adjacent inline elements, and the
files were written again. `how-it-works.md` was read through before its
enumeration; the others are read before theirs.

Two included-by-default pages carry statements: the not-found page says "The
site has six" pages (the sitemap lists eight), and the contact page says
"What you send here goes by email to the practice and nowhere else".
