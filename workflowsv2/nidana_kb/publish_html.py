#!/usr/bin/env python3
"""Write the translated Madhava Nidana as a browsable static site: an index
and one page per chapter, for the team to read at tuuyi.com/madhava_nidanam.

    python3 workflowsv2/nidana_kb/publish_html.py --kb /data/Sree/kb/madhava_nidana \\
        --translations /data/Sree/translations --out /data/Sree/site_mn/madhava_nidanam \\
        --report-email <address>

WHAT A CHAPTER PAGE SHOWS, per section of the team's edition: the heading in
English, IAST and Devanagari and the section's place in the edition; each
verse in Devanagari beside its IAST; the translation, key words and word
split; the Madhukośa and the Ātaṅkadarpaṇa as Sanskrit beside English, one
row per segment (per section for chapters 1-2); the translator's notes and
the reviewer's corrections; and a link that opens an email about the
section. Every verse, section and commentary segment is an anchor named by
its knowledge-base id (MN.2.4, MN.2.4-7, MN.2.4-7:mk3), so a link can point
at any of them.

THE SANSKRIT IS NIIMH's EDITION (CCRAS), marked All Rights Reserved; the
pages are for the team behind a login, and every page says so and carries
the draft notice. The pages are marked noindex.

INPUTS. The knowledge base text layer (<kb>/text/chNN.json, written by
ingest.py) for the verses, translations and commentary; the section files
(<translations>/chNN/sec_SS.json) for the English headings and the review
log, found through each passage's `edition` value "<file> §i"; titles.json
beside this file for the English chapter titles; and the chapter .docx
files in <translations>, copied for download.
"""
from __future__ import annotations

import argparse
import html
import json
import re
import shutil
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
SITE_CSS = "/site.css?v=20260925"
BASE = "/madhava_nidanam"
COMMENTARIES = (("madhukosha", "mk", "Madhukośa", "of Vijayarakṣita and Śrīkaṇṭhadatta"),
                ("atankadarpana", "at", "Ātaṅkadarpaṇa", "of Vācaspati"))

DRAFT = ("<div class=\"mn-draft\"><strong>Draft for the team.</strong> The English is a machine "
         "translation, checked by a second, independent AI pass. No Ayurveda physician has reviewed "
         "it yet; it is not for clinical use. Translator notes and review corrections are shown under "
         "each section. The Sanskrit text is from NIIMH e-Mādhavanidānam (CCRAS, all rights reserved) "
         "and is shown here for internal study only; do not copy or share it outside the team.</div>")

MN_CSS = """
.mn-draft { background: var(--amber-bg); color: var(--amber); border: 1px solid var(--panel-edge);
  border-radius: 8px; padding: 12px 16px; margin: 20px 0; font-size: 15px; }
.mn-sa { font-family: "Noto Serif Devanagari", "Noto Sans Devanagari", "Kohinoor Devanagari",
  "Devanagari MT", "Mangal", serif; font-size: 1.08em; line-height: 1.75; }
.mn-iast { font-style: italic; color: var(--muted); }
.mn-sa, .mn-iast, .mn-seg, .mn-verse { overflow-wrap: anywhere; min-width: 0; }
.mn-nav { display: flex; gap: 18px; flex-wrap: wrap; font-size: 15px; margin: 16px 0; }
.mn-tools { display: flex; gap: 10px; flex-wrap: wrap; margin: 12px 0 20px; }
.mn-tools button, .mn-jump button { font: inherit; font-size: 14px; padding: 6px 12px; border-radius: 6px;
  border: 1px solid var(--btn-edge); background: var(--panel); color: var(--text); cursor: pointer; }
.mn-jump input { font: inherit; font-size: 15px; padding: 6px 10px; border: 1px solid var(--btn-edge);
  border-radius: 6px; background: var(--panel); width: 12em; }
.mn-sec { background: var(--panel); border: 1px solid var(--rule); border-radius: 10px;
  padding: 18px 20px; margin: 22px 0; }
.mn-sec h2 { margin: 0 0 4px; font-size: 22px; }
.mn-sub { color: var(--label); font-size: 14px; margin: 0 0 12px; }
.mn-id { font-family: var(--mono); font-size: 12px; color: var(--label); text-decoration: none; }
.mn-verse { display: grid; grid-template-columns: 1fr 1fr; gap: 6px 18px; padding: 8px 0;
  border-top: 1px dashed var(--rule); }
.mn-tr { margin: 12px 0; }
.mn-label { font-family: var(--mono); font-size: 12px; letter-spacing: .08em; text-transform: uppercase;
  color: var(--label); margin: 14px 0 4px; }
details { margin: 10px 0; }
summary { cursor: pointer; font-weight: 600; }
.mn-seg { display: grid; grid-template-columns: 1fr 1fr; gap: 6px 18px; padding: 8px 0;
  border-top: 1px dashed var(--rule); }
.mn-seg:target, .mn-verse:target, .mn-sec:target { background: var(--accent-bg); }
.mn-words { margin: 4px 0; padding-left: 20px; }
.mn-notes li, .mn-rev li { margin: 4px 0; font-size: 15px; }
.mn-report { font-size: 13px; }
table.mn-toc { border-collapse: collapse; width: 100%; font-size: 15px; }
table.mn-toc td, table.mn-toc th { border-bottom: 1px solid var(--rule); padding: 6px 8px; text-align: left;
  vertical-align: top; }
body.mn-nosa .mn-sa, body.mn-nosa .mn-iast { display: none; }
body.mn-nosa .mn-verse, body.mn-nosa .mn-seg { grid-template-columns: 1fr; }
@media (max-width: 760px) { .mn-verse, .mn-seg { grid-template-columns: 1fr; } .wrap { padding: 0 16px; } }
"""

JS = """<script>
function mnToggleSa(){document.body.classList.toggle('mn-nosa');}
function mnExpand(open){document.querySelectorAll('details').forEach(function(d){d.open=open;});}
(function(){var h=decodeURIComponent(location.hash.slice(1));if(!h)return;
 var el=document.getElementById(h);if(!el)return;
 var d=el.closest('details');while(d){d.open=true;d=d.parentElement.closest('details');}
 el.scrollIntoView();})();
</script>"""

JUMP_JS = """<script>
function mnJump(e){e.preventDefault();var v=document.getElementById('mnq').value.trim();
 var m=v.match(/^(?:MN\\.)?(\\d+)(?:\\.(.*))?$/i);if(!m)return false;
 var ch=('0'+parseInt(m[1],10)).slice(-2);
 location.href='BASE/ch'+ch+(m[2]?'#MN.'+parseInt(m[1],10)+'.'+m[2]:'');return false;}
</script>""".replace("BASE", BASE)


def esc(s: Any) -> str:
    return html.escape(str(s or ""), quote=True)


def head(title: str, path: str) -> str:
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="robots" content="noindex,nofollow">
<title>{esc(title)} — Madhava Nidana (team draft)</title>
<link rel="canonical" href="https://tuuyi.com{BASE}{path}">
<link rel="icon" href="/favicon.svg" type="image/svg+xml">
<link rel="icon" href="/favicon.ico" sizes="32x32">
<link rel="stylesheet" href="{SITE_CSS}">
<link rel="stylesheet" href="{BASE}/mn.css">
</head>
<body>
<div class="wrap">
<header class="top">
  <a class="brand" href="/">Tuuyi</a>
  <nav><a href="{BASE}/">Madhava Nidana</a></nav>
</header>
<main>
{DRAFT}
"""


FOOT = """</main>
<footer>
  <div>Madhava Nidana with Madhukośa and Ātaṅkadarpaṇa · Sanskrit: NIIMH e-Mādhavanidānam (CCRAS) · English: team draft</div>
</footer>
</div>
""" + JS + "\n</body>\n</html>\n"


def section_file(translations: Path, chapter: int, edition: Optional[str]) -> Dict[str, Any]:
    """The section's translation file, found through the passage's
    `edition` value '<file> §i'."""
    m = re.search(r"§(\d+)", edition or "")
    if not m:
        return {}
    p = translations / f"ch{chapter:02d}" / f"sec_{int(m.group(1)):02d}.json"
    return json.loads(p.read_text(encoding="utf-8")) if p.is_file() else {}


def mailto(email: str, anchor: str, chapter: int) -> str:
    if not email:
        return ""
    from urllib.parse import quote
    subj = quote(f"Madhava Nidana {anchor}")
    body = quote(f"About {anchor} (https://tuuyi.com{BASE}/ch{chapter:02d}#{anchor}):\n\n")
    return f'<a class="mn-report" href="mailto:{esc(email)}?subject={subj}&amp;body={body}">Report on this section</a>'


def render_commentary(segs: List[Dict[str, Any]], short: str, name: str, by: str) -> str:
    if not segs:
        return ""
    rows = []
    for seg in segs:
        sid = seg.get("id", "")
        en = esc(seg.get("en", "")).replace("\n\n", "<br><br>")
        rows.append(f'<div class="mn-seg" id="{esc(sid)}"><div class="mn-sa">{esc(seg.get("sa", ""))}'
                    f' <a class="mn-id" href="#{esc(sid)}">{esc(sid)}</a></div><div>{en}</div></div>')
    return (f'<details class="mn-com"><summary>{esc(name)} <span class="mn-sub">{esc(by)} · '
            f'{len(segs)} segment{"s" if len(segs) != 1 else ""}</span></summary>{"".join(rows)}</details>')


def render_section(ps: Dict[str, Any], verses: Dict[str, Dict[str, Any]], sec: Dict[str, Any],
                   chapter: int, email: str) -> str:
    pid = ps["id"]
    head_en = sec.get("heading_en") or ""
    out = [f'<section class="mn-sec" id="{esc(pid)}">',
           f'<h2>{esc(head_en) or esc(ps.get("section"))}</h2>',
           f'<p class="mn-sub">{esc(ps.get("section"))} · <a class="mn-id" href="#{esc(pid)}">{esc(pid)}</a>'
           f' · edition: {esc(ps.get("edition") or "not recorded")}</p>']
    for vid in ps.get("verses") or []:
        v = verses.get(vid) or {}
        src = f' <span class="mn-sub">({esc(v.get("cites"))})</span>' if v.get("cites") else ""
        out.append(f'<div class="mn-verse" id="{esc(vid)}"><div class="mn-sa">{esc(v.get("sa"))} '
                   f'<a class="mn-id" href="#{esc(vid)}">{esc(vid)}</a>{src}</div>'
                   f'<div class="mn-iast">{esc(v.get("iast"))}</div></div>')
    if ps.get("translation"):
        out.append('<p class="mn-label">Translation</p><div class="mn-tr">'
                   + esc(ps["translation"]).replace("\n", "<br>") + "</div>")
    if ps.get("words"):
        out.append('<details><summary>Key words</summary><ul class="mn-words">' + "".join(
            f'<li><span class="mn-iast">{esc(w.get("term"))}</span> — {esc(w.get("en"))}</li>'
            for w in ps["words"]) + "</ul></details>")
    if ps.get("padaccheda"):
        out.append(f'<details><summary>Word split (padaccheda)</summary><p class="mn-iast">'
                   f'{esc(ps["padaccheda"])}</p></details>')
    com = ps.get("commentary") or {}
    for name, short, label, by in COMMENTARIES:
        segs = com.get(name)
        if isinstance(segs, list):
            out.append(render_commentary(segs, short, label, by))
    notes = sec.get("notes") or ps.get("notes") or []
    if notes:
        out.append(f'<details class="mn-notes"><summary>Translator notes ({len(notes)})</summary><ul>'
                   + "".join(f"<li>{esc(n)}</li>" for n in notes) + "</ul></details>")
    changes = (sec.get("review") or {}).get("changes") or []
    if changes:
        out.append(f'<details class="mn-rev"><summary>Review corrections ({len(changes)})</summary><ul>'
                   + "".join(f'<li><strong>{esc(c.get("where"))}</strong> ({esc(c.get("type"))}): '
                             f'“{esc(c.get("before"))}” → “{esc(c.get("after"))}”. {esc(c.get("why"))}</li>'
                             for c in changes) + "</ul></details>")
    out.append(mailto(email, pid, chapter))
    out.append("</section>")
    return "\n".join(out)


def render_chapter(doc: Dict[str, Any], translations: Path, title_en: str, n_chapters: int,
                   email: str, docx_name: Optional[str]) -> str:
    ch = doc["chapter"]
    verses = {v["id"]: v for v in doc.get("verses", [])}
    nav = [f'<a href="{BASE}/">All chapters</a>']
    if ch > 1:
        nav.append(f'<a href="{BASE}/ch{ch - 1:02d}">← Chapter {ch - 1}</a>')
    if ch < n_chapters:
        nav.append(f'<a href="{BASE}/ch{ch + 1:02d}">Chapter {ch + 1} →</a>')
    if docx_name:
        nav.append(f'<a href="{BASE}/docx/{esc(docx_name)}">Download .docx</a>')
    sections = []
    toc = []
    for ps in doc.get("passages", []):
        sec = section_file(translations, ch, ps.get("edition"))
        sections.append(render_section(ps, verses, sec, ch, email))
        toc.append(f'<li><a href="#{esc(ps["id"])}">{esc(sec.get("heading_en") or ps.get("section"))}</a>'
                   f' <span class="mn-id">{esc(ps["id"])}</span></li>')
    body = (head(f"Chapter {ch}: {title_en}", f"/ch{ch:02d}")
            + f'<div class="mn-nav">{" · ".join(nav)}</div>'
            + f'<h1>Chapter {ch}: {esc(title_en)}</h1>'
            + f'<p class="mn-sa">{esc(doc.get("title"))}</p>'
            + '<div class="mn-tools"><button type="button" onclick="mnToggleSa()">Show / hide Sanskrit</button>'
            + '<button type="button" onclick="mnExpand(true)">Expand all</button>'
            + '<button type="button" onclick="mnExpand(false)">Collapse all</button></div>'
            + f'<details><summary>Sections ({len(sections)})</summary><ol>{"".join(toc)}</ol></details>'
            + "\n".join(sections)
            + f'<div class="mn-nav">{" · ".join(nav)}</div>'
            + FOOT)
    return body


def render_index(rows: List[Dict[str, Any]]) -> str:
    trs = "".join(
        f'<tr><td>{r["ch"]}</td><td><a href="{BASE}/ch{r["ch"]:02d}">{esc(r["title_en"])}</a></td>'
        f'<td class="mn-sa">{esc(r["title_sa"])}</td><td>{r["verses"]}</td><td>{r["sections"]}</td>'
        f'<td>{"<a href=" + chr(34) + BASE + "/docx/" + esc(r["docx"]) + chr(34) + ">.docx</a>" if r["docx"] else ""}</td></tr>'
        for r in rows)
    total_v = sum(r["verses"] for r in rows)
    total_s = sum(r["sections"] for r in rows)
    return (head("Contents", "/")
            + "<h1>Mādhava Nidāna</h1>"
            + '<p class="lede">Mādhavakara\'s Rugviniścaya with the Madhukośa of Vijayarakṣita and '
              "Śrīkaṇṭhadatta and the Ātaṅkadarpaṇa of Vācaspati, in Sanskrit and English.</p>"
            + f"<p>{len(rows)} chapters, {total_v} verses, {total_s} sections. Each chapter page gives, "
              "section by section, the verses in Devanagari and IAST with their translation, and both "
              "commentaries as Sanskrit beside English. Every verse, section and commentary segment has "
              "its own link: its id, such as MN.2.4 (a verse), MN.2.4-7 (a section) or MN.2.4-7:mk3 "
              "(a Madhukośa segment; :at for the Ātaṅkadarpaṇa).</p>"
            + JUMP_JS
            + '<form class="mn-jump" onsubmit="return mnJump(event)"><label>Go to an id: '
              '<input id="mnq" placeholder="MN.22.14 or 22"></label> <button type="submit">Go</button></form>'
            + '<table class="mn-toc"><thead><tr><th>#</th><th>Chapter</th><th>Sanskrit</th><th>Verses</th>'
              f"<th>Sections</th><th></th></tr></thead><tbody>{trs}</tbody></table>"
            + FOOT)


def publish(kb_dir: Path, translations: Path, out: Path, email: str = "") -> Dict[str, Any]:
    titles = json.loads((HERE / "titles.json").read_text(encoding="utf-8"))
    docs = sorted((json.loads(p.read_text(encoding="utf-8")) for p in (kb_dir / "text").glob("ch*.json")),
                  key=lambda d: d["chapter"])
    out.mkdir(parents=True, exist_ok=True)
    (out / "docx").mkdir(exist_ok=True)
    (out / "mn.css").write_text(MN_CSS, encoding="utf-8")
    rows, counts = [], {"verses": 0, "sections": 0, "segments": 0, "pages": 0}
    n = max(d["chapter"] for d in docs)
    for doc in docs:
        ch = doc["chapter"]
        docx = sorted(translations.glob(f"{ch:02d}_*_en.docx"))
        docx_name = docx[0].name if docx else None
        if docx:
            shutil.copy2(docx[0], out / "docx" / docx_name)
        title_en = titles.get(f"{ch:02d}", f"Chapter {ch}")
        (out / f"ch{ch:02d}.html").write_text(
            render_chapter(doc, translations, title_en, n, email, docx_name), encoding="utf-8")
        nv, ns = len(doc.get("verses", [])), len(doc.get("passages", []))
        counts["verses"] += nv
        counts["sections"] += ns
        counts["segments"] += sum(len(v) for ps in doc.get("passages", [])
                                  for v in (ps.get("commentary") or {}).values() if isinstance(v, list))
        counts["pages"] += 1
        rows.append({"ch": ch, "title_en": title_en, "title_sa": doc.get("title", ""), "verses": nv,
                     "sections": ns, "docx": docx_name})
    (out / "index.html").write_text(render_index(rows), encoding="utf-8")
    return counts


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kb", type=Path, required=True)
    ap.add_argument("--translations", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--report-email", default="", help="address the 'Report' links write to")
    args = ap.parse_args()
    print(json.dumps(publish(args.kb, args.translations, args.out, args.report_email)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
