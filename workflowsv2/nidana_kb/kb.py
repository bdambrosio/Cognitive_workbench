"""The Madhava Nidana knowledge base on disk: where each file is, how it is
written, and the read-only lookups the consultation uses.

THREE LAYERS, each a directory under the knowledge base root:

  text/chNN.json      the translation as the team delivered it (ingest.py)
  clinical/chNN.json  diseases, their variants and their features, each
                      feature citing verse ids (extract.py)
  lexicon.json        the features themselves, shared across chapters

The embeddings used to match a finding to a lexicon feature are computed in
memory when first needed (a few seconds for a thousand features) rather
than stored.

A verse id is `MN.<chapter>.<verse>`; a passage id is the id of its first
verse, with `-<last>` when it holds several (`MN.2.4-5`). Every citation a
workflow writes is resolved here, by a file lookup, before it is shown.

The root is $AYUR_KB, else data/ayurveda/kb/madhava_nidana under the repo.
"""
from __future__ import annotations

import datetime
import json
import os
import re
import unicodedata
from functools import lru_cache
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

REPO = Path(__file__).resolve().parents[2]
DEFAULT_KB = REPO / "data" / "ayurveda" / "kb" / "madhava_nidana"
#: MN.<chapter>.<verse>; the verse may be a half (14a, 14b) or a repeat of a
#: number the edition uses twice (1r2).
VERSE_ID_RE = re.compile(r"\bMN\.(\d+)\.(\d+[ab]?(?:r\d+)?)\b")


def kb_root(explicit: Optional[Path] = None) -> Path:
    if explicit:
        return Path(explicit)
    env = os.environ.get("AYUR_KB")
    return Path(env) if env else DEFAULT_KB


def text_path(kb_dir: Path, chapter: int) -> Path:
    return Path(kb_dir) / "text" / f"ch{int(chapter):02d}.json"


def clinical_path(kb_dir: Path, chapter: int) -> Path:
    return Path(kb_dir) / "clinical" / f"ch{int(chapter):02d}.json"


def lexicon_path(kb_dir: Path) -> Path:
    return Path(kb_dir) / "lexicon.json"


def write_json(path: Path, obj: Any) -> None:
    from utils.file_utils import atomic_write_text
    path.parent.mkdir(parents=True, exist_ok=True)
    atomic_write_text(path, json.dumps(obj, ensure_ascii=False, indent=1) + "\n")


def read_json(path: Path, default: Any = None) -> Any:
    p = Path(path)
    if not p.is_file():
        return default
    return json.loads(p.read_text(encoding="utf-8"))


def update_manifest(kb_dir: Path, source: Dict[str, Any], chapters: Sequence[int]) -> None:
    """Which document each chapter's text came from. The manifest's
    `version` is the time of its last change; a consultation records it."""
    path = Path(kb_dir) / "manifest.json"
    man = read_json(path, {"chapters": {}})
    for n in chapters:
        man["chapters"][str(n)] = {"doc": source["doc"], "sha256": source["sha256"],
                                   "ingested_at": source["ingested_at"]}
    man["version"] = datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")
    write_json(path, man)


def normalise(text: str) -> str:
    """For checking a quoted phrase against a verse: lower case, marks and
    punctuation removed, spaces removed (sandhi joins words, so word breaks
    in a quote need not match the verse's)."""
    t = unicodedata.normalize("NFD", text or "").lower()
    return "".join(c for c in t if c.isalnum() and unicodedata.category(c) != "Mn")


class KB:
    """The knowledge base loaded into memory. Construct once per session;
    the files are small (the whole text is well under 10 MB)."""

    def __init__(self, root: Optional[Path] = None) -> None:
        self.root = kb_root(root)
        self.manifest = read_json(self.root / "manifest.json", {"chapters": {}})
        self.verses: Dict[str, Dict[str, Any]] = {}
        self.passages: Dict[str, Dict[str, Any]] = {}
        self.passage_of: Dict[str, str] = {}
        #: Commentary segments by id (MN.2.9-10:mk3): the Sanskrit of one
        #: sentence or so of a commentary and its English.
        self.segments: Dict[str, Dict[str, Any]] = {}
        self.chapters: Dict[int, Dict[str, Any]] = {}
        for p in sorted((self.root / "text").glob("ch*.json")):
            doc = read_json(p)
            self.chapters[doc["chapter"]] = {"title": doc.get("title"), "source": doc.get("source")}
            for v in doc.get("verses", []):
                self.verses[v["id"]] = dict(v, chapter=doc["chapter"])
            for ps in doc.get("passages", []):
                self.passages[ps["id"]] = dict(ps, chapter=doc["chapter"])
                for vid in ps.get("verses", []):
                    self.passage_of[vid] = ps["id"]
                for name, segs in (ps.get("commentary") or {}).items():
                    for seg in segs if isinstance(segs, list) else []:
                        self.segments[seg["id"]] = dict(seg, commentary=name, passage=ps["id"])
        self.diseases: Dict[str, Dict[str, Any]] = {}
        for p in sorted((self.root / "clinical").glob("ch*.json")):
            for d in (read_json(p) or {}).get("diseases", []):
                self.diseases[d["id"]] = d
        self.lexicon: Dict[str, Dict[str, Any]] = read_json(lexicon_path(self.root), {}) or {}

    @property
    def version(self) -> Optional[str]:
        return self.manifest.get("version")

    # ---- lookups ------------------------------------------------------------

    def verse(self, vid: str) -> Optional[Dict[str, Any]]:
        return self.verses.get(vid)

    def passage_for(self, vid: str) -> Optional[Dict[str, Any]]:
        pid = self.passage_of.get(vid)
        return self.passages.get(pid) if pid else None

    def segment(self, sid: str) -> Optional[Dict[str, Any]]:
        return self.segments.get(sid)

    def commentary(self, cid: str) -> Optional[Dict[str, Any]]:
        """A whole commentary on a passage, by `<passage id>:mk` (Madhukośa)
        or `<passage id>:at` (Ātaṅkadarpaṇa): its segments' Sanskrit and
        English, in order, with the segment ids."""
        pid, _, short = cid.rpartition(":")
        name = {"mk": "madhukosha", "at": "atankadarpana"}.get(short)
        ps = self.passages.get(pid)
        if not (ps and name):
            return None
        segs = (ps.get("commentary") or {}).get(name)
        if not isinstance(segs, list) or not segs:
            return None
        return {"id": cid, "commentary": name, "passage": pid, "edition": ps.get("edition"),
                "segments": [{"id": g["id"], "sa": g.get("sa", ""), "en": g.get("en", "")} for g in segs]}

    def feature(self, fid: str) -> Optional[Dict[str, Any]]:
        return self.lexicon.get(fid)

    def disease(self, did: str) -> Optional[Dict[str, Any]]:
        return self.diseases.get(did)

    def variants(self) -> List[Dict[str, Any]]:
        """Every disease variant with the features that apply to it: the
        disease's general features plus the variant's own. A disease with no
        variants is one variant with its own id.

        A variant with no features of its own, when its siblings have some,
        is left out: the two- and three-doṣa forms are defined by a rule
        (two or all of the siblings' signs together; EXTRACT.md §4), and
        with only the shared features they would tie with every sibling."""
        out = []
        for d in self.diseases.values():
            general = [f for f in d.get("features", []) if not f.get("variant")]
            vs = d.get("variants") or [{"id": d["id"], "doshas": []}]
            with_own = {f.get("variant") for f in d.get("features", []) if f.get("variant")}
            for v in vs:
                if with_own and v["id"] not in with_own:
                    continue
                own = [f for f in d.get("features", []) if f.get("variant") == v["id"]]
                out.append({"id": v["id"], "disease": d["id"], "names": d.get("names", {}),
                            "label": v.get("label") or (d.get("names") or {}).get("en") or v["id"],
                            "doshas": v.get("doshas", []), "features": general + own})
        return out

    def resolve_citation(self, vid: str, quote: Optional[str] = None) -> Dict[str, Any]:
        """Whether a verse id exists and, when a quote is given, whether the
        quote is in that verse's IAST or Devanagari. Returns {"ok", "why"}."""
        v = self.verses.get(vid)
        if v is None:
            return {"ok": False, "why": f"{vid} is not in the knowledge base"}
        if quote:
            q = normalise(quote)
            if q and q not in normalise(v.get("iast", "")) and q not in normalise(v.get("sa", "")):
                return {"ok": False, "why": f"'{quote}' is not in the text of {vid}"}
        return {"ok": True, "why": ""}

    def cited_ids(self, text: str) -> List[str]:
        return [f"MN.{a}.{b}" for a, b in VERSE_ID_RE.findall(text or "")]

    def render_verse(self, vid: str) -> str:
        """A verse as the consultation shows it: id, Devanagari, IAST, and
        the translation of the passage it belongs to."""
        v = self.verses.get(vid)
        if v is None:
            return f"{vid}: not in the knowledge base"
        ps = self.passage_for(vid) or {}
        coms = [f"{ps.get('id')}:{short}" for name, short in
                (("madhukosha", "mk"), ("atankadarpana", "at"))
                if (ps.get("commentary") or {}).get(name)]
        return (f"{vid} ({v.get('section') or 'chapter ' + str(v['chapter'])})\n{v['sa']}\n"
                f"{v.get('iast') or '(no transliteration)'}\n"
                f"Section {ps.get('id', '?')}; edition: {ps.get('edition') or 'not recorded'}"
                + (f"; commentaries: {', '.join(coms)}" if coms else "") + "\n"
                f"Translation ({ps.get('id', '?')}): {ps.get('translation') or '(none)'}")

    # ---- mapping a finding to a feature -------------------------------------

    def candidates(self, text: str, k: int = 8) -> List[Dict[str, Any]]:
        """The lexicon features nearest in meaning to a finding's words, by
        embedding similarity. The caller decides which, if any, it is."""
        ids, mat = _lexicon_matrix(str(self.root), _lexicon_key(self.lexicon))
        if not ids:
            return []
        q = embed([text])[0]
        sims = mat @ q
        order = sims.argsort()[::-1][:k]
        return [{"id": ids[i], "score": float(sims[i]), **self.lexicon[ids[i]]} for i in order]


EMBEDDER = "BAAI/bge-small-en-v1.5"


def feature_text(f: Dict[str, Any]) -> str:
    """What a lexicon entry is embedded as: its English and clinical names
    and its Sanskrit terms."""
    return "; ".join(x for x in [f.get("en", ""), f.get("clinical", ""),
                                 ", ".join(f.get("sa") or [])] if x)


@lru_cache(maxsize=1)
def _embedder():
    """Loaded once per process. claims_audit/duplicates.embed loads the same
    model per call, which suits its one batch per run; a consultation embeds
    each new finding, so the model is kept."""
    from sentence_transformers import SentenceTransformer
    try:
        return SentenceTransformer(EMBEDDER, local_files_only=True, device="cpu")
    except Exception:                                          # noqa: BLE001
        return SentenceTransformer(EMBEDDER, device="cpu")


def embed(texts: Sequence[str]):
    return _embedder().encode(list(texts), normalize_embeddings=True, show_progress_bar=False)


def _lexicon_key(lexicon: Dict[str, Any]) -> str:
    return json.dumps({k: feature_text(v) for k, v in sorted(lexicon.items())}, ensure_ascii=False)


@lru_cache(maxsize=4)
def _lexicon_matrix(root: str, key: str):
    """Embeddings of the lexicon, computed once per lexicon content."""
    items = json.loads(key)
    ids = list(items)
    if not ids:
        return [], None
    return ids, embed([items[i] for i in ids])
