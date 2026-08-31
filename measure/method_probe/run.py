#!/usr/bin/env python3
"""Ask several models what METHOD tells them to do, and record the answers.

    python3 measure/method_probe/run.py --models or_glm53flash local_qwen38flashnext grok_4p6
    python3 measure/method_probe/run.py --models or_glm53flash --only findings-count

WHAT THIS MEASURES, and what it does not. A run measures interpretation and
performance together and costs fifteen minutes. This isolates interpretation:
no target, no materials, no report. It answers "would this reader do the right
thing", never "did it".

DISAGREEMENT IS THE INSTRUMENT. `questions.yaml` marks each question decided,
judgement or undecided. A decided question has an answer we settled and put in
METHOD, so a reader that diverges is evidence the text failed to carry it — not
that the reader is weak. A judgement question has no right answer and the
signal is the spread. An undecided one measures what a gap costs.

EACH QUESTION IS ITS OWN CALL. Eighteen questions in one call lets a model
build a consistent story across them, which is the opposite of what is being
measured: an auditor meets these decisions one at a time, hundreds of turns
apart, with no memory of having answered a sibling.

AT THE MODEL'S CONFIGURED EFFORT, not a uniform one. GLM and Qwen run audits at
medium and grok's file says low. Probing all three at one level would measure a
reader nobody deploys. The effort used is recorded on every row.

THE DELIVERED TEXT, NOT THE FILE. `load_workflow` strips every section marked
for the practice — §§10, 12a, 13-14 and 17-19 never reach an auditor — so
probing the raw file would measure a document nobody reads.

NO GRADING HERE. Answers are recorded verbatim for a person to read against
each question's `expect`. A scorer would be a second model's opinion of a first
model's reading of a document, and the thing under test is the document.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
for p in (str(REPO), str(REPO / "src"), str(REPO / "measure")):
    if p not in sys.path:
        sys.path.insert(0, p)

from chat.workflow import load_workflow                        # noqa: E402
from prescreen import probe                                    # noqa: E402

QUESTIONS = HERE / "questions.yaml"

SYSTEM = (
    "You are the auditor on a technical claims-audit engagement. The method "
    "you work to is given below in full. Answer the question that follows "
    "from that method.\n\n"
    "Answer as the auditor would decide, not as a summary of what the "
    "document says. Be brief: the decision, then one or two sentences of "
    "reasoning. Cite the section you are relying on.\n\n"
    "=== METHOD ===\n{method}\n=== END METHOD ===")


def model_route(name: str) -> dict:
    """A measure/models/*.yaml mapped onto `prescreen.probe` arguments.

    The route is read from the same file a real run uses, so the reader probed
    is the reader deployed — same model id, same endpoint, same pinned
    provider, same reasoning effort.
    """
    doc = yaml.safe_load((REPO / "measure" / "models" / f"{name}.yaml")
                         .read_text(encoding="utf-8")) or {}
    llm = doc.get("llm_config") or {}
    # THE BACKEND'S OWN NORMALISATION, not a guess. `_ChatBackend.__init__`
    # strips a trailing `/v1` and then posts to `{base}/v1/chat/completions`,
    # so a config ending in `/v1` and one that does not both resolve to the
    # same endpoint. Appending blind sends the local route to
    # `:5000/chat/completions`, which 404s.
    base = (llm.get("vllm_url") or "http://127.0.0.1:5000").rstrip("/")
    if base.endswith("/v1"):
        base = base[:-3]
    order = ((llm.get("extra_body") or {}).get("provider") or {}).get("order") or []
    return {
        "config": name,
        "model": llm.get("model"),
        "url": f"{base}/v1/chat/completions",
        # None is a local server that wants no bearer token.
        "key_env": llm.get("api_key"),
        "effort": llm.get("reasoning_effort"),
        # Pins the gateway provider where there is one; elsewhere it is just a
        # label on the row.
        "tag": order[0] if order else name,
    }


def ask(route: dict, method: str, q: dict, timeout: int) -> dict:
    """One question, one call. `probe` never raises; nor does this."""
    row = probe(route["model"], route["tag"], effort=route["effort"],
                url=route["url"], key_env=route["key_env"],
                timeout=timeout,
                system=SYSTEM.format(method=method),
                user=q["ask"].strip(),
                # Prose, not a react action.
                schema=None)
    return {"config": route["config"], "question": q["id"], "kind": q["kind"],
            "decision": q["decision"], "effort": route["effort"], **row}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--models", nargs="+", required=True,
                    help="names under measure/models/, without .yaml")
    ap.add_argument("--only", nargs="*", default=None,
                    help="question ids; default is all of them")
    ap.add_argument("--out", type=Path,
                    default=HERE / f"answers_{time.strftime('%Y-%m-%dT%H-%M-%SZ', time.gmtime())}.jsonl")
    ap.add_argument("--timeout", type=int, default=300)
    args = ap.parse_args()

    method = load_workflow(REPO / "workflows" / "claims_audit" / "method" / "METHOD.md")
    spec = yaml.safe_load(QUESTIONS.read_text(encoding="utf-8"))
    qs = [q for q in spec["questions"]
          if not args.only or q["id"] in args.only]
    if not qs:
        raise SystemExit(f"no such question id in {QUESTIONS}")
    routes = [model_route(m) for m in args.models]

    print(f"{len(qs)} questions x {len(routes)} models, "
          f"method as delivered ({len(method):,} chars)")
    for r in routes:
        print(f"  {r['config']:26} {r['model']} @ {r['effort']}  {r['url']}")

    with args.out.open("w", encoding="utf-8") as fh:
        for q in qs:
            for r in routes:
                row = ask(r, method, q, args.timeout)
                fh.write(json.dumps(row) + "\n")
                fh.flush()
                status = row.get("status")
                head = (row.get("content") or row.get("err") or "")[:70]
                print(f"  {q['id']:32} {r['config']:24} {status} "
                      f"{head.replace(chr(10), ' ')}")
    print(f"\n{args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
