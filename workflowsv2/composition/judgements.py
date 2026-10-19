#!/usr/bin/env python3
"""The practice's reading of each licence the scan flagged, and what the
report says about the components' licences. No model.

    python3 workflowsv2/composition/judgements.py --engagement <name>
        lists the flagged licences and whether each has been read
    python3 workflowsv2/composition/judgements.py --engagement <name> \\
        --component <name>@<version> --verdict "conditions apply" \\
        --note "<one sentence>" --by <email>
        records a reading

WHY. A flag says a person should read the licence, not that it is a problem:
whether it matters depends on what the buyer will do with the software
(Bruce, 2026-10-19). The reading is recorded here, shown in the components
appendix and summed up in one line under the executive summary, and release
is refused while a flagged licence has not been read.

THE RECORD is `composition/licence_judgements.json` in the engagement,
beside the scans and not inside one, so a reading survives a rescan of the
same component at the same version. One entry per component, keyed
`<name>@<version>`: verdict, note, who, when.
"""
from __future__ import annotations

import argparse
import datetime
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

REPO = Path(__file__).resolve().parents[2]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2.composition import licences                    # noqa: E402

FILENAME = "licence_judgements.json"
#: What the practice found on reading a flagged licence.
VERDICTS = ("no issue", "conditions apply", "issue")


def key(comp: Dict[str, Any]) -> str:
    return f"{comp['name']}@{comp['version'] or comp['version_range']}"


def path(scan_dir: Path) -> Path:
    """The record for the engagement a scan directory belongs to."""
    return Path(scan_dir).parent / FILENAME


def load(scan_dir: Path) -> Dict[str, Any]:
    p = path(scan_dir)
    return json.loads(p.read_text(encoding="utf-8")) if p.is_file() else {}


def flagged(scan: Dict[str, Any]) -> List[Dict[str, Any]]:
    """The components whose licence a person has to read."""
    return [c for c in scan["components"] if "licence" in c and licences.to_read(c)]


def unread(scan: Dict[str, Any]) -> List[Dict[str, Any]]:
    got = scan.get("judgements") or {}
    return [c for c in flagged(scan) if key(c) not in got]


def release_problem(scan: Optional[Dict[str, Any]]) -> Optional[str]:
    """Why the report cannot be released yet, or None."""
    left = unread(scan) if scan else []
    if not left:
        return None
    return (f"{len(left)} flagged licence(s) not yet read: "
            + ", ".join(key(c) for c in left)
            + ". Record a reading with workflowsv2/composition/judgements.py.")


def summary_line(scan: Dict[str, Any]) -> List[str]:
    """The line under the executive summary: the licences of the third-party
    components, with every licence the practice read as raising conditions or
    an issue named. Empty for a scan made before the licence lookup."""
    comps = [c for c in scan["components"] if "licence" in c]
    if not comps:
        return []
    third = [c for c in comps if c["licence"]["from"] != "local"]
    if not third:
        return []
    got = scan.get("judgements") or {}
    permissive = sum(1 for c in third if c["licence"]["class"] == "permissive"
                     and not licences.to_read(c))
    read = [c for c in flagged(scan) if key(c) in got]
    not_read = [c for c in flagged(scan) if key(c) not in got]
    unlooked = sum(1 for c in third if c["licence"]["flags"] and not licences.to_read(c))
    if permissive == len(third):
        s = f"All {len(third)} third-party components declare permissive licences."
    else:
        s = f"{permissive} of the {len(third)} third-party components declare permissive licences."
    if read:
        n = {v: [c for c in read if got[key(c)]["verdict"] == v] for v in VERDICTS}
        parts = []
        if n["no issue"]:
            k = len(n["no issue"])
            parts.append(f"{k} raise{'s' if k == 1 else ''} no issue")
        if n["conditions apply"]:
            parts.append("conditions apply to " + _and([c["name"] for c in n["conditions apply"]]))
        parts += [f"{c['name']} raises an issue" for c in n["issue"]]
        s += (f" The practice read the {_n(len(read), 'licence', 'licences')} the scan "
              "flagged: " + _and(parts) + ".")
    if not_read:
        s += (f" {_n(len(not_read), 'flagged licence is', 'flagged licences are')} not yet "
              f"read: {', '.join(key(c) for c in not_read)}.")
    if unlooked:
        s += f" {unlooked} were not looked up, and their licences are not known."
    s += " Each licence is as its author declared it; this is not a legal opinion."
    if flagged(scan) or unlooked:
        s += " The components appendix sets out each."
    return ["**The components' licences.** " + s, ""]


def _n(k: int, one: str, many: str) -> str:
    return f"{k} {one if k == 1 else many}"


def _and(xs: List[str]) -> str:
    return xs[0] if len(xs) == 1 else ", ".join(xs[:-1]) + " and " + xs[-1]


def worklist_rows(scan: Optional[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Worklist items: each flagged licence not yet read (blocking, since
    release is refused until it is), and the components not looked up."""
    if not scan:
        return []
    rows = [{"stage": "composition", "code": "licence_not_read", "severity": "blocking",
             "source": "composition",
             "text": f"{key(c)}: licence {c['licence']['expression'] or 'none found'} — "
                     f"{'; '.join(c['licence']['flags'])}. Read it (components.json has the "
                     "archive's declaration and its licence files' first lines, where the "
                     "archive was read) and record a reading with judgements.py."}
            for c in unread(scan)]
    unlooked = [c for c in scan["components"]
                if "licence" in c and c["licence"]["flags"] and not licences.to_read(c)]
    if unlooked:
        rows.append({"stage": "composition", "code": "licence_not_looked_up", "severity": "check",
                     "source": "composition",
                     "text": f"{len(unlooked)} component(s) were not looked up, so their licence "
                             "is not known: " + "; ".join(
                                 f"{key(c)} ({c['licence']['flags'][0].split(': ', 1)[1]})"
                                 for c in unlooked)
                             + ". Ask the seller for the versions in use, or read their licences."})
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engagement", required=True)
    ap.add_argument("--component", help="<name>@<version>, as the listing shows it")
    ap.add_argument("--verdict", choices=VERDICTS)
    ap.add_argument("--note", help="one sentence: what the licence asks and whether it bears on the buyer")
    ap.add_argument("--by", help="who read it")
    args = ap.parse_args()
    from workflowsv2.composition import scan as composition
    eng = REPO / "workflowsv2" / "claims_audit" / "engagements" / args.engagement
    s = composition.latest(eng)
    if not s:
        raise SystemExit(f"{args.engagement}: no composition scan")
    if not args.component:
        got = s["judgements"]
        for c in flagged(s):
            j = got.get(key(c))
            print(f"{key(c)}  {c['licence']['expression'] or 'none found'}  "
                  f"[{'; '.join(c['licence']['flags'])}]  "
                  + (f"READ: {j['verdict']} — {j['note']} ({j['by']})" if j else "NOT READ"))
        return 0
    if not (args.verdict and args.note and args.by):
        raise SystemExit("--verdict, --note and --by are all needed to record a reading")
    if args.component not in {key(c) for c in flagged(s)}:
        raise SystemExit(f"{args.component} is not a flagged component of the latest scan")
    got = load(Path(s["dir"]))
    got[args.component] = {"verdict": args.verdict, "note": args.note.strip(), "by": args.by,
                           "at": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")}
    from utils.file_utils import atomic_write_json
    atomic_write_json(path(Path(s["dir"])), got, indent=1)
    print(f"recorded: {args.component} — {args.verdict}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
