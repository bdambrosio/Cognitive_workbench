"""The report appendix for a composition scan: computed from the scan record,
no model. Called by workflowsv2/audit_report/render.py when the engagement
enables composition analysis and has a scan.

The vulnerability matches appear as one count, with what a match does and
does not establish (Bruce, 2026-09-22: a listed match reads as a problem to a
buyer, and most are never checked for use). The list stays in matches.json.
"""
from __future__ import annotations

from collections import Counter
from typing import Any, Dict, List

from workflowsv2.composition import licences

#: Plain names for syft's component types; an unlisted type is shown as syft
#: names it.
KINDS = {
    "rust-crate": "Rust crate",
    "npm": "npm package",
    "python": "Python package",
    "go-module": "Go module",
    "java-archive": "Java archive",
    "gem": "Ruby gem",
    "php-composer": "PHP Composer package",
    "dotnet": ".NET package",
    "github-action": "GitHub Action",
    "github-action-workflow": "GitHub workflow",
}
#: Kinds that the project's build and test machinery uses, not the running
#: software; said once above the table instead of on every row.
CI_KINDS = ("github-action", "github-action-workflow")
#: Plain names for gitleaks rule ids; an unlisted rule is shown by its id with
#: the hyphens taken out. gitleaks's own descriptions assert harm ("could lead
#: to data breaches") that a match does not establish.
SECRET_KINDS = {"generic-api-key": "generic API key or token",
                "private-key": "private key",
                "gcp-api-key": "Google Cloud API key",
                "aws-access-token": "AWS access key",
                "github-pat": "GitHub personal access token",
                "slack-bot-token": "Slack bot token",
                "stripe-access-token": "Stripe key",
                "jwt": "JSON web token"}


def _n(k: int, one: str, many: str) -> str:
    return f"{k} {one if k == 1 else many}"


def _cell(x: Any) -> str:
    return str(x if x is not None else "").replace("|", "\\|").replace("\n", " ")


def render(scan: Dict[str, Any]) -> List[str]:
    meta, comps = scan["meta"], scan["components"]
    c = meta["counts"]
    db = meta.get("db") or {}
    out = ["## Appendix — components and licences", "",
           "The third-party components the target's dependency files declare, "
           "listed by programs, not by the review. Nothing here was tested "
           "against a claim, and nothing here is a finding.", ""]

    files = meta.get("files") or {}
    out += ["**What the scan read.** " + (
        ", ".join(f"`{f}` ({n})" for f, n in files.items()) + "."
        if files else "No dependency file the scanning program recognises. "
        "No components were listed, and none could be matched against known "
        "vulnerabilities."), ""]
    out += ["**What it did not examine.** Code copied into the repository "
            "without a dependency file; the contents of container images, which "
            "were not fetched; services the software calls over the network; "
            "and any dependency file of a kind the scanning program does not "
            "recognise.", ""]
    if comps:
        fixed = len(comps) - c.get("version_range", 0)
        if c.get("version_range"):
            out += [f"**Versions not fixed.** {c['version_range']} of the "
                    f"{len(comps)} components are declared with a range of "
                    "versions, not one version; the version in use cannot be "
                    "known from the materials. They are listed with their range "
                    "and were not matched against known vulnerabilities.", ""]
        out += ["**Known vulnerabilities.** " + (
            f"{c['matches']} published vulnerabilities match {c['matched_components']} "
            f"of the {fixed} components with a fixed version, by name and version. "
            "A match means only that the recorded version falls in a range an "
            "advisory names. Nobody examined whether those components run in the "
            "software as delivered, or whether the affected code is ever called, "
            "so a match is not a finding about the target." if c.get("matches") else
            f"No published vulnerability matches the {fixed} components with a "
            "fixed version, by name and version. That is a statement about the "
            "advisory database on the date below, not an assurance that none "
            "exists."), ""]
        if any("licence" in x for x in comps):
            out += _licences(comps)
        else:
            out += _lockfile_licences(comps)

    out += [f"**Programs.** Syft {meta.get('syft')}, Grype {meta.get('grype')}; "
            f"vulnerability database built {(db.get('built') or '')[:10]} "
            f"(schema {db.get('schema')}), held fixed for the scan. "
            + (f"Target commit {meta['target_rev'][:12]}, scanned "
               if meta.get("target_rev") else "The target is not a git checkout; scanned ")
            + f"{meta.get('scanned_at', '')[:10]}.", ""]

    if not comps:
        return out
    ci = sum(1 for x in comps if x["type"] in CI_KINDS)
    if ci:
        out += [f"The {ci} GitHub Actions and workflows are used by the project's "
                "continuous-integration workflows to build and test it, not by the "
                "running software.", ""]
    out += ["| kind | component | version | licence | declared in |",
            "|---|---|---|---|---|"]
    for x in comps:
        out.append(f"| {_cell(KINDS.get(x['type'], x['type']))} | {_cell(x['name'])} | "
                   f"{_cell(x['version'] or x['version_range'])} | "
                   f"{_cell(_licence_cell(x))} | "
                   f"{_cell(', '.join(x['files']))} |")
    return out


def _licence_cell(x: Dict[str, Any]) -> str:
    lic = x.get("licence")
    if lic is None:                                   # a scan made before the lookup
        return " OR ".join(x["licences"]) or "not recorded"
    if lic["from"] == "local":
        return "part of the target"
    return lic["expression"] or "not found"


#: Plain words for what each class of licence asks; said once, not per row.
CLASS_NOTE = ("A permissive licence (MIT, Apache-2.0, BSD and the like) asks for "
              "little beyond keeping its notice. A copyleft licence attaches "
              "conditions to software that includes the component: a weak one to "
              "changes to the component itself, a strong one to the whole program "
              "when it is distributed, and a network one even when the program is "
              "only run as a service for others.")


#: Why a component was not looked up, as (plural, singular) predicates.
UNLOOKED = {"not from a public registry": ("are not from a public registry",
                                           "is not from a public registry"),
            "no single version": ("are declared with a range of versions",
                                  "is declared with a range of versions"),
            "a kind the lookup does not cover": ("are of a kind the lookup does not cover",
                                                 "is of a kind the lookup does not cover")}


def _lockfile_licences(comps: List[Dict[str, Any]]) -> List[str]:
    """A scan made before the licence lookup: the licences the dependency
    files record, and nothing else."""
    lic = Counter(" OR ".join(x["licences"]) if x["licences"] else None for x in comps)
    none = lic.pop(None, 0)
    if not lic:
        return ["**Declared licences.** The dependency files record no licence "
                f"for any of the {len(comps)} components; their licences were "
                "not determined.", ""]
    return ["**Declared licences.** " + ", ".join(
        f"{k} {n}" for k, n in sorted(lic.items(), key=lambda kv: (-kv[1], kv[0])))
        + "." + (f" For {none} components the dependency file records no "
                 "licence; their licences were not determined." if none else ""), ""]


def _licences(comps: List[Dict[str, Any]]) -> List[str]:
    """The licence of every third-party component, by what it asks of the
    buyer, the flagged ones by name, and where the licences came from."""
    local = [x for x in comps if x["licence"]["from"] == "local"]
    third = [x for x in comps if x["licence"]["from"] != "local"]
    out: List[str] = []
    def kind(x):
        if x["licence"]["class"]:
            return x["licence"]["class"]
        return "not looked up" if x["licence"]["flags"] and not licences.to_read(x) else "none found"
    by = Counter(kind(x) for x in third)
    words = {"unknown": "with a licence identifier the scan does not recognise",
             "none found": "with no licence found", "not looked up": "not looked up"}
    if third and set(by) == {"permissive"}:
        head = f"All {len(third)} third-party components declare permissive licences."
    elif third:
        order = list(licences.CLASSES) + ["none found", "not looked up"]
        head = (f"Of the {len(third)} third-party components: "
                + "; ".join(f"{by[k]} {words.get(k, k)}" for k in order if by.get(k)) + ".")
    else:
        head = "Every component listed is part of the target itself."
    out += ["**Licences.** " + head + " " + CLASS_NOTE, ""]

    flagged = [x for x in third if licences.to_read(x)]
    unlooked = Counter(x["licence"]["flags"][0].split(": ", 1)[1] for x in third
                       if x["licence"]["flags"] and not licences.to_read(x))
    if flagged:
        out += ["**Licences to read.** " + "; ".join(
            f"{x['name']} {x['version'] or x['version_range'] or ''}".strip()
            + " (" + ", ".join(([x["licence"]["expression"]] if x["licence"]["expression"] else [])
                               + x["licence"]["flags"]) + ")" for x in flagged)
            + ". These need their licence text read by a person; the review did "
              "not read it.", ""]
    if unlooked:
        out += ["**Not looked up.** " + "; ".join(
            f"{_n(n, 'component', 'components')} {UNLOOKED[why][n == 1]}"
            for why, n in sorted(unlooked.items()))
            + ". Their licences were not looked up and are not known from the materials.", ""]
    if local:
        names = sorted({x["name"] for x in local})
        out += [f"**Part of the target.** {', '.join(names)} "
                + ("is the target's own code, not a third-party component, and was "
                   "not looked up." if len(names) == 1 else
                   "are the target's own code, not third-party components, and were "
                   "not looked up."), ""]

    src = Counter(x["licence"]["from"] for x in third)
    dates = sorted(x["licence"]["at"][:10] for x in third if x["licence"].get("at"))
    said = []
    if src.get("lockfile"):
        said.append(f"for {src['lockfile']}, the dependency file")
    if src.get("deps.dev"):
        said.append(f"for {src['deps.dev']}, the licence the package declared to its "
                    "registry for that version, as deps.dev reports it")
    if src.get("github"):
        said.append(f"for {src['github']} GitHub Actions, the current licence of "
                    "the Action's repository, as GitHub reports it")
    if said:
        said = "; ".join(said)
        out += ["**Where the licences come from.** " + said[0].upper() + said[1:]
                + (f". Looked up {dates[0]}" if dates and dates[0] == dates[-1] else
                   f". Looked up between {dates[0]} and {dates[-1]}" if dates else "")
                + ". A declared licence is what the component's author stated. Files "
                  "inside a component can carry other licences, and none of this is a "
                  "legal opinion.", ""]
    return out


def render_secrets(scan: Dict[str, Any]) -> List[str]:
    """The credentials appendix: counts by kind, no locations and no values.
    The rows stay in secrets.json, as the vulnerability list stays in
    matches.json. Empty for a scan made before the secrets scan was added."""
    rows, meta = scan.get("secrets"), scan["meta"]
    if rows is None:
        return []
    out = ["## Appendix — credentials in the files and history", "",
           "Text in the target that matches a credential pattern (an API key, "
           "a token, a private key), found by a program, not by the review. "
           "Nothing here was tested against a claim, and nothing here is a "
           "finding.", ""]

    def kinds(where: str) -> str:
        n = Counter(SECRET_KINDS.get(x["rule"], (x["rule"] or "").replace("-", " "))
                    for x in rows if x["where"] == where)
        return ", ".join(f"{k} {v}" for k, v in sorted(n.items(), key=lambda kv: (-kv[1], kv[0])))

    files = [x for x in rows if x["where"] == "files"]
    out += ["**In the files as delivered.** " + (
        f"{_n(len(files), 'match', 'matches')} in "
        f"{_n(len({x['file'] for x in files}), 'file', 'files')}: {kinds('files')}."
        if files else "No text matches a credential pattern."), ""]
    if meta.get("history_scanned"):
        hist = [x for x in rows if x["where"] == "history"]
        out += ["**In the git history only.** " + (
            f"{_n(len(hist), 'further match', 'further matches')} in earlier "
            f"commits: {kinds('history')}. "
            "A credential committed and later removed stays readable to anyone "
            "given the repository." if hist else
            "No further match in earlier commits."), ""]
    else:
        out += ["**In the git history.** The target is not a git checkout; its "
                "history was not examined.", ""]
    out += ["**What a match is.** Text that fits a credential pattern. Nobody "
            "checked whether it is a real credential or whether it still works; "
            "examples and test values match as well.", "",
            f"**Program.** Gitleaks {meta.get('gitleaks')}, with that version's "
            "default rules held fixed for the scan. Settings and exemptions "
            "written into the target itself were ignored.", ""]
    return out
