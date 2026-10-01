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
        lic = Counter(" OR ".join(x["licences"]) if x["licences"] else None for x in comps)
        none = lic.pop(None, 0)
        if not lic:
            out += ["**Declared licences.** The dependency files record no licence "
                    f"for any of the {len(comps)} components; their licences were "
                    "not determined.", ""]
        else:
            out += ["**Declared licences.** " + ", ".join(
                f"{k} {n}" for k, n in sorted(lic.items(), key=lambda kv: (-kv[1], kv[0])))
                + "." + (f" For {none} components the dependency file records no "
                         "licence; their licences were not determined." if none else ""), ""]

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
                   f"{_cell(' OR '.join(x['licences']) or 'not recorded')} | "
                   f"{_cell(', '.join(x['files']))} |")
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
